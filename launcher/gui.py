"""Tkinter GUI front-end for the unified launcher.

Thin layer over ``launcher.core``: every widget reads/writes a single
``LauncherState``; the theory readout (gain condition, epsilon bound, informed
counts) recomputes live on any change; the Run button dispatches on a worker
thread so the window stays responsive.

Launch with:  ``python -m launcher``
"""

from __future__ import annotations

import queue
import threading
from pathlib import Path

import numpy as np

from .core import (
    MODES_BY_PLATFORM,
    NOISE_MODELS,
    PLATFORMS,
    SGN_MODES,
    LauncherState,
    compute_readout,
    default_positions,
    dispatch,
    valid_modes,
)

try:  # tkinter is stdlib but may be absent on headless/server Pythons
    import tkinter as tk
    from tkinter import messagebox, ttk
    _TK_AVAILABLE = True
except Exception:  # pragma: no cover - only on tk-less builds
    _TK_AVAILABLE = False


# scalar fields shown as simple labelled entries: (attr, label, type)
_SCALARS = [
    ("kappa", "kappa", float),
    ("radius", "R (formation radius)", float),
    ("dmax", "Dmax (sensing range)", float),
    ("alpha", "alpha (formation gain)", float),
    ("beta", "beta (localization gain)", float),
    ("dt", "dt (timestep)", float),
    ("duration", "duration [s]", float),
    ("seed", "seed", int),
    ("noise_std", "noise std", float),
    ("noise_bound", "noise bound (delta)", float),
    ("control_point_offset", "control-point offset r", float),
]


class LauncherGUI:
    """The main window. Construction alone does not enter the event loop."""

    def __init__(self, root: "tk.Tk", state: LauncherState | None = None) -> None:
        self.root = root
        self.state = state or LauncherState()
        self._result_queue: "queue.Queue" = queue.Queue()
        self._pos_entries: list[tuple[tk.Entry, tk.Entry, tk.IntVar]] = []
        # While True, the per-robot layout auto-follows the source (so moving the
        # source keeps every robot within sensing range). Any manual position edit
        # turns this off so the user's custom layout is preserved.
        self._positions_are_default = self.state.positions is None
        root.title("SGF Simulator — Launcher")
        self._build()
        self._sync_modes()
        self._refresh_readout()

    # -- layout ---------------------------------------------------------
    def _build(self) -> None:
        main = ttk.Frame(self.root, padding=8)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        left = ttk.LabelFrame(main, text="Configuration", padding=8)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        right = ttk.LabelFrame(main, text="Theory readout & run", padding=8)
        right.grid(row=0, column=1, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        r = 0
        # platform / mode / backend
        self.platform_var = tk.StringVar(value=self.state.platform)
        self.mode_var = tk.StringVar(value=self.state.mode)
        self.backend_var = tk.StringVar(value=self.state.coppelia_backend)
        self.noise_var = tk.StringVar(value=self.state.noise_model)
        self.sgn_var = tk.StringVar(value=self.state.sgn_mode)

        r = self._add_combo(left, r, "Platform", self.platform_var, PLATFORMS, self._on_platform)
        self.mode_combo, r = self._add_combo(
            left, r, "Mode", self.mode_var, valid_modes(self.state.platform),
            self._on_change, return_widget=True)
        r = self._add_combo(left, r, "CoppeliaSim backend", self.backend_var,
                            ("mock", "coppelia"), self._on_change)
        r = self._add_combo(left, r, "Noise", self.noise_var, NOISE_MODELS, self._on_change)
        r = self._add_combo(left, r, "Sgn controller", self.sgn_var, SGN_MODES, self._on_change)

        # source
        ttk.Label(left, text="source (x, y)").grid(row=r, column=0, sticky="w")
        self.src_x = ttk.Entry(left, width=8)
        self.src_y = ttk.Entry(left, width=8)
        self.src_x.insert(0, str(self.state.source[0]))
        self.src_y.insert(0, str(self.state.source[1]))
        self.src_x.grid(row=r, column=1, sticky="w")
        self.src_y.grid(row=r, column=2, sticky="w")
        self.src_x.bind("<FocusOut>", self._on_source_change)
        self.src_y.bind("<FocusOut>", self._on_source_change)
        r += 1

        # n with a rebuild button for the positions grid
        ttk.Label(left, text="n (robots)").grid(row=r, column=0, sticky="w")
        self.n_entry = ttk.Entry(left, width=8)
        self.n_entry.insert(0, str(self.state.n))
        self.n_entry.grid(row=r, column=1, sticky="w")
        ttk.Button(left, text="apply n", command=self._on_apply_n).grid(row=r, column=2, sticky="w")
        r += 1

        # scalar fields
        self.scalar_entries: dict[str, ttk.Entry] = {}
        for attr, label, _ in _SCALARS:
            ttk.Label(left, text=label).grid(row=r, column=0, sticky="w")
            e = ttk.Entry(left, width=12)
            e.insert(0, str(getattr(self.state, attr)))
            e.grid(row=r, column=1, columnspan=2, sticky="w")
            e.bind("<FocusOut>", self._on_change)
            self.scalar_entries[attr] = e
            r += 1

        # per-robot positions + informed grid (scrollable)
        posframe = ttk.LabelFrame(left, text="Per-robot: x, y, informed (1/0)", padding=4)
        posframe.grid(row=r, column=0, columnspan=3, sticky="nsew", pady=(6, 0))
        ttk.Button(posframe, text="reset to defaults",
                   command=self._reset_positions).grid(row=0, column=0, columnspan=3, sticky="w")
        self.pos_container = ttk.Frame(posframe)
        self.pos_container.grid(row=1, column=0, columnspan=3, sticky="nsew")
        self._build_position_grid()

        # right panel: readout + run
        self.readout = tk.Text(right, width=52, height=16, wrap="word")
        self.readout.grid(row=0, column=0, sticky="nsew")
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)
        btns = ttk.Frame(right)
        btns.grid(row=1, column=0, sticky="ew", pady=6)
        ttk.Button(btns, text="Refresh readout", command=self._refresh_readout).grid(row=0, column=0)
        self.run_btn = ttk.Button(btns, text="Run", command=self._on_run)
        self.run_btn.grid(row=0, column=1, padx=6)
        self.status = ttk.Label(right, text="ready")
        self.status.grid(row=2, column=0, sticky="w")

    def _add_combo(self, parent, row, label, var, values, callback, return_widget=False):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w")
        combo = ttk.Combobox(parent, textvariable=var, values=list(values),
                             state="readonly", width=18)
        combo.grid(row=row, column=1, columnspan=2, sticky="w")
        combo.bind("<<ComboboxSelected>>", callback)
        if return_widget:
            return combo, row + 1
        return row + 1

    def _build_position_grid(self) -> None:
        for child in self.pos_container.winfo_children():
            child.destroy()
        self._pos_entries.clear()
        positions = self.state.resolved_positions()
        informed = self.state.resolved_informed()
        for i in range(self.state.n):
            ttk.Label(self.pos_container, text=f"{i}").grid(row=i, column=0)
            ex = ttk.Entry(self.pos_container, width=7)
            ey = ttk.Entry(self.pos_container, width=7)
            ex.insert(0, f"{positions[i, 0]:g}")
            ey.insert(0, f"{positions[i, 1]:g}")
            ex.grid(row=i, column=1)
            ey.grid(row=i, column=2)
            var = tk.IntVar(value=int(informed[i]))
            chk = ttk.Checkbutton(self.pos_container, variable=var, command=self._on_change)
            chk.grid(row=i, column=3)
            ex.bind("<FocusOut>", self._on_position_edit)
            ey.bind("<FocusOut>", self._on_position_edit)
            self._pos_entries.append((ex, ey, var))

    # -- state sync -----------------------------------------------------
    def _pull_into_state(self) -> None:
        """Read every widget into self.state (tolerant of transient bad input)."""

        self.state.platform = self.platform_var.get()
        self.state.mode = self.mode_var.get()
        self.state.coppelia_backend = self.backend_var.get()
        self.state.noise_model = self.noise_var.get()
        self.state.sgn_mode = self.sgn_var.get()
        try:
            self.state.source = (float(self.src_x.get()), float(self.src_y.get()))
        except ValueError:
            pass
        for attr, _, caster in _SCALARS:
            try:
                setattr(self.state, attr, caster(self.scalar_entries[attr].get()))
            except (ValueError, KeyError):
                pass
        # per-robot grid
        if self._pos_entries and len(self._pos_entries) == self.state.n:
            pos = np.zeros((self.state.n, 2))
            mask = np.ones(self.state.n, dtype=int)
            ok = True
            for i, (ex, ey, var) in enumerate(self._pos_entries):
                try:
                    pos[i] = [float(ex.get()), float(ey.get())]
                    mask[i] = int(var.get())
                except ValueError:
                    ok = False
            if ok:
                self.state.positions = pos
                self.state.informed = mask

    def _sync_modes(self) -> None:
        modes = valid_modes(self.state.platform)
        self.mode_combo.configure(values=list(modes))
        if self.state.mode not in modes and modes:
            self.state.mode = modes[0]
            self.mode_var.set(modes[0])

    # -- callbacks ------------------------------------------------------
    def _on_platform(self, event=None) -> None:
        self.state.platform = self.platform_var.get()
        modes = valid_modes(self.state.platform)
        self.mode_combo.configure(values=list(modes))
        self.mode_var.set(modes[0] if modes else "")
        self.state.mode = self.mode_var.get()
        self._on_change()

    def _on_apply_n(self) -> None:
        try:
            n = int(self.n_entry.get())
        except ValueError:
            return
        if n < 3:
            messagebox.showerror("invalid n", "n must be >= 3")
            return
        self.state.n = n
        self.state.positions = default_positions(n, self.state.source)
        self.state.informed = np.ones(n, dtype=int)
        self._positions_are_default = True
        self._build_position_grid()
        self._on_change()

    def _reset_positions(self) -> None:
        self.state.positions = default_positions(self.state.n, self.state.source)
        self.state.informed = np.ones(self.state.n, dtype=int)
        self._positions_are_default = True
        self._build_position_grid()
        self._on_change()

    def _on_source_change(self, event=None) -> None:
        try:
            self.state.source = (float(self.src_x.get()), float(self.src_y.get()))
        except ValueError:
            self._on_change()
            return
        # If the layout is still the auto default, slide it to follow the source so
        # every robot keeps starting within sensing range. Custom layouts are left
        # untouched (a runtime warning fires later if that leaves nobody informed).
        if self._positions_are_default:
            self.state.positions = default_positions(self.state.n, self.state.source)
            self._build_position_grid()
        self._on_change()

    def _on_position_edit(self, event=None) -> None:
        self._positions_are_default = False
        self._on_change()

    def _on_change(self, event=None) -> None:
        self._pull_into_state()
        self._refresh_readout()

    def _refresh_readout(self) -> None:
        self._pull_into_state()
        self.readout.delete("1.0", "end")
        problems = self.state.validate()
        lines = [f"Platform : {self.state.platform}", f"Mode     : {self.state.mode}",
                 f"n        : {self.state.n}", ""]
        if problems:
            lines.append("CONFIG PROBLEMS:")
            lines.extend(f"  - {p}" for p in problems)
            lines.append("")
        try:
            readout = compute_readout(self.state)
            lines.extend(readout.as_lines())
        except Exception as exc:  # keep the GUI alive on any transient error
            lines.append(f"(readout unavailable: {exc})")
        self.readout.insert("1.0", "\n".join(lines))

    # -- run ------------------------------------------------------------
    def _on_run(self) -> None:
        self._pull_into_state()
        problems = self.state.validate()
        if problems:
            messagebox.showerror("invalid configuration", "\n".join(problems))
            return
        self.run_btn.configure(state="disabled")
        self.status.configure(text="running...")
        worker = threading.Thread(target=self._run_worker, daemon=True)
        worker.start()
        self.root.after(200, self._poll_worker)

    def _run_worker(self) -> None:
        try:
            result = dispatch(self.state)
            self._result_queue.put(("ok", result))
        except Exception as exc:  # noqa: BLE001 - surface any dispatch failure
            self._result_queue.put(("error", str(exc)))

    def _poll_worker(self) -> None:
        try:
            kind, payload = self._result_queue.get_nowait()
        except queue.Empty:
            self.root.after(200, self._poll_worker)
            return
        self.run_btn.configure(state="normal")
        if kind == "error":
            self.status.configure(text="failed")
            messagebox.showerror("run failed", payload)
            return
        self.status.configure(text="done")
        out = payload.get("output_dir", "?")
        msg = f"Run complete.\nOutput: {out}"
        if "summary" in payload:
            v = payload["summary"].get("validation", {})
            msg += (f"\ninside_bound: {v.get('inside_bound')}"
                    f"\nmin_n_informed: {v.get('min_n_informed')}")
        if payload.get("platform") == "MATLAB":
            msg += f"\ncommand:\n{payload.get('command')}"
            if payload.get("returncode") not in (0, None):
                msg += f"\n(returncode {payload.get('returncode')})"
        messagebox.showinfo("run complete", msg)


def main() -> int:
    if not _TK_AVAILABLE:
        print("tkinter is not available in this Python build; the GUI cannot start.")
        print("The launcher core (launcher.core) still works headlessly.")
        return 1
    root = tk.Tk()
    LauncherGUI(root)
    root.mainloop()
    return 0
