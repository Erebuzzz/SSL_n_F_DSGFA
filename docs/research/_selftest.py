r"""Confirm _texcheck.py actually fires on faults it claims to catch."""
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).parent
SRC = (HERE / "research_gaps.tex").read_text(encoding="utf-8")
CHECK = HERE / "_texcheck.py"

MUTATIONS = [
    ("undeclared context",
     r"\NotationContext{core}", r"\NotationContext{bogus}",
     "has no \\DeclareNotationKey"),
    ("unused declaration",
     r"\NotationContext{moving}", r"",
     "is never activated"),
    ("longtable row with an extra column",
     r"$N$ & Number of sources \\", r"$N$ & Number & of sources \\",
     "expected 2"),
    ("over-long notation key",
     r"\DeclareNotationKey{status}{%",
     r"\DeclareNotationKey{status}{" + "padding " * 90 + "%",
     "over the 3.5-line footer budget"),
    ("odd math delimiter in a key",
     r"$n$ robots, $i\in\V", r"$n robots, $i\in\V",
     "odd number of $ delimiters"),
    ("unbalanced left/right",
     r"\left|\sin(2\pi\chi_k)\right|", r"\left|\sin(2\pi\chi_k)|",
     "unbalanced \\left/\\right"),
    ("dangling reference",
     r"\label{lem:basin-certificate}", r"\label{lem:basin-certificate-typo}",
     "referenced but no \\label"),
]

failures = 0
for name, old, new, expect in MUTATIONS:
    if SRC.count(old) < 1:
        print(f"SKIP  {name}: anchor not found")
        failures += 1
        continue
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "mutated.tex"
        path.write_text(SRC.replace(old, new, 1), encoding="utf-8")
        run = subprocess.run([sys.executable, str(CHECK), str(path)],
                             capture_output=True, text=True)
    caught = expect in run.stdout
    print(f"{'PASS' if caught else 'FAIL'}  {name}")
    if not caught:
        failures += 1
        print("      expected to see:", expect)
        print("      got:", run.stdout.strip()[:400])

clean = subprocess.run([sys.executable, str(CHECK)], capture_output=True, text=True)
if clean.returncode != 0:
    print("FAIL  unmutated document should be clean")
    failures += 1
else:
    print("PASS  unmutated document is clean")

print(f"\n{len(MUTATIONS) + 1 - failures}/{len(MUTATIONS) + 1} self-tests passed")
sys.exit(1 if failures else 0)
