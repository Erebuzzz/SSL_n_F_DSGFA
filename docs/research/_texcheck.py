"""Static sanity checks for research_gaps.tex.

Not a substitute for a real compile. Checks delimiter balance per equation,
citation/label resolution, common backslash typos, the per-page notation-key
machinery, and longtable column counts.
"""

import re
import sys
from pathlib import Path

args = [a for a in sys.argv[1:] if a != "--budget"]
SHOW_BUDGET = "--budget" in sys.argv[1:]
TEX = Path(args[0]) if args else Path(__file__).with_name("research_gaps.tex")
src = TEX.read_text(encoding="utf-8")
lines = src.split("\n")

problems = []


def strip_comments(text):
    """Drop % comments so the checks do not parse commented-out examples."""
    out = []
    for line in text.split("\n"):
        match = re.search(r"(?<!\\)%", line)
        out.append(line if match is None else line[:match.start()])
    return "\n".join(out)


code = strip_comments(src)

# 1. \left ... \right balance, checked per display equation.
depth = 0
eq_start = 0
for num, line in enumerate(lines, 1):
    if line.lstrip().startswith("%"):
        continue
    if re.search(r"\\begin\{(equation|align|gather)\*?\}", line):
        depth, eq_start = 0, num
    for kind in re.findall(r"\\(left|right)(?![a-zA-Z])", line):
        depth += 1 if kind == "left" else -1
        if depth < 0:
            problems.append(f"line {num}: \\right with no matching \\left")
            depth = 0
    if re.search(r"\\end\{(equation|align|gather)\*?\}", line) and depth != 0:
        problems.append(
            f"lines {eq_start}-{num}: unbalanced \\left/\\right, net {depth:+d}"
        )
        depth = 0

# 2. Citations and cross-references.
def keys(pattern):
    found = set()
    for match in re.finditer(pattern, code):
        found.update(k.strip() for k in match.group(1).split(","))
    return found

cited = keys(r"\\cite\{([^}]*)\}")
defined = keys(r"\\bibitem\{([^}]*)\}")
labels = keys(r"\\label\{([^}]*)\}")
referenced = keys(r"\\(?:c|C)?ref\{([^}]*)\}") | keys(r"\\eqref\{([^}]*)\}")

for k in sorted(cited - defined):
    problems.append(f"cited but no \\bibitem: {k}")
for k in sorted(defined - cited):
    problems.append(f"\\bibitem never cited: {k}")
for k in sorted(referenced - labels):
    problems.append(f"referenced but no \\label: {k}")

# 3. Environment balance.
counts = {}
for kind, env in re.findall(r"\\(begin|end)\{([^}]*)\}", code):
    counts[env] = counts.get(env, 0) + (1 if kind == "begin" else -1)
for env, net in sorted(counts.items()):
    if net:
        problems.append(f"unbalanced environment {env}: net {net:+d}")

# 4. Common typo: a known macro appearing without its leading backslash.
# Only macros that are not also ordinary English words, and only after
# stripping label/ref/newcommand arguments where the bare name is legitimate.
macros = ["qquad", "cdot", "frac", "sgn", "leq", "geq", "phi", "kappa", "beta"]
strip = re.compile(r"\\(?:label|[cC]?ref|eqref|cite|newcommand|operatorname)\{[^}]*\}")
for num, line in enumerate(lines, 1):
    if line.lstrip().startswith("%"):
        continue
    cleaned = strip.sub("", line)
    for macro in macros:
        if re.search(r"(?<![\\A-Za-z]){}(?![A-Za-z])".format(macro), cleaned):
            problems.append(f"line {num}: bare '{macro}' missing backslash?")


# 5. Per-page notation key: every context used must be declared, and every
# declared context must be used. A missing declaration silently prints an
# empty key rather than raising an error, so a compile would not catch it.
def brace_body(text, start):
    """Return the balanced {...} group beginning at index start, or None."""
    if start >= len(text) or text[start] != "{":
        return None
    depth = 0
    for i in range(start, len(text)):
        if text[i] == "{" and text[i - 1] != "\\":
            depth += 1
        elif text[i] == "}" and text[i - 1] != "\\":
            depth -= 1
            if depth == 0:
                return text[start + 1:i]
    return None


declared = {}
for match in re.finditer(r"\\DeclareNotationKey\{([^}]*)\}", code):
    body = brace_body(code, match.end())
    if body is None:
        problems.append(f"\\DeclareNotationKey{{{match.group(1)}}}: unbalanced body braces")
    else:
        declared[match.group(1)] = body

used = keys(r"\\NotationContext\{([^}]*)\}")
for k in sorted(used - set(declared)):
    problems.append(f"\\NotationContext{{{k}}} has no \\DeclareNotationKey")
for k in sorted(set(declared) - used):
    problems.append(f"\\DeclareNotationKey{{{k}}} is never activated")

# Footer height guard. The key is set at \scriptsize across \textwidth, which
# fits roughly 115 characters per line, and the geometry leaves room for four.
# The threshold is 3.5 rather than 4 because this estimate strips macros, so it
# undercounts math-heavy keys, which set wider than their source length.
LINE_CHARS, MAX_LINES = 115, 3.5
budget = []
for name, body in sorted(declared.items()):
    visible = re.sub(r"\s+", " ", re.sub(r"\\[a-zA-Z]+\s*", "", body)).strip()
    est = len(visible) / LINE_CHARS
    budget.append((est, name, len(visible)))
    if est > MAX_LINES:
        problems.append(
            f"notation key '{name}' is about {est:.1f} lines, over the {MAX_LINES}-line footer budget"
        )
    if body.count("$") % 2:
        problems.append(f"notation key '{name}': odd number of $ delimiters")

# 6. longtable column counts. A row with the wrong number of & is a hard error.
def count_columns(spec):
    """Column count for a tabular preamble, skipping @/>/< insertion groups."""
    i = total = 0
    while i < len(spec):
        ch = spec[i]
        if ch in "@><!":
            group = brace_body(spec, i + 1)
            i += 2 + len(group) + 1 if group is not None else 1
            continue
        if ch in "pmb" and brace_body(spec, i + 1) is not None:
            total += 1
            i += 2 + len(brace_body(spec, i + 1)) + 1
            continue
        if ch in "lcrX":
            total += 1
        i += 1
    return total


for match in re.finditer(r"\\begin\{longtable\}", code):
    idx = match.end()
    if idx < len(code) and code[idx] == "[":
        idx = code.index("]", idx) + 1
    spec = brace_body(code, idx)
    if spec is None:
        problems.append("longtable: could not read the column specification")
        continue
    ncols = count_columns(spec)
    body_start = idx + len(spec) + 2
    end = code.index(r"\end{longtable}", body_start)
    body = code[body_start:end]
    for raw in body.split("\\\\"):
        row = raw.strip()
        if not row or row.startswith("%"):
            continue
        row = re.sub(r"\\(?:hline|endfirsthead|endhead|endfoot|endlastfoot)\b", "", row).strip()
        if not row:
            continue
        spans = sum(int(m.group(1)) - 1 for m in re.finditer(r"\\multicolumn\{(\d+)\}", row))
        got = row.count("&") + 1 + spans
        if got != ncols:
            snippet = row[:60].replace("\n", " ")
            problems.append(f"longtable row has {got} columns, expected {ncols}: {snippet}")

print(f"cite keys: {len(cited)}  bibitems: {len(defined)}  "
      f"labels: {len(labels)}  refs: {len(referenced)}  "
      f"notation keys: {len(declared)}")

if SHOW_BUDGET and budget:
    width = max(len(name) for _, name, _ in budget)
    print(f"\nfooter budget, {MAX_LINES} lines maximum:")
    for est, name, chars in sorted(budget, reverse=True):
        print(f"  {name:<{width}}  {chars:>4} chars  {est:4.1f} lines  {'#' * round(est * 10)}")
if problems:
    print(f"\n{len(problems)} problem(s):")
    for p in problems:
        print("  -", p)
    sys.exit(1)
print("\nno problems found")
