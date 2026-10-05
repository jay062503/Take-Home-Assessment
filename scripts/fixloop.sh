#!/usr/bin/env bash
# Regenerate the fix-loop before/after runs from their git tags, plus the readable diff.
#
#   scripts/fixloop.sh            # both runs + diff
#   scripts/fixloop.sh before     # only the before run
#   scripts/fixloop.sh after      # only the after run
#
# Each run checks the tagged commit out into a temporary git worktree and benchmarks it with that
# commit's own code and config, so "before" really is the old code, not a config flag on new code.
set -euo pipefail
ROOT="$(git -C "$(dirname "$0")/.." rev-parse --show-toplevel)"
cd "$ROOT"
PY="${PY:-$ROOT/.venv/bin/python}"
MANIFEST="${MANIFEST:-data/synthetic/apartment_a/manifest.yaml}"
WHICH="${1:-both}"

[ -f "$MANIFEST" ] || "$PY" -m propscan synth   # synthetic data is gitignored; regenerate it

run() {
  local tag="$1" out="$2" wt
  wt="$(mktemp -d)/wt"
  git worktree add --detach --quiet "$wt" "$tag"
  echo "[fixloop] $tag -> $out"
  (cd "$wt" && PYTHONPATH="$wt" "$PY" -c "import sys; from propscan.cli import main; sys.exit(main())" \
    bench "$ROOT/$MANIFEST" -o "$ROOT/$out")
  git worktree remove --force "$wt"
}

case "$WHICH" in
  before|both) run fixloop-before reports/synthetic_before ;;
esac
case "$WHICH" in
  after|both) run fixloop-after reports/synthetic_after ;;
esac
git diff fixloop-before fixloop-after -- propscan configs > docs/fixloop.diff
echo "[fixloop] diff: docs/fixloop.diff ($(wc -l < docs/fixloop.diff) lines)"
