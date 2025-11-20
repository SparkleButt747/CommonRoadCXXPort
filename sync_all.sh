#!/usr/bin/env bash
set -euo pipefail

REMOTE="${1:-origin}"
FORCE="${2:---no-force}"   # second arg: --force or --no-force

# Remember where you started
START_BRANCH="$(git symbolic-ref --quiet --short HEAD 2>/dev/null || true)"

echo "Fetching ALL heads/tags from '$REMOTE' and pruning..."
git fetch --prune "$REMOTE" "+refs/heads/*:refs/remotes/$REMOTE/*" --tags

# Enumerate branches directly from remote
REMOTE_REFS="$(git ls-remote --heads "$REMOTE" | awk '{print $2}')"
[ -n "$REMOTE_REFS" ] || { echo "No remote heads found on $REMOTE"; exit 0; }

# Create missing locals and ensure tracking
IFS=$'\n'
for REF in $REMOTE_REFS; do
  B="${REF#refs/heads/}"
  echo "Ensuring local branch for '$REMOTE/$B'..."
  if git show-ref --verify --quiet "refs/heads/$B"; then
    git branch --set-upstream-to="$REMOTE/$B" "$B" >/dev/null 2>&1 || true
  else
    git branch --track "$B" "$REMOTE/$B" >/dev/null
    echo "  created local '$B' tracking '$REMOTE/$B'"
  fi
done
unset IFS

# Sync each local branch
for B in $(git for-each-ref --format='%(refname:short)' refs/heads); do
  if git rev-parse --verify --quiet "$REMOTE/$B" >/dev/null; then
    echo "Syncing '$B'..."
    if [ "$FORCE" = "--force" ]; then
      git reset --hard >/dev/null 2>&1 || true
      git clean -fd >/dev/null 2>&1 || true
      git switch -q -f "$B"
      git reset --hard "$REMOTE/$B" >/dev/null 2>&1 || true
      git clean -fd >/dev/null 2>&1 || true
      echo "  forced to match '$REMOTE/$B'"
    else
      git switch -q "$B" || {
        echo "  skipped (dirty working tree; use --force to override)"; 
        continue;
      }
      if git merge --ff-only "$REMOTE/$B" >/dev/null 2>&1; then
        echo "  fast-forwarded"
      else
        echo "  skipped (non-fast-forward required)"
      fi
    fi
  fi
done

# Return to start
if [ -n "${START_BRANCH:-}" ] && git show-ref --verify --quiet "refs/heads/$START_BRANCH"; then
  git switch -q "$START_BRANCH"
fi

echo "-----"
git branch -vv
echo "-----"
echo "Done. Mode: $FORCE"
