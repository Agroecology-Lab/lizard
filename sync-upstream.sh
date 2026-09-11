#!/usr/bin/env bash
# Sync Agroecology-Lab/lizard main with upstream zauberzeug/lizard.
# Rebases local commits on top of upstream, then force-pushes to origin.
set -euo pipefail

cd "$(dirname "$0")"

BRANCH="main"

if ! git remote get-url upstream >/dev/null 2>&1; then
    git remote add upstream https://github.com/zauberzeug/lizard.git
fi

git fetch upstream
git checkout "$BRANCH"

if ! git rebase "upstream/$BRANCH"; then
    echo
    echo "Rebase stopped due to conflicts."
    echo "Fix conflicts, then: git add <file> && git rebase --continue"
    echo "Or bail out: git rebase --abort"
    exit 1
fi

git push origin "$BRANCH" --force-with-lease

echo "Done. $BRANCH synced with upstream and pushed to origin."
