#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/nuaa/ZHY/3DPlanner_FULL
TS=$(date +%Y%m%d_%H%M%S)
LOG="${P6A_LOG_DIR:-}"
if [ -z "$LOG" ]; then
  LOG=$(find "$ROOT/test-log" -maxdepth 1 -type d -name '*_p6a_jetson_deployment_prep' 2>/dev/null | sort | tail -1)
fi
if [ -z "$LOG" ]; then
  LOG="$ROOT/test-log/${TS}_p6a_jetson_deployment_prep"
  mkdir -p "$LOG"
fi

PKG="$ROOT/latest_p6a_jetson_deployment_prep_package.tar.gz"
TMP="/tmp/p6a_debug_package_${TS}"
rm -rf "$TMP"
mkdir -p "$TMP/log" "$TMP/root_docs" "$TMP/git"

if [ -d "$LOG" ]; then
  find "$LOG" -maxdepth 1 -type f \
    ! -name '*.tar.gz' \
    ! -name '*.bag' \
    ! -name '*.db3' \
    -exec cp -a {} "$TMP/log/" \;
fi

for path in \
  "$ROOT/DEPLOYMENT_P6A_JETSON_UBUNTU_GUIDE.md" \
  "$ROOT/MANUAL_GITHUB_UPLOAD_P5B_P5C.md" \
  "$ROOT/P5A_FINAL_BASELINE_README.md"; do
  [ -f "$path" ] && cp -a "$path" "$TMP/root_docs/"
done

GIT_CMD=(git --git-dir="$ROOT/.git_3dplanner_full" --work-tree="$ROOT")
{
  echo "# P6A Git State"
  "${GIT_CMD[@]}" branch --show-current 2>/dev/null || true
  "${GIT_CMD[@]}" rev-parse HEAD 2>/dev/null || true
  "${GIT_CMD[@]}" tag --points-at HEAD 2>/dev/null || true
  "${GIT_CMD[@]}" status --short 2>/dev/null || true
  "${GIT_CMD[@]}" remote -v 2>/dev/null || true
} > "$TMP/git/p6a_git_state.txt"

(
  cd "$TMP"
  tar -czf "$PKG" .
)

echo "P6A_DEBUG_PACKAGE_EXPORT=PASS"
echo "debug_package=$PKG"
