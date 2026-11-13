#!/usr/bin/env bash
#
# mayhem/test.sh — runs the known-answer tests built by mayhem/build.sh (build/mayhem/kat). They
# feed fixed inputs to upstream's edge-aware debayering and depth -> point cloud conversions and
# check the exact values that come out. No compilation here.
set -uo pipefail
[ -n "${SOURCE_DATE_EPOCH:-}" ] || unset SOURCE_DATE_EPOCH
cd "$SRC"

emit_ctrf() {
  local tool="$1" passed="$2" failed="$3" skipped="${4:-0}" pending="${5:-0}" other="${6:-0}"
  local tests=$(( passed + failed + skipped + pending + other ))
  cat > "${CTRF_REPORT:-$SRC/ctrf-report.json}" <<JSON
{
  "results": {
    "tool": { "name": "$tool" },
    "summary": {
      "tests": $tests,
      "passed": $passed,
      "failed": $failed,
      "pending": $pending,
      "skipped": $skipped,
      "other": $other
    }
  }
}
JSON
  printf 'CTRF {"results":{"tool":{"name":"%s"},"summary":{"tests":%d,"passed":%d,"failed":%d,"pending":%d,"skipped":%d,"other":%d}}}\n' \
    "$tool" "$tests" "$passed" "$failed" "$pending" "$skipped" "$other"
  [ "$failed" -eq 0 ]
}

KAT="$SRC/build/mayhem/kat"
if [ ! -x "$KAT" ]; then
  echo "missing $KAT — mayhem/build.sh did not produce it" >&2
  emit_ctrf "image_pipeline-kat" 0 1
  exit 1
fi

out=$("$KAT" 2>&1); rc=$?
echo "$out"
passed=$(printf '%s\n' "$out" | grep -c '^PASS ')
failed=$(printf '%s\n' "$out" | grep -c '^FAIL ')
# A crash, or fewer results than the suite defines, counts as a failure.
expected=12
if [ "$rc" -ne 0 ]; then failed=$((failed + 1)); fi
if [ $((passed + failed)) -lt "$expected" ]; then failed=$((failed + expected - passed - failed)); fi
emit_ctrf "image_pipeline-kat" "$passed" "$failed"
