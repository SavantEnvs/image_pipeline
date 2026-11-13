#!/usr/bin/env bash
# mayhem/build.sh — builds the fuzz harnesses, their -standalone reproducers and the KAT oracle.
#
# image_pipeline's nodes need rclcpp, which Debian does not ship. The nodes' real work lives in
# library code that does not: image_proc/src/edge_aware.cpp and depth_image_proc/src/conversions.cpp,
# plus sensor_msgs' point_cloud2_iterator.hpp and image_geometry's pinhole_camera_model.cpp (both
# pinned by commit in the Dockerfile). The generated message structs are replaced by plain-struct
# shims in mayhem/shim/. Everything else compiled here is upstream's own source, unmodified.
set -euo pipefail

[ -n "${SOURCE_DATE_EPOCH:-}" ] || unset SOURCE_DATE_EPOCH
: "${SANITIZER_FLAGS=-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer}"
: "${DEBUG_FLAGS:=-g -gdwarf-3}"
: "${CC:=clang}" ; : "${CXX:=clang++}" ; : "${LIB_FUZZING_ENGINE:=-fsanitize=fuzzer}"
: "${MAYHEM_JOBS:=$(nproc)}"
: "${COVERAGE_FLAGS=}"
export SANITIZER_FLAGS DEBUG_FLAGS CC CXX LIB_FUZZING_ENGINE MAYHEM_JOBS COVERAGE_FLAGS

cd "$SRC"

DEPS=/opt/toolchains/deps
CI=$(echo "$DEPS"/common_interfaces-*/src)
VO=$(echo "$DEPS"/vision_opencv-*/src)
OUT="$SRC/build/mayhem"
rm -rf "$OUT" && mkdir -p "$OUT"

INC=(-I"$SRC/mayhem/shim" -I"$CI/sensor_msgs/include" -I"$VO/image_geometry/include"
     -I"$SRC/depth_image_proc/include" -I"$SRC/image_proc/include" $(pkg-config --cflags opencv4))
OPENCV_LIBS=(-lopencv_calib3d -lopencv_imgproc -lopencv_core)
LIB_SRCS=("$SRC/image_proc/src/edge_aware.cpp" "$SRC/depth_image_proc/src/conversions.cpp"
          "$VO/image_geometry/src/pinhole_camera_model.cpp")

# 1) The fuzzed library code, instrumented (ASan+UBSan, SanCov, DWARF 3).
FUZZ_FLAGS=($SANITIZER_FLAGS $DEBUG_FLAGS -fsanitize=fuzzer-no-link -O1 -std=c++17)
pids=()
for src in "${LIB_SRCS[@]}"; do
  $CXX "${FUZZ_FLAGS[@]}" "${INC[@]}" -c "$src" -o "$OUT/$(basename "${src%.cpp}").o" &
  pids+=($!)
done
$CXX $SANITIZER_FLAGS $DEBUG_FLAGS -c "$SRC/mayhem/lsan_off.cc" -o "$OUT/lsan_off.o" &
pids+=($!)
$CC $SANITIZER_FLAGS $DEBUG_FLAGS -c "$STANDALONE_FUZZ_MAIN" -o "$OUT/standalone_main.o" &
pids+=($!)
for p in "${pids[@]}"; do wait "$p"; done
LIB_OBJS=("$OUT"/edge_aware.o "$OUT"/conversions.o "$OUT"/pinhole_camera_model.o "$OUT"/lsan_off.o)

# 2) Each harness: libFuzzer binary + standalone reproducer.
for h in fuzz_edge_aware_debayer fuzz_depth_to_cloud fuzz_depth_radial; do
  $CXX "${FUZZ_FLAGS[@]}" "${INC[@]}" $LIB_FUZZING_ENGINE "$SRC/mayhem/harnesses/$h.cc" \
    "${LIB_OBJS[@]}" "${OPENCV_LIBS[@]}" -o "$SRC/$h"
  $CXX "${FUZZ_FLAGS[@]}" "${INC[@]}" "$SRC/mayhem/harnesses/$h.cc" "$OUT/standalone_main.o" \
    "${LIB_OBJS[@]}" "${OPENCV_LIBS[@]}" -o "$SRC/$h-standalone"
done

# 3) The KAT oracle, with normal flags and no sanitizers.
$CXX -O2 -std=c++17 $COVERAGE_FLAGS "${INC[@]}" "$SRC/mayhem/tests/kat.cc" "${LIB_SRCS[@]}" \
  "${OPENCV_LIBS[@]}" -o "$OUT/kat"
