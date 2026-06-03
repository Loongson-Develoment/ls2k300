#!/bin/bash
set -e

MODE="${1:-with_target}"

mkdir -p build
cd build

if [ "${MODE}" = "without_target" ]; then
    cmake -DRECORD_WITH_TARGET=OFF ..
    OUTPUT_NAME="example_video_recorder_without_target"
else
    cmake -DRECORD_WITH_TARGET=ON ..
    OUTPUT_NAME="example_video_recorder_with_target"
fi

make -j8
mv "${OUTPUT_NAME}" ../
cd ..
rm -rf build
chmod +x "${OUTPUT_NAME}"
mv "${OUTPUT_NAME}" ~/Desktop/SH
