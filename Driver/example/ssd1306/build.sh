#!/bin/bash
set -e

mkdir -p build
cd build
cmake ..
make -j8
mv example_EMM_V5 ../
cd ..
rm -rf build
chmod +x example_EMM_V5
mv example_EMM_V5 ~/Desktop/SH
