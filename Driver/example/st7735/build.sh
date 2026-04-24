#!/bin/bash
set -e

mkdir -p build
cd build
cmake ..
make -j8
mv example_st7735 ../
cd ..
rm -rf build
chmod +x example_st7735
mv example_st7735 ~/Desktop/SH
