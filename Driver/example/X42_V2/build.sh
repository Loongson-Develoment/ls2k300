#!/bin/bash
set -e

mkdir -p build
cd build
cmake ..
make -j8
mv example_X42_V2 ../
cd ..
rm -rf build
chmod +x example_X42_V2
mv example_X42_V2 ~/Desktop/SH
