#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_timer ../
cd ..
rm -rf build
chmod +x example_timer
mv example_timer ~/Desktop/SH
