#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_canfd ../
cd ..
rm -rf build
chmod +x example_canfd
mv example_canfd ~/Desktop/SH
