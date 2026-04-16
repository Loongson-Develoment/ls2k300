#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_i2c ../
cd ..
rm -rf build
chmod +x example_i2c
mv example_i2c ~/Desktop/SH
