#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_encoder ../
cd ..
rm -rf build
chmod +x example_encoder
mv example_encoder ~/Desktop/SH
