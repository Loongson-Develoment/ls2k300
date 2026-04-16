#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_adc ../
cd ..
rm -rf build
chmod +x example_adc
mv example_adc ~/Desktop/SH
