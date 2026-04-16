#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_signal ../
cd ..
rm -rf build
chmod +x example_signal
mv example_signal ~/Desktop/SH
