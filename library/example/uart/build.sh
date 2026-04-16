#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_uart ../
cd ..
rm -rf build
chmod +x example_uart
mv example_uart ~/Desktop/SH
