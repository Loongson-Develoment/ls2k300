#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_spi ../
cd ..
rm -rf build
chmod +x example_spi
mv example_spi ~/Desktop/SH
