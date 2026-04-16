#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_soft_spi ../
cd ..
rm -rf build
chmod +x example_soft_spi
mv example_soft_spi ~/Desktop/SH
