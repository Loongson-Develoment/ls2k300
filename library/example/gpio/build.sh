#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_gpio ../
cd ..
rm -rf build
chmod +x example_gpio
mv example_gpio ~/Desktop/SH
