#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv example_gtim_pwm ../
cd ..
rm -rf build
chmod +x example_gtim_pwm
mv example_gtim_pwm ~/Desktop/SH
