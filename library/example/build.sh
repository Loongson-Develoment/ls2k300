#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv ls2k0300_examples ../
