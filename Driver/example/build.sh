#!/bin/bash
set -e

cmake -S . -B build
cmake --build build -j8

cp -f build/st7735/example_st7735 ./example_st7735
chmod +x example_st7735
