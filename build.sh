#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv main ../
cd ..
scp main root@192.168.145.100:~/
cp main ~/Desktop/SH
