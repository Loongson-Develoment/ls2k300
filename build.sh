#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv main ../
cd ..
scp main root@192.168.178.200:~/
cp main ~/Desktop/SH
