#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv test_motor ../
cd ..
scp test_motor root@192.168.214.100:~/
cp test_motor ~/Desktop/SH
