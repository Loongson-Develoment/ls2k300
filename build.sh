#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv main1 main2 ../
cd ..
scp main1 root@192.168.214.101:~/
scp main2 root@192.168.214.100:~/

cp main1 main2 ~/Desktop/SH
