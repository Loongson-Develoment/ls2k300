#!/bin/bash
mkdir -p build
cd build
cmake ..
make -j8
mv main1 main2 main2_limit_inject_test ../
cd ..
scp main1 root@192.168.63.100:~/
scp main2 root@192.168.63.101:~/
scp main2_limit_inject_test root@192.168.63.101:~/

cp main1 main2 main2_limit_inject_test ~/Desktop/SH
