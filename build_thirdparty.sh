#!/bin/bash

echo ""
echo "Building Sophus lib!"
echo ""

cd thirdparty/Sophus
mkdir build
mkdir install
cd build 
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install/
make && make install
cd ../../../