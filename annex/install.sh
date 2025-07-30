#!/bin/bash

echo "Downloading nvpro_core"
git clone https://github.com/nvpro-samples/nvpro_core.git
#cp -r nvpro_corecopy nvpro_core

echo "Patching nvpro_core for standalone use"
cp -f patch/CMakeLists.txt nvpro_core/CMakeLists.txt
cp -f patch/NvFoundation.h nvpro_core/nvp/NvFoundation.h
cd nvpro_core
mkdir build
cd build
cmake ..
cmake ..

echo "Please open mvpro_core.sln from nvpro_core/build in Visual Studio"
echo "Build the project in both debug/release configs (as needed)"
echo "By the end of this you should have been able to compile nvpro_core into an nvpro_core_vk.lib file"
echo "Then proceed with install2.sh"
read -n1 -r -p "Press any key to continue..."
echo ""