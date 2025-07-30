#!/bin/bash

echo "Downloading Bullet3"
git clone https://github.com/bulletphysics/bullet3.git
#cp -r bullet3copy bullet3

echo "Patching Bullet"
cp patch/changes.patch bullet3/changes.patch
cd bullet3
git apply changes.patch
echo "Removing patch file"
rm changes.patch
echo "Compiling Vulkan (SPIR-V) shaders"
./compileShaders.bat

echo "Applying updated build files"
cd ..
cp patch/build_visual_studio_vulkan.bat bullet3/build_visual_studio_vulkan.bat
cp -f patch/premake4.lua bullet3/build3/premake4.lua
cp -f patch/premake4_examplebrowser.lua bullet3/examples/ExampleBrowser/premake4.lua
cp -f patch/premake4_opencl.lua bullet3/src/Bullet3OpenCL/premake4.lua

echo "Building Visual Studio solution"
cd bullet3
./build_visual_studio_vulkan.bat
echo "If no errors occured and vs2010 appeared in bullet3/build3 folder, follow these steps:"
echo "1) Open 0_BulletSolution.sln with your version of Visual Studio"
echo "2) When prompted, update the solution to the newer version of VS"
echo "3) Build in desired configuration"
echo "4) Run the program with --enable_experimental_opencl command-line argument to see the OpenCL examples category in the bottom of the example browser"
echo "5) Run either Box-Box or Convex-Plane examples"
read -n1 -r -p "Press any key to continue..."
echo ""