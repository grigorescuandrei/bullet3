IF NOT EXIST bin mkdir bin
IF NOT EXIST bin\openvr_api.dll  copy examples\ThirdPartyLibs\openvr\bin\win32\openvr_api.dll bin
IF NOT EXIST bin\openvr64pi.dll  copy examples\ThirdPartyLibs\openvr\bin\win64\openvr_api.dll bin\openvr64pi.dll

rem copied after build_visual_studio_vr_pybullet_double.bat
rem check for consistency with that if this fails

rem find a python version (hopefully just 1) and use this
dir c:\python* /b /ad > tmp1234.txt

set /p myvar1= < tmp1234.txt
set myvar=c:/%myvar1%
del tmp1234.txt

rem find a vulkan 1.4 version (hopefully just 1) and use this
dir c:\VulkanSDK\1.4* /b /ad > tmp1234.txt

set /p myvar2a= < tmp1234.txt
set myvar2=C:/VulkanSDK/%myvar2a%
del tmp1234.txt

rem you can also override and hardcode the Python path like this (just remove the # hashmark in next line)
rem SET myvar=c:\python-3.5.2

cd build3


premake4 --nvpro-includes --dynamic-runtime --double --enable_stable_pd --enable_multithreading --midi --enable_static_vr_plugin --enable_openvr --enable_pybullet --vulkan_include_dir="%myvar2%/Include" --vulkan_lib_dir="%myvar2%/Lib" --python_include_dir="%myvar%/include" --python_lib_dir="%myvar%/libs"   --targetdir="../bin" vs2010 

rem premake4  --double   --enable_multithreading --midi --enable_static_vr_plugin --enable_openvr --enable_pybullet --python_include_dir="%myvar%/include" --python_lib_dir="%myvar%/libs"   --targetdir="../binserver" vs2010 
rem premake4  --double --enable_grpc --enable_multithreading --midi --enable_static_vr_plugin --enable_openvr --enable_pybullet --python_include_dir="%myvar%/include" --python_lib_dir="%myvar%/libs"   --targetdir="../binserver" vs2010 
rem premake4  --serial --audio --double --midi --enable_openvr --enable_pybullet --python_include_dir="%myvar%/include" --python_lib_dir="%myvar%/libs"   --targetdir="../bin" vs2010 

start vs2010

