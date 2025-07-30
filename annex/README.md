# Exploration of capabilities and applications of recent specialized hardware components in modern GPUs
## Dissertation project + annexes

This annex contains various resources related to the dissertation thesis, including complete test results, installation scripts, licenses of used libraries and other documentation.

The project itself represents modifications made to the Bullet3 physics engine to replace the GPGPU castRays function used by client code with one that uses hardware-accelerated ray tracing. It also includes benchmark examples and classes meant for testing the accuracy and performance of the method.

The results are described in the respective dissertation thesis.

Installing
----------

### Requirements:
- GPU with hardware-accelerated ray tracing
    - also required is support for ``position_fetch`` support which can be checked [here](https://vulkan.gpuinfo.org/listdevicescoverage.php?extension=VK_KHR_ray_tracing_position_fetch)
    - example GPUs are those in RTX 3000+ series
    - tested successfully on NVIDIA RTX 3070 Mobile, compatible but with bad results (undiagnosed) on NVIDIA RTX 4060 Mobile and AMD RX 7600 XT
- Windows OS
    - this was tested on Windows 10
    - this requirement could be eluded if ``build_visual_studio_vulkan.bat`` is replaced with an equivalent
    - this file is based on similar files found in bullet3 repo, which includes scripts for other platforms
- around 2.5 GB of free space
- internet connection for downloading bullet3 and nvpro_core GitHub repos
- git
    - for ``git clone``, ``git apply``
    - and for the Git Bash utility used to run the install scripts (``.sh``)
- Vulkan SDK >=1.4
    - this will regularly be looked up under ``C:/VulkanSDK/1.4*``
    - however it can be manually specified in ``build_visual_studio_vulkan.bat`` as the example suggests
- Visual Studio
    - the build scripts will generate VS solutions that are then used to compile the project
    - bullet3's solution comes as a VS2010 project in particular but can be retargeted later when opening it

### Steps:
1) Make sure the above requirements are respected
    - glslc can be terminal from terminal
    - git can be used from terminal
    - Vulkan can be found at any of the specified locations
2) Open git bash in the annex archive's root folder
3) Run ``install.sh``. This script will:
    - clone the nvpro_core repository
    - patch the build files with changes from ``patch`` folder meant to make it stand-alone and compatible
    - make a ``build`` folder and use cmake to generate a VS solution (``build/nvpro_core.sln``)
    - wait for the user to:
        - open the solution in VS
        - compile it with either Debug or Release configurations as will be used in bullet3 (=> ``build/Debug/nvpro_core_vk.lib`` or ``build/Release/nvpro_core_vk.lib``)
    - then instructs the user to follow the next step that is:
4) Run ``install2.sh``. This script will:
    - clone the bullet3 repository
    - apply patches for the build files **and the dissertation project's code that represents the original contribution**
    - use premake4 in ``build_visual_studio_vulkan.bat`` to generate VS solution in ``vs2010`` folder
    - instruct the user on how to compile and run the project
5) Open ``0_Bullet3Solution.sln``
6) Build either Debug or Release configurations based on which were built on step 3)
7) Run the application with the ``--enable_experimental_opencl`` command-line argument
8) Scroll down to OpenCL examples and run either ``Box-Box`` or ``Plane Convex``

### Troubleshooting:
 - ``.spv file for vulkan ray casting not found`` - make sure that ``compileShaders.bat`` was executed

Licenses
--------

Below are the licenses of the libraries directly used in this work, of which files were used and in some cases, as seen in the patch files, modified.

### Bullet

- All source code files are licensed under the permissive zlib license (http://opensource.org/licenses/Zlib) unless marked differently in a particular folder/file.

### nvpro_core:
- nvpro_core is licensed under the Apache License 2.0.

Credits
-------
Also applied in this work is gh#4626 in Bullet3 GitHub repository that fixes ExampleBrowser text rendering