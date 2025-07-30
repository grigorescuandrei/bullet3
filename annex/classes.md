# Classes
This file contains the added or modified classes and documents what functionalities were added and how they are used

## Class hierarchy

``b3VulkanUtils.h`` - added
 - represents the Vulkan equivalent of a GPU API context, like the OpenCL one Bullet3 already has
 - also includes other libraries such as glm and nvvk tat are used in the rest of the Vulkan implementation
 - includes:
    - shaderPaths - a vector of strings with relative paths where the program looks for Vulkan compute shaders/kernels
    - ``VkInstance``, ``VkDevice``, ``VkPhysicalDevice``, ``queueIndex`` (async compute as elaborated later), ``VkQueue`` and ``VkCommandPool`` - used by Vulkan
    - ``nvvk::ResourceAllocatorDma*`` and ``nvvk::RaytracingBuilderKHR`` - used by nvvk, with role in maintaining the acceleration structures, both top and bottom level
    - other validation layer/debug functions

``CommonOpenCLBase.h`` - modified
 - represents an interface for OpenCL accelerated Bullet3 demos
 - modified to also initialize the Vulkan context (``initVulkan()``) previously described alongside the OpenCL context
    - performs the checks for the required Vulkan extensions (such as ray tracing ones) against each of the available GPUs
    - picks a dedicated async compute queue for operations
    - performs setup of other related components
 - also clears used resources in ``exitVulkan()``

 ``GpuRigidBodyDemo.cpp`` - modified
  - represents the base implementation for OpenCL accelerated demos and inherits above interface
  - creates, among others, ``b3GpuNarrowPhase`` and ``b3GpuRigidBodyPipeline`` which uses it
  - now passes the new Vulkan context to other involved components of the application
  - makes the calls for creating and updating the acceleration structures, as well as for destroying Vulkan resources calling methods in ``b3GpuNarrowPhase``
  - represents the client code for ``castRaysVk`` defined in ``b3GpuRigidBodyPipeline`` and implements the rigid body drag-and-drop functionality, by emitting one ray and expecting one result

``b3GpuConvexScene`` - modified
 - represents the demos, inherits from above class, with ``b3GpuPlaneConvexScene`` representing the a class for used demos
 - the mentioned class creates a grid of objects on scene setup, that then fall onto a plane (a large cube)
 - the cubes and tetrahedra used are registered once, and reused, as convex shapes in ``b3GpuNarrowPhase``
 - also houses the implementation for ``b3RaycastBar2`` elaborated on separately, but used in benchmark and evaluations

``b3GpuNarrowPhase`` - modified
 - handles the convex (and concave) shapes added to the simulations by storing their vertex and index data in ``b3ConvexUtility` objects (temporarily, modified to be for the duration of its lifetime)
 - handles local AABBs and geometry data in a global "triangle soup"
 - modified to create BLAS from individual geometry data (``objectToVkGeometry()`` which allocates GPU buffers for each shape, then builds ``nnvk::RaytracingBuilder::BlasInput`` instances) for each shape added and to build them in batch (``createBottomLevelAS()`` which uses nvvk)
 - also creates and maintains the TLAS with its own methods (``createTopLevelAS()`` and ``updateTopLevelAS()`` which use nvvk) using up-to-date transform data (for which it has to transfer all rigid body instances, which hold it, from GPU to CPU - not ideal)
 - on use by client code (``b3GpuConvexScene``), hands these informations to ``b3GpuRaycast`` to be handled by ``castRaysVk``

``b3GpuRaycast`` - modified
 - houses the previous ``castRays`` method which uses OpenCL
 - builds an parallel linearized BVH as acceleration structure for this ray casting
 - modified to add a ``castRaysVk`` method which uses the narrow phase data including the acceleration structures supplied by ``b3GpuNarrowPhase``
 - uses Vulkan's compute pipeline to launch ``vk_raycast_comp.spv`` kernels which perform the actual casting
 - on return, completes the object-space hit position with narrow phase information to obtain full world-space hit positions (should be done in kernel if it had access to transform data)

``vk_raycast.comp`` - added
 - represents a compute shader, or a kernel, written in GLSL
 - for each ray defined in input, performs a RT-accelerated ray query and returns the outputs
 - pushes the ray query as far as it can until first front face met (for parity)
 - uses ``position_fetch`` extension to get the vertex coordinates, which, along already available barycentric coordinates and instance info from ray queries, can be used to calculate an object-space hit position for first hit of each ray

Separate modifications:
 - gh#4626 from bullet3 GitHub pull requests was implemented to fix text missing first part - affected file is GLInstancingRenderer.cpp
 - further examples were added to verify the function of the proposed method:
    - Plane Convex - similar to Box-Box, representing a grid of tetrahedra shapes instead
    - Tetra Breakable - example not previously exposed that instances individual tetrahedra in a mesh constituting for example a bunny - each tetrahedron is its own shape
- to Plane Convex and Box-Box examples, ``btRaycastBar2`` was adapted from benchmark examples into a ``b3RaycastBar2``, with new functions
    - drawing was moved to ``physicsDebugDraw`` to remain visible outside of ``stepSimulation`` (such as when paused by pressing I)
    - ``compare`` function added to compare accuracy between GPGPU ``castRays`` and RT accelerated ``castRaysVk``, with functionality to draw differences
- ``compileShaders.bat`` - added - for compiling the Vulkan shaders into a ``.spv`` file
- ``OpenGLExampleBrowser.cpp`` modified to draw physics debug on top of other renders