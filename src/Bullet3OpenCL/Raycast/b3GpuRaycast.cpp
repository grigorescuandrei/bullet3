
#include "b3GpuRaycast.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3Collidable.h"
#include "Bullet3Collision/NarrowPhaseCollision/shared/b3RigidBodyData.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhaseInternalData.h"

#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3OpenCLArray.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3FillCL.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3RadixSort32CL.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuBroadphaseInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvh.h"

#include "Bullet3OpenCL/Raycast/kernels/rayCastKernels.h"

#define B3_RAYCAST_PATH "src/Bullet3OpenCL/Raycast/kernels/rayCastKernels.cl"

struct b3GpuRaycastInternalData
{
	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_q;
	cl_kernel m_raytraceKernel;
	cl_kernel m_raytracePairsKernel;
	cl_kernel m_findRayRigidPairIndexRanges;

	b3VulkanContext m_vkContext;
	VkPipeline m_pipeline;
	VkPipelineLayout m_pipelineLayout;
	VkDescriptorSetLayout m_descriptorSetLayout;
	VkShaderModule m_shader_module;

	b3GpuParallelLinearBvh* m_plbvh;
	b3RadixSort32CL* m_radixSorter;
	b3FillCL* m_fill;

	//1 element per ray
	b3OpenCLArray<b3RayInfo>* m_gpuRays;
	b3OpenCLArray<b3RayHit>* m_gpuHitResults;
	b3OpenCLArray<int>* m_firstRayRigidPairIndexPerRay;
	b3OpenCLArray<int>* m_numRayRigidPairsPerRay;

	//1 element per (ray index, rigid index) pair, where the ray intersects with the rigid's AABB
	b3OpenCLArray<int>* m_gpuNumRayRigidPairs;
	b3OpenCLArray<b3Int2>* m_gpuRayRigidPairs;  //x == ray index, y == rigid index

	int m_test;
};

b3GpuRaycast::b3GpuRaycast(cl_context ctx, cl_device_id device, cl_command_queue q, b3VulkanContext vkContext)
{
	m_data = new b3GpuRaycastInternalData;
	m_data->m_context = ctx;
	m_data->m_device = device;
	m_data->m_q = q;
	m_data->m_raytraceKernel = 0;
	m_data->m_raytracePairsKernel = 0;
	m_data->m_findRayRigidPairIndexRanges = 0;

	m_data->m_vkContext = vkContext;

	auto shader = nvh::loadFile("vk_raycast_comp.spv", true, {SHADERS_PATH}, true);
	VkShaderModuleCreateInfo shaderModuleCreateInfo = {
        VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,      // VkStructureType              sType;
        0,                                                // const void*                  pNext;
        0,                                                // VkShaderModuleCreateFlags    flags;
        shader.size(),                                    // size_t                       codeSize;
        reinterpret_cast<const uint32_t*>(shader.data())  // const uint32_t*              pCode;
    };

	if (vkCreateShaderModule(vkContext.m_device, &shaderModuleCreateInfo, 0, &(m_data->m_shader_module)) != VK_SUCCESS) {
		b3Error("failed to create raycast shader module!");
	}

    VkDescriptorSetLayoutBinding descriptorSetLayoutBindings[3] = {
        {
            0,												// uint32_t              binding;
            VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,  // VkDescriptorType      descriptorType;
            1,												// uint32_t              descriptorCount;
            VK_SHADER_STAGE_COMPUTE_BIT,					// VkShaderStageFlags    stageFlags;
            0												// const VkSampler*      pImmutableSamplers;
        },
		{
			1,
			VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			1,
			VK_SHADER_STAGE_COMPUTE_BIT,
			0
		},
		{
			2,
			VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			1,
			VK_SHADER_STAGE_COMPUTE_BIT,
			0
		}
    };

    VkDescriptorSetLayoutCreateInfo descriptorSetLayoutCreateInfo = {
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,  // VkStructureType                        sType;
        0,                                                    // const void*                            pNext;
        0,                                                    // VkDescriptorSetLayoutCreateFlags       flags;
        3,                                                    // uint32_t                               bindingCount;
        descriptorSetLayoutBindings                           // const VkDescriptorSetLayoutBinding*    pBindings;
    };

	if (vkCreateDescriptorSetLayout(vkContext.m_device, &descriptorSetLayoutCreateInfo, 0, &(m_data->m_descriptorSetLayout)) != VK_SUCCESS) {
		b3Error("failed to create raycast descriptor set layout");
	}

	// add pushConstantsRange here
    VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = {
        VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,  // VkStructureType                 sType;
        0,                                              // const void*                     pNext;
        0,                                              // VkPipelineLayoutCreateFlags     flags;
        1,                                              // uint32_t                        setLayoutCount;
        &(m_data->m_descriptorSetLayout),               // const VkDescriptorSetLayout*    pSetLayouts;
        0,                                              // uint32_t                        pushConstantRangeCount;
        0                                               // const VkPushConstantRange*      pPushConstantRanges;
    };

	if (vkCreatePipelineLayout(vkContext.m_device, &pipelineLayoutCreateInfo, 0, &(m_data->m_pipelineLayout)) != VK_SUCCESS) {
		b3Error("failed to create raycast pipeline layout");
	}

    VkComputePipelineCreateInfo computePipelineCreateInfo = {
        VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO, // VkStructureType                    sType;
        0,                                              // const void*                        pNext;
        0,                                              // VkPipelineCreateFlags              flags;
        {
            // VkPipelineShaderStageCreateInfo    stage;
            VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,  // VkStructureType                     sType;
            0,                                                    // const void*                         pNext;
            0,                                                    // VkPipelineShaderStageCreateFlags    flags;
            VK_SHADER_STAGE_COMPUTE_BIT,                          // VkShaderStageFlagBits               stage;
            m_data->m_shader_module,                              // VkShaderModule                      module;
            "main",                                               // const char*                         pName;
            0                                                     // const VkSpecializationInfo*         pSpecializationInfo;
        },
        m_data->m_pipelineLayout,                       // VkPipelineLayout                   layout;
        0,                                              // VkPipeline                         basePipelineHandle;
        0                                               // int32_t                            basePipelineIndex;
    };

    if (vkCreateComputePipelines(vkContext.m_device, 0, 1, &computePipelineCreateInfo, 0, &(m_data->m_pipeline)) != VK_SUCCESS) {
		b3Error("failed to create raycast pipeline");
	}

	m_data->m_plbvh = new b3GpuParallelLinearBvh(ctx, device, q);
	m_data->m_radixSorter = new b3RadixSort32CL(ctx, device, q);
	m_data->m_fill = new b3FillCL(ctx, device, q);

	m_data->m_gpuRays = new b3OpenCLArray<b3RayInfo>(ctx, q);
	m_data->m_gpuHitResults = new b3OpenCLArray<b3RayHit>(ctx, q);
	m_data->m_firstRayRigidPairIndexPerRay = new b3OpenCLArray<int>(ctx, q);
	m_data->m_numRayRigidPairsPerRay = new b3OpenCLArray<int>(ctx, q);
	m_data->m_gpuNumRayRigidPairs = new b3OpenCLArray<int>(ctx, q);
	m_data->m_gpuRayRigidPairs = new b3OpenCLArray<b3Int2>(ctx, q);

	{
		cl_int errNum = 0;
		cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context, m_data->m_device, rayCastKernelCL, &errNum, "", B3_RAYCAST_PATH);
		b3Assert(errNum == CL_SUCCESS);
		m_data->m_raytraceKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device, rayCastKernelCL, "rayCastKernel", &errNum, prog);
		b3Assert(errNum == CL_SUCCESS);
		m_data->m_raytracePairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device, rayCastKernelCL, "rayCastPairsKernel", &errNum, prog);
		b3Assert(errNum == CL_SUCCESS);
		m_data->m_findRayRigidPairIndexRanges = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device, rayCastKernelCL, "findRayRigidPairIndexRanges", &errNum, prog);
		b3Assert(errNum == CL_SUCCESS);
		clReleaseProgram(prog);
	}
}

b3GpuRaycast::~b3GpuRaycast()
{
	clReleaseKernel(m_data->m_raytraceKernel);
	clReleaseKernel(m_data->m_raytracePairsKernel);
	clReleaseKernel(m_data->m_findRayRigidPairIndexRanges);

	// TODO: clear raycast compute pipeline
    vkDestroyPipeline(m_data->m_vkContext.m_device, m_data->m_pipeline, NULL);
    vkDestroyPipelineLayout(m_data->m_vkContext.m_device, m_data->m_pipelineLayout, NULL);
    vkDestroyDescriptorSetLayout(m_data->m_vkContext.m_device, m_data->m_descriptorSetLayout, NULL);
    vkDestroyShaderModule(m_data->m_vkContext.m_device, m_data->m_shader_module, NULL);

	delete m_data->m_plbvh;
	delete m_data->m_radixSorter;
	delete m_data->m_fill;

	delete m_data->m_gpuRays;
	delete m_data->m_gpuHitResults;
	delete m_data->m_firstRayRigidPairIndexPerRay;
	delete m_data->m_numRayRigidPairsPerRay;
	delete m_data->m_gpuNumRayRigidPairs;
	delete m_data->m_gpuRayRigidPairs;

	delete m_data;
}

bool sphere_intersect(const b3Vector3& spherePos, b3Scalar radius, const b3Vector3& rayFrom, const b3Vector3& rayTo, float& hitFraction)
{
	b3Vector3 rs = rayFrom - spherePos;
	b3Vector3 rayDir = rayTo - rayFrom;

	float A = b3Dot(rayDir, rayDir);
	float B = b3Dot(rs, rayDir);
	float C = b3Dot(rs, rs) - (radius * radius);

	float D = B * B - A * C;

	if (D > 0.0)
	{
		float t = (-B - sqrt(D)) / A;

		if ((t >= 0.0f) && (t < hitFraction))
		{
			hitFraction = t;
			return true;
		}
	}
	return false;
}

bool rayConvex(const b3Vector3& rayFromLocal, const b3Vector3& rayToLocal, const b3ConvexPolyhedronData& poly,
			   const b3AlignedObjectArray<b3GpuFace>& faces, float& hitFraction, b3Vector3& hitNormal)
{
	float exitFraction = hitFraction;
	float enterFraction = -0.1f;
	b3Vector3 curHitNormal = b3MakeVector3(0, 0, 0);
	for (int i = 0; i < poly.m_numFaces; i++)
	{
		const b3GpuFace& face = faces[poly.m_faceOffset + i];
		float fromPlaneDist = b3Dot(rayFromLocal, face.m_plane) + face.m_plane.w;
		float toPlaneDist = b3Dot(rayToLocal, face.m_plane) + face.m_plane.w;
		if (fromPlaneDist < 0.f)
		{
			if (toPlaneDist >= 0.f)
			{
				float fraction = fromPlaneDist / (fromPlaneDist - toPlaneDist);
				if (exitFraction > fraction)
				{
					exitFraction = fraction;
				}
			}
		}
		else
		{
			if (toPlaneDist < 0.f)
			{
				float fraction = fromPlaneDist / (fromPlaneDist - toPlaneDist);
				if (enterFraction <= fraction)
				{
					enterFraction = fraction;
					curHitNormal = face.m_plane;
					curHitNormal.w = 0.f;
				}
			}
			else
			{
				return false;
			}
		}
		if (exitFraction <= enterFraction)
			return false;
	}

	if (enterFraction < 0.f)
		return false;

	hitFraction = enterFraction;
	hitNormal = curHitNormal;
	return true;
}

void b3GpuRaycast::castRaysHost(const b3AlignedObjectArray<b3RayInfo>& rays, b3AlignedObjectArray<b3RayHit>& hitResults,
								int numBodies, const struct b3RigidBodyData* bodies, int numCollidables, const struct b3Collidable* collidables, const struct b3GpuNarrowPhaseInternalData* narrowphaseData)
{
	//	return castRays(rays,hitResults,numBodies,bodies,numCollidables,collidables);

	B3_PROFILE("castRaysHost");
	for (int r = 0; r < rays.size(); r++)
	{
		b3Vector3 rayFrom = rays[r].m_from;
		b3Vector3 rayTo = rays[r].m_to;
		float hitFraction = hitResults[r].m_hitFraction;

		int hitBodyIndex = -1;
		b3Vector3 hitNormal;

		for (int b = 0; b < numBodies; b++)
		{
			const b3Vector3& pos = bodies[b].m_pos;
			//const b3Quaternion& orn = bodies[b].m_quat;

			switch (collidables[bodies[b].m_collidableIdx].m_shapeType)
			{
				case SHAPE_SPHERE:
				{
					b3Scalar radius = collidables[bodies[b].m_collidableIdx].m_radius;
					if (sphere_intersect(pos, radius, rayFrom, rayTo, hitFraction))
					{
						hitBodyIndex = b;
						b3Vector3 hitPoint;
						hitPoint.setInterpolate3(rays[r].m_from, rays[r].m_to, hitFraction);
						hitNormal = (hitPoint - bodies[b].m_pos).normalize();
					}
				}
				case SHAPE_CONVEX_HULL:
				{
					b3Transform convexWorldTransform;
					convexWorldTransform.setIdentity();
					convexWorldTransform.setOrigin(bodies[b].m_pos);
					convexWorldTransform.setRotation(bodies[b].m_quat);
					b3Transform convexWorld2Local = convexWorldTransform.inverse();

					b3Vector3 rayFromLocal = convexWorld2Local(rayFrom);
					b3Vector3 rayToLocal = convexWorld2Local(rayTo);

					int shapeIndex = collidables[bodies[b].m_collidableIdx].m_shapeIndex;
					const b3ConvexPolyhedronData& poly = narrowphaseData->m_convexPolyhedra[shapeIndex];
					if (rayConvex(rayFromLocal, rayToLocal, poly, narrowphaseData->m_convexFaces, hitFraction, hitNormal))
					{
						hitBodyIndex = b;
					}

					break;
				}
				default:
				{
					static bool once = true;
					if (once)
					{
						once = false;
						b3Warning("Raytest: unsupported shape type\n");
					}
				}
			}
		}
		if (hitBodyIndex >= 0)
		{
			hitResults[r].m_hitFraction = hitFraction;
			hitResults[r].m_hitPoint.setInterpolate3(rays[r].m_from, rays[r].m_to, hitFraction);
			hitResults[r].m_hitNormal = hitNormal;
			hitResults[r].m_hitBody = hitBodyIndex;
		}
	}
}

void b3GpuRaycast::castRaysVk(const b3AlignedObjectArray<b3RayInfo>& rays, b3AlignedObjectArray<b3RayHit>& hitResults,
	int numBodies, const struct b3RigidBodyData* bodies, int numCollidables, const struct b3Collidable* collidables,
	const struct b3GpuNarrowPhaseInternalData* narrowphaseData, class b3GpuBroadphaseInterface* broadphase) {

	int nbRays = rays.size();
	nvvk::ResourceAllocatorDma& m_alloc = *(m_data->m_vkContext.m_pAlloc);

	VkDescriptorPoolSize descriptorPoolSize[2] = {
		{
			VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,  // VkDescriptorType    type;
			1												// uint32_t            descriptorCount;
		},
		{
			VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			2
		}
    };

    VkDescriptorPoolCreateInfo descriptorPoolCreateInfo = {
        VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,  // VkStructureType                sType;
        0,                                              // const void*                    pNext;
        0,                                              // VkDescriptorPoolCreateFlags    flags;
        1,                                              // uint32_t                       maxSets;
        2,                                              // uint32_t                       poolSizeCount;
        descriptorPoolSize                              // const VkDescriptorPoolSize*    pPoolSizes;
    };

    VkDescriptorPool descriptorPool;
	if (vkCreateDescriptorPool(m_data->m_vkContext.m_device, &descriptorPoolCreateInfo, 0, &descriptorPool) != VK_SUCCESS) {
		b3Error("failed to create descriptor pool");
	}

    VkDescriptorSetAllocateInfo descriptorSetAllocateInfo = {
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO, // VkStructureType                 sType;
        0,                                              // const void*                     pNext;
        descriptorPool,                                 // VkDescriptorPool                descriptorPool;
        1,                                              // uint32_t                        descriptorSetCount;
        &(m_data->m_descriptorSetLayout)                // const VkDescriptorSetLayout*    pSetLayouts;
    };

    VkDescriptorSet descriptorSet;
    if (vkAllocateDescriptorSets(m_data->m_vkContext.m_device, &descriptorSetAllocateInfo, &descriptorSet) != VK_SUCCESS) {
		b3Error("failed to create descriptor set");
	}

	// begin invocation of raycast compute shader
	nvvk::CommandPool cmdBufGet(m_data->m_vkContext.m_device, m_data->m_vkContext.m_queueIndex);
	VkCommandBuffer cmdBuf = cmdBufGet.createCommandBuffer();

	auto tlas = m_data->m_vkContext.m_pRtBuilder->getAccelerationStructure();
	VkWriteDescriptorSetAccelerationStructureKHR descASInfo{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR};
	descASInfo.accelerationStructureCount = 1;
	descASInfo.pAccelerationStructures    = &tlas;

	auto flag = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
	std::vector<b3RayInfo> raysVector(nbRays);
	std::vector<b3RayHit> hitsVector(nbRays);
	int i;
	for (i = 0; i < nbRays; ++i) {
		raysVector.push_back(rays.at(i));
		hitsVector.push_back(b3RayHit());
	}

	nvvk::Buffer rayInfoBuffer = m_alloc.createBuffer(cmdBuf, raysVector, flag);
	nvvk::Buffer rayHitBuffer = m_alloc.createBuffer(cmdBuf, hitsVector, flag);

    VkDescriptorBufferInfo in_descriptorBufferInfo = {
        rayInfoBuffer.buffer,
        0,            // VkDeviceSize    offset;
        VK_WHOLE_SIZE // VkDeviceSize    range;
    };

    VkDescriptorBufferInfo out_descriptorBufferInfo = {
        rayHitBuffer.buffer,
        0,             // VkDeviceSize    offset;
        VK_WHOLE_SIZE  // VkDeviceSize    range;
    };

    VkWriteDescriptorSet writeDescriptorSet[3] = {
        {
            VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,			// VkStructureType                  sType;
            &descASInfo,									// const void*                      pNext;
            descriptorSet,									// VkDescriptorSet                  dstSet;
            0,												// uint32_t                         dstBinding;
            0,												// uint32_t                         dstArrayElement;
            1,												// uint32_t                         descriptorCount;
            VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,	// VkDescriptorType                 descriptorType;
            0,												// const VkDescriptorImageInfo*     pImageInfo;
            0,												// const VkDescriptorBufferInfo*    pBufferInfo;
            0												// const VkBufferView*              pTexelBufferView;
        },
        {
            VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, // VkStructureType                  sType;
            0,                                      // const void*                      pNext;
            descriptorSet,                          // VkDescriptorSet                  dstSet;
            1,                                      // uint32_t                         dstBinding;
            0,                                      // uint32_t                         dstArrayElement;
            1,                                      // uint32_t                         descriptorCount;
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,      // VkDescriptorType                 descriptorType;
            0,                                      // const VkDescriptorImageInfo*     pImageInfo;
            &in_descriptorBufferInfo,               // const VkDescriptorBufferInfo*    pBufferInfo;
            0                                       // const VkBufferView*              pTexelBufferView;
        },
        {
            VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, // VkStructureType                  sType;
            0,                                      // const void*                      pNext;
            descriptorSet,                          // VkDescriptorSet                  dstSet;
            2,                                      // uint32_t                         dstBinding;
            0,                                      // uint32_t                         dstArrayElement;
            1,                                      // uint32_t                         descriptorCount;
            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,      // VkDescriptorType                 descriptorType;
            0,                                      // const VkDescriptorImageInfo*     pImageInfo;
            &out_descriptorBufferInfo,              // const VkDescriptorBufferInfo*    pBufferInfo;
            0                                       // const VkBufferView*              pTexelBufferView;
        }
    };

    vkUpdateDescriptorSets(m_data->m_vkContext.m_device, 3, writeDescriptorSet, 0, 0);

	vkCmdBindPipeline(cmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE, m_data->m_pipeline);

	vkCmdBindDescriptorSets(cmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE,
		m_data->m_pipelineLayout, 0, 1, &descriptorSet, 0, 0);

	vkCmdDispatch(cmdBuf, nbRays, 1, 1);

	cmdBufGet.submitAndWait(cmdBuf);
	m_alloc.finalizeAndReleaseStaging();

	m_alloc.destroy(rayInfoBuffer);
	m_alloc.destroy(rayHitBuffer);
	vkDestroyDescriptorPool(m_data->m_vkContext.m_device, descriptorPool, NULL);
}

///todo: add some acceleration structure (AABBs, tree etc)
void b3GpuRaycast::castRays(const b3AlignedObjectArray<b3RayInfo>& rays, b3AlignedObjectArray<b3RayHit>& hitResults,
							int numBodies, const struct b3RigidBodyData* bodies, int numCollidables, const struct b3Collidable* collidables,
							const struct b3GpuNarrowPhaseInternalData* narrowphaseData, class b3GpuBroadphaseInterface* broadphase)
{
	//castRaysHost(rays,hitResults,numBodies,bodies,numCollidables,collidables,narrowphaseData);

	B3_PROFILE("castRaysGPU");

	{
		B3_PROFILE("raycast copyFromHost");
		m_data->m_gpuRays->copyFromHost(rays);
		m_data->m_gpuHitResults->copyFromHost(hitResults);
	}

	int numRays = hitResults.size();
	{
		m_data->m_firstRayRigidPairIndexPerRay->resize(numRays);
		m_data->m_numRayRigidPairsPerRay->resize(numRays);

		m_data->m_gpuNumRayRigidPairs->resize(1);
		m_data->m_gpuRayRigidPairs->resize(numRays * 16);
	}

	//run kernel
	const bool USE_BRUTE_FORCE_RAYCAST = false;
	if (USE_BRUTE_FORCE_RAYCAST)
	{
		B3_PROFILE("raycast launch1D");

		b3LauncherCL launcher(m_data->m_q, m_data->m_raytraceKernel, "m_raytraceKernel");
		int numRays = rays.size();
		launcher.setConst(numRays);

		launcher.setBuffer(m_data->m_gpuRays->getBufferCL());
		launcher.setBuffer(m_data->m_gpuHitResults->getBufferCL());

		launcher.setConst(numBodies);
		launcher.setBuffer(narrowphaseData->m_bodyBufferGPU->getBufferCL());
		launcher.setBuffer(narrowphaseData->m_collidablesGPU->getBufferCL());
		launcher.setBuffer(narrowphaseData->m_convexFacesGPU->getBufferCL());
		launcher.setBuffer(narrowphaseData->m_convexPolyhedraGPU->getBufferCL());

		launcher.launch1D(numRays);
		clFinish(m_data->m_q);
	}
	else
	{
		m_data->m_plbvh->build(broadphase->getAllAabbsGPU(), broadphase->getSmallAabbIndicesGPU(), broadphase->getLargeAabbIndicesGPU());

		m_data->m_plbvh->testRaysAgainstBvhAabbs(*m_data->m_gpuRays, *m_data->m_gpuNumRayRigidPairs, *m_data->m_gpuRayRigidPairs);

		int numRayRigidPairs = -1;
		m_data->m_gpuNumRayRigidPairs->copyToHostPointer(&numRayRigidPairs, 1);
		if (numRayRigidPairs > m_data->m_gpuRayRigidPairs->size())
		{
			numRayRigidPairs = m_data->m_gpuRayRigidPairs->size();
			m_data->m_gpuNumRayRigidPairs->copyFromHostPointer(&numRayRigidPairs, 1);
		}

		m_data->m_gpuRayRigidPairs->resize(numRayRigidPairs);  //Radix sort needs b3OpenCLArray::size() to be correct

		//Sort ray-rigid pairs by ray index
		{
			B3_PROFILE("sort ray-rigid pairs");
			m_data->m_radixSorter->execute(*reinterpret_cast<b3OpenCLArray<b3SortData>*>(m_data->m_gpuRayRigidPairs));
		}

		//detect start,count of each ray pair
		{
			B3_PROFILE("detect ray-rigid pair index ranges");

			{
				B3_PROFILE("reset ray-rigid pair index ranges");

				m_data->m_fill->execute(*m_data->m_firstRayRigidPairIndexPerRay, numRayRigidPairs, numRays);  //atomic_min used to find first index
				m_data->m_fill->execute(*m_data->m_numRayRigidPairsPerRay, 0, numRays);
				clFinish(m_data->m_q);
			}

			b3BufferInfoCL bufferInfo[] =
				{
					b3BufferInfoCL(m_data->m_gpuRayRigidPairs->getBufferCL()),

					b3BufferInfoCL(m_data->m_firstRayRigidPairIndexPerRay->getBufferCL()),
					b3BufferInfoCL(m_data->m_numRayRigidPairsPerRay->getBufferCL())};

			b3LauncherCL launcher(m_data->m_q, m_data->m_findRayRigidPairIndexRanges, "m_findRayRigidPairIndexRanges");
			launcher.setBuffers(bufferInfo, sizeof(bufferInfo) / sizeof(b3BufferInfoCL));
			launcher.setConst(numRayRigidPairs);

			launcher.launch1D(numRayRigidPairs);
			clFinish(m_data->m_q);
		}

		{
			B3_PROFILE("ray-rigid intersection");

			b3BufferInfoCL bufferInfo[] =
				{
					b3BufferInfoCL(m_data->m_gpuRays->getBufferCL()),
					b3BufferInfoCL(m_data->m_gpuHitResults->getBufferCL()),
					b3BufferInfoCL(m_data->m_firstRayRigidPairIndexPerRay->getBufferCL()),
					b3BufferInfoCL(m_data->m_numRayRigidPairsPerRay->getBufferCL()),

					b3BufferInfoCL(narrowphaseData->m_bodyBufferGPU->getBufferCL()),
					b3BufferInfoCL(narrowphaseData->m_collidablesGPU->getBufferCL()),
					b3BufferInfoCL(narrowphaseData->m_convexFacesGPU->getBufferCL()),
					b3BufferInfoCL(narrowphaseData->m_convexPolyhedraGPU->getBufferCL()),

					b3BufferInfoCL(m_data->m_gpuRayRigidPairs->getBufferCL())};

			b3LauncherCL launcher(m_data->m_q, m_data->m_raytracePairsKernel, "m_raytracePairsKernel");
			launcher.setBuffers(bufferInfo, sizeof(bufferInfo) / sizeof(b3BufferInfoCL));
			launcher.setConst(numRays);

			launcher.launch1D(numRays);
			clFinish(m_data->m_q);
		}
	}

	//copy results
	{
		B3_PROFILE("raycast copyToHost");
		m_data->m_gpuHitResults->copyToHost(hitResults);
	}
}