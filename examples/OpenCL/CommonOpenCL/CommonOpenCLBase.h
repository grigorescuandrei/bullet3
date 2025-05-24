#ifndef COMMON_MULTI_BODY_SETUP_H
#define COMMON_MULTI_BODY_SETUP_H

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonCameraInterface.h"

#include "GpuDemoInternalData.h"
#include "Bullet3Common/b3Scalar.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "Bullet3OpenCL/Initialize/b3VulkanUtils.h"

// # Vulkan
#include "vulkan/vulkan_core.h"
#include "nvpsystem.hpp"
#include <nvvk/context_vk.hpp>
// TODO: maybe add this
//#include "nvvk/commands_vk.hpp"
#include <nvvk/raytraceKHR_vk.hpp>

struct CommonOpenCLBase : public CommonExampleInterface
{
	struct GUIHelperInterface* m_guiHelper;
	struct GpuDemoInternalData* m_clData;

	b3VulkanContext m_vkContext;
	nvvk::ResourceAllocatorDma m_alloc; // TODO: may need to replace this without DMA
	nvvk::RaytracingBuilderKHR m_rtBuilder;

	CommonOpenCLBase(GUIHelperInterface* helper)
		: m_guiHelper(helper),
		  m_clData(0)
	{
		m_clData = new GpuDemoInternalData();
	}

	virtual ~CommonOpenCLBase()
	{
		delete m_clData;
		m_clData = 0;
	}

	virtual void stepSimulation(float deltaTime)
	{
	}

	virtual void initCL(int preferredDeviceIndex, int preferredPlatformIndex)
	{
		//	void* glCtx=0;
		//	void* glDC = 0;

		int ciErrNum = 0;

		cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
		//if (gAllowCpuOpenCL)
		//	deviceType = CL_DEVICE_TYPE_ALL;

		//	if (useInterop)
		//	{
		//		m_data->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
		//	} else
		{
			m_clData->m_clContext = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0, 0, preferredDeviceIndex, preferredPlatformIndex, &m_clData->m_platformId);
		}

		oclCHECKERROR(ciErrNum, CL_SUCCESS);

		int numDev = b3OpenCLUtils::getNumDevices(m_clData->m_clContext);

		if (numDev > 0)
		{
			m_clData->m_clDevice = b3OpenCLUtils::getDevice(m_clData->m_clContext, 0);
			m_clData->m_clQueue = clCreateCommandQueue(m_clData->m_clContext, m_clData->m_clDevice, 0, &ciErrNum);
			oclCHECKERROR(ciErrNum, CL_SUCCESS);

			b3OpenCLDeviceInfo info;
			b3OpenCLUtils::getDeviceInfo(m_clData->m_clDevice, &info);
			m_clData->m_clDeviceName = info.m_deviceName;
			m_clData->m_clInitialized = true;
		}
	}

	virtual void initVulkan() {
		// Vulkan required extensions
		//assert(glfwVulkanSupported() == 1);
		uint32_t count{0};

		// Requesting Vulkan extensions and layers
		nvvk::ContextCreateInfo contextInfo;
		//contextInfo.verboseUsed = true;
		contextInfo.setVersion(1, 2);                       // Using Vulkan 1.2
		contextInfo.addInstanceLayer("VK_LAYER_LUNARG_monitor", true);              // FPS in titlebar
		contextInfo.addInstanceExtension(VK_EXT_DEBUG_UTILS_EXTENSION_NAME, true);  // Allow debug names
		//contextInfo.addDeviceExtension(VK_KHR_SURFACE_EXTENSION_NAME); /// (this is instance actually)
		//contextInfo.addDeviceExtension(VK_KHR_SWAPCHAIN_EXTENSION_NAME);            // Enabling ability to present rendering

		//requesting raytracing extensions
		// #VKRay: Activate the ray tracing extension
		VkPhysicalDeviceAccelerationStructureFeaturesKHR accelFeature{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR};
		contextInfo.addDeviceExtension(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME, false, &accelFeature); // To build acceleration structures
		VkPhysicalDeviceRayTracingPipelineFeaturesKHR rtPipelineFeature{};
		rtPipelineFeature.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
		contextInfo.addDeviceExtension(VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME, false, &rtPipelineFeature); // To use vkCmdTraceRaysKHR
		contextInfo.addDeviceExtension(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME); // Required by ray tracing pipeline

		// Creating Vulkan base application
		nvvk::Context vkctx{};
		vkctx.initInstance(contextInfo);
		// Find all compatible devices
		auto compatibleDevices = vkctx.getCompatibleDevices(contextInfo);
		assert(!compatibleDevices.empty());
		// Use a compatible device
		bool successful = vkctx.initDevice(compatibleDevices[0], contextInfo);
		if (successful) {
			b3Printf("Successfully initialized Vulkan device!\n");
		}

		//setupVulkan(vkctx.m_instance, vkctx.m_device, vkctx.m_physicalDevice, vkctx.m_queueGCT.familyIndex);
		setupVulkan(vkctx);
	}

	void setupVulkan(const nvvk::Context& context) {
		m_vkContext.m_instance = context.m_instance;
		m_vkContext.m_device = context.m_device;
		m_vkContext.m_physicalDevice = context.m_physicalDevice;
		m_vkContext.m_queueIndex = context.m_queueGCT.familyIndex;
		vkGetDeviceQueue(m_vkContext.m_device, m_vkContext.m_queueIndex, 0, &(m_vkContext.m_queue));
		
		VkCommandPoolCreateInfo poolCreateInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
		poolCreateInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
		vkCreateCommandPool(context.m_device, &poolCreateInfo, nullptr, &(m_vkContext.cmdPool));

		m_alloc.init(m_vkContext.m_instance, m_vkContext.m_device, m_vkContext.m_physicalDevice);
		m_vkContext.m_pAlloc = &m_alloc;
		m_rtBuilder.setup(m_vkContext.m_device, m_vkContext.m_pAlloc, m_vkContext.m_queueIndex);
		m_vkContext.m_pRtBuilder = &m_rtBuilder;

		// TODO: pipelinecachecreateinfo
	}

	virtual void exitCL()
	{
		if (m_clData && m_clData->m_clInitialized)
		{
			clReleaseCommandQueue(m_clData->m_clQueue);
			clReleaseContext(m_clData->m_clContext);
			m_clData->m_clInitialized = false;
		}
	}

	virtual void exitVulkan() {

	}

	virtual void renderScene()
	{
		if (m_guiHelper->getRenderInterface())
		{
			m_guiHelper->getRenderInterface()->renderScene();
		}
	}

	virtual void physicsDebugDraw(int debugDrawFlags)
	{
	}

	virtual bool keyboardCallback(int key, int state)
	{
		return false;  //don't handle this key
	}

	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}

	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

		if (!renderer)
		{
			b3Assert(0);
			return false;
		}

		CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;

		if (state == 1)
		{
			if (button == 0 && (!window->isModifierKeyPressed(B3G_ALT) && !window->isModifierKeyPressed(B3G_CONTROL)))
			{
				/*btVector3 camPos;
				renderer->getActiveCamera()->getCameraPosition(camPos);

				btVector3 rayFrom = camPos;
				btVector3 rayTo = getRayTo(int(x),int(y));

				pickBody(rayFrom, rayTo);
				*/
			}
		}
		else
		{
			if (button == 0)
			{
				//				removePickingConstraint();
				//remove p2p
			}
		}

		//printf("button=%d, state=%d\n",button,state);
		return false;
	}
};

#endif  //COMMON_MULTI_BODY_SETUP_H
