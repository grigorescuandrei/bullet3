// Vulkan

#pragma once

#include "vulkan/vulkan_core.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <nvvk/context_vk.hpp>
#include "nvvk/raytraceKHR_vk.hpp"
#include "nvvk/buffers_vk.hpp"

struct b3VulkanContext {
	VkInstance m_instance;
	VkDevice m_device;
	VkPhysicalDevice m_physicalDevice;
	uint32_t m_queueIndex;
	VkQueue m_queue;
	VkCommandPool cmdPool;

	nvvk::ResourceAllocatorDma* m_pAlloc; // TODO: may need to replace this without DMA
	nvvk::RaytracingBuilderKHR* m_pRtBuilder;
};