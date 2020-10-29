#pragma once

#include <radeonrays_vlk.h>

#include "../mesh_data.h"

template <typename T>
using VkScopedObject = std::shared_ptr<std::remove_pointer_t<T>>;

class RRApplication
{
public:
	RRApplication();

	virtual void Run(uint32_t res, MeshData* mesh_data, std::vector<uint32_t>& out_data);

private:
	/// Allocate a region of memory of a given type.
	VkScopedObject<VkDeviceMemory> AllocateDeviceMemory(std::uint32_t memory_type_index, std::size_t size) const;

	/// Create buffer (w/o any allocations).
	VkScopedObject<VkBuffer> CreateBuffer(VkBufferUsageFlags usage, std::size_t size) const;

	/// Find memory index by its properties.
	uint32_t FindDeviceMemoryIndex(VkMemoryPropertyFlags flags) const;

	/// Upload the specified range of memory.
	template <typename TYPE>
	void UploadMemory(std::vector<TYPE> const& source, VkBuffer const& destination) const;

	// Vulkan data.
	VkScopedObject<VkInstance>    instance_;
	VkScopedObject<VkDevice>      device_;
	VkScopedObject<VkCommandPool> command_pool_;
	VkScopedObject<VkQueryPool>   query_pool_;
	VkPhysicalDevice              phdevice_;
	uint32_t                      queue_family_index_;
	float                         timestamp_period_;
	std::string device_name;
};
