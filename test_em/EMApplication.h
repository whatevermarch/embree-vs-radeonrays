#pragma once

#include <embree3/rtcore.h>

#include "../mesh_data.h"

template <typename T>
using RTCScopedObject = std::shared_ptr<std::remove_pointer_t<T>>;

class EMApplication
{
public:
    EMApplication();

    virtual void Run(uint32_t res, MeshData* mesh_data, std::vector<uint32_t>& out_data);

private:
    // Embree Data.
    RTCScopedObject<RTCDevice>      device_;
};
