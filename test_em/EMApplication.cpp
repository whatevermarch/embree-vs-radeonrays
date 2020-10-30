#include <chrono>
#include <iostream>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "test_em/EMApplication.h"

#define TILE_SIZE 8


using namespace std::chrono;

void errorFunction(void* userPtr, enum RTCError error, const char* str)
{
    printf("error %d: %s\n", error, str);
}


EMApplication::EMApplication()
{
    RTCDevice device = rtcNewDevice(NULL);
    if (!device)
        printf("error %d: cannot create device\n", rtcGetDeviceError(NULL));
    rtcSetDeviceErrorFunction(device, errorFunction, NULL);

    device_ = RTCScopedObject<RTCDevice>(device, [](RTCDevice device) { rtcReleaseDevice(device); });
}

void EMApplication::Run(uint32_t res, MeshData* mesh_data, std::vector<uint32_t>& out_data)
{
    std::cout << "Running Embree.." << std::endl;

    //  construct scene object
    RTCScene scene = rtcNewScene(device_.get());

    //  padding data
    size_t num_vertices = mesh_data->positions.size() / 3;
    std::vector<float> pos_data(num_vertices * 4);
    size_t num_indices = mesh_data->indices.size() / 3;
    std::vector<uint32_t> idx_data(num_indices * 4);
     for(size_t i = 0; i < num_vertices; i++)
     {
         size_t offset_org = i * 3, offset_dst = i * 4;
         pos_data[offset_dst] = mesh_data->positions[offset_org];
         pos_data[offset_dst + 1] = mesh_data->positions[offset_org + 1];
         pos_data[offset_dst + 2] = mesh_data->positions[offset_org + 2];
         pos_data[offset_dst + 3] = 0.f;
     }
     for(size_t i = 0; i < num_indices; i++)
     {
         size_t offset_org = i * 3, offset_dst = i * 4;
         idx_data[offset_dst] = mesh_data->indices[offset_org];
         idx_data[offset_dst + 1] = mesh_data->indices[offset_org + 1];
         idx_data[offset_dst + 2] = mesh_data->indices[offset_org + 2];
         idx_data[offset_dst + 3] = 0.f;
     }
     std::cout << "Rearrange geometry data successed!" << std::endl;

    //  construct geometry
    RTCGeometry geom = rtcNewGeometry(device_.get(), RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
        pos_data.data(), 0, sizeof(float) * 4, num_vertices);
    rtcSetSharedGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
        idx_data.data(), 0, sizeof(uint32_t) * 4, num_indices);
    rtcCommitGeometry(geom);

    //  commit geometry to the scene
    uint32_t geom_id = rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);

    //  construct ray intersection context
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    //  construct rays
    using RayHit = RTCRayHit;
    std::vector<RayHit> queries(res * res);
    
#ifdef _OPENMP
    //  subdivide all rays into many tiles
    int num_space_x = (res + TILE_SIZE - 1) / TILE_SIZE;
    int num_space_y = (res + TILE_SIZE - 1) / TILE_SIZE;
    int num_tiles = num_space_x * num_space_y;
    std::vector<int> num_rays_per_tile(num_tiles);

    #pragma omp parallel for schedule(guided, 1)
    for (int tile_idx = 0; tile_idx < num_tiles; tile_idx++)
    {
        int init_x = (tile_idx % num_space_x) * TILE_SIZE;
        int init_y = (tile_idx / num_space_x) * TILE_SIZE;

        //  determine the number of rays in this tile
        int num_rays_x = (res - init_x < TILE_SIZE) ? (res - init_x) : TILE_SIZE,
            num_rays_y = (res - init_y < TILE_SIZE) ? (res - init_y) : TILE_SIZE;
        num_rays_per_tile[tile_idx] = num_rays_x * num_rays_y;

        auto ray_itr = queries.begin() + (TILE_SIZE * TILE_SIZE) * tile_idx;

        for (int y = init_y; y < init_y + num_rays_y; y++)
        {
            for (int x = init_x; x < init_x + num_rays_x; x++)
            {
                RayHit& ray = *(ray_itr++);
                ray.ray.org_x = 0.f;
                ray.ray.org_y = 250.f;
                ray.ray.org_z = 0.f;

                ray.ray.dir_x = -1.f;
                ray.ray.dir_y = -1.f + (2.f / res) * y;
                ray.ray.dir_z = -1.f + (2.f / res) * x;

                ray.ray.tnear = 0.001f;
                ray.ray.tfar = 100000.f;

                ray.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                ray.hit.primID = RTC_INVALID_GEOMETRY_ID;
            }
        }
    }
#else
    for (int y = 0; y < res; ++y)
    {
        for (int x = 0; x < res; ++x)
        {
            auto i = res * y + x;

            queries[i].ray.org_x = 0.f;
            queries[i].ray.org_y = 250.f;
            queries[i].ray.org_z = 0.f;

            queries[i].ray.dir_x = -1.f;
            queries[i].ray.dir_y = -1.f + (2.f / res) * y;
            queries[i].ray.dir_z = -1.f + (2.f / res) * x;

            queries[i].ray.tnear = 0.001f;
            queries[i].ray.tfar = 100000.f;

            queries[i].hit.geomID = RTC_INVALID_GEOMETRY_ID;
            queries[i].hit.primID = RTC_INVALID_GEOMETRY_ID;
        }
    }
#endif

    std::cout << "construct rays success!" << std::endl;

    //  trace ray streams (warm up)
    rtcIntersect1M(scene, &context, queries.data(), res * res, sizeof(RayHit));

    std::cout << "start timing..." << std::endl;
    //  measure time
    auto time_start = high_resolution_clock::now();

#ifdef _OPENMP
    #pragma omp parallel for schedule(guided, 1)
    for (int tile_idx = 0; tile_idx < num_tiles; tile_idx ++)
    {
        auto ray_itr = queries.begin() + (TILE_SIZE * TILE_SIZE) * tile_idx;

        rtcIntersect1M(scene, &context, &(*ray_itr), num_rays_per_tile[tile_idx], sizeof(RayHit));
    }
#else
    for (int i = 0; i < 10; i++)
        rtcIntersect1M(scene, &context, queries.data(), res * res, sizeof(RayHit));
#endif

    auto time_end = high_resolution_clock::now();

    duration<double, std::milli> delta_time = time_end - time_start;

    std::cout << "Trace time: " << delta_time.count() << "ms\n";
	std::cout << "Throughput: " << (res * res) / (delta_time.count() * 1e-3) * 1e-6 * 10 << " MRays/s\n";

#ifdef _OPENMP
    //  gather result
    for (int y = 0; y < res; ++y)
    {
        for (int x = 0; x < res; ++x)
        {
            int wi = res * (res - 1 - y) + x;

            int tile_idx_x = x / TILE_SIZE,
                tile_idx_y = y / TILE_SIZE;
            int tile_idx = tile_idx_y * num_space_x + tile_idx_x;
            int pix_idx_x = x % TILE_SIZE,
                pix_idx_y = y % TILE_SIZE;
            int num_pix_x = (res - x < TILE_SIZE) ? (res - x) : TILE_SIZE;
            int pix_idx = pix_idx_y * num_pix_x + pix_idx_x;
            int i = (TILE_SIZE * TILE_SIZE) * tile_idx + pix_idx;

            if (queries[i].hit.geomID != RTC_INVALID_GEOMETRY_ID)
            {
                out_data[wi] = 0xff000000 | (uint32_t(queries[i].hit.u * 255) << 8) |
                    (uint32_t(queries[i].hit.v * 255) << 16);
            }
            else
            {
                out_data[wi] = 0xff101010;
            }
        }
    }
#else
    //  gather result
	for (int y = 0; y < res; ++y)
	{
		for (int x = 0; x < res; ++x)
		{
			int wi = res * (res - 1 - y) + x;
			int i = res * y + x;

			if (queries[i].hit.geomID != RTC_INVALID_GEOMETRY_ID)
			{
				out_data[wi] = 0xff000000 | (uint32_t(queries[i].hit.u * 255) << 8) |
					(uint32_t(queries[i].hit.v * 255) << 16);
			}
			else
			{
				out_data[wi] = 0xff101010;
			}
		}
	}
#endif

    rtcReleaseScene(scene);
}