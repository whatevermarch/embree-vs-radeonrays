#pragma once

#include <chrono>
#include <iostream>

#include "mesh_data.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "test_em/EMApplication.h"
#include "test_rr/RRApplication.h"

#define MESH_DATA_PATH "D:\\Documents\\Researches\\embree-vs-radeonrays\\sponza\\sponza.obj"

using namespace std::chrono;

class Application
{
public:
	Application()
	{
		std::cout << "Start loading mesh.." << std::endl;

		mesh_data_ = new MeshData(MESH_DATA_PATH);

		std::cout << "Mesh succesfully loaded!" << std::endl;

		em_ = new EMApplication();
		rr_ = new RRApplication();
	}
	virtual ~Application()
	{ 
		delete em_;
		delete rr_;
		delete mesh_data_; 
	}

	void Run()
	{
		constexpr uint32_t kResolution = 1024;
		out_data_.resize(kResolution * kResolution);

		em_->Run(kResolution, mesh_data_, out_data_);
		stbi_write_jpg("res_embree.jpg", kResolution, kResolution, 4, out_data_.data(), 120);

		rr_->Run(kResolution, mesh_data_, out_data_);
		stbi_write_jpg("res_radeon.jpg", kResolution, kResolution, 4, out_data_.data(), 120);
	}

private:
	EMApplication* em_ = nullptr;
	RRApplication* rr_ = nullptr;

	MeshData* mesh_data_ = nullptr;
	std::vector<uint32_t> out_data_;
};
