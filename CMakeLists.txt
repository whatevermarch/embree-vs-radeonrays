cmake_minimum_required(VERSION 3.12)

project(benchmark CXX)

#   embree_DIR is root directory
find_package(embree 3.0 REQUIRED)
find_package(Vulkan REQUIRED)

add_executable(benchmark
    main.cpp
    application.h
    test_em/EMApplication.h
    test_em/EMApplication.cpp
    test_rr/RRApplication.h
    test_rr/RRApplication.cpp
    mesh_data.h)

target_include_directories(benchmark PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

target_link_directories(benchmark PRIVATE
    ${PROJECT_SOURCE_DIR}/bin
    ${PROJECT_SOURCE_DIR}/lib
)

target_link_libraries(benchmark PRIVATE embree radeonrays Vulkan::Vulkan)
