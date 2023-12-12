cmake_minimum_required(VERSION 3.12)

project(controller-in-cloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

### Franka Dynamics Library
add_library(Dynamics STATIC)

target_sources(Dynamics PRIVATE
  include/franka_dynamics.cpp
)

target_link_libraries(Dynamics Eigen3::Eigen)


set(CODESET
  cloud_controller
  # test_tdpa
)


foreach(code ${CODESET})
  add_executable(${code} 
    src/${code}.cpp
  )

  target_include_directories(${code} PUBLIC
    include
  )

  target_link_libraries(${code} 
    Eigen3::Eigen 
    Threads::Threads 
    nlohmann_json::nlohmann_json
    Dynamics
  )
endforeach()