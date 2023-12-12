cmake_minimum_required(VERSION 3.12)

project(controller-at-local CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(Franka REQUIRED HINTS ~/libfranka/build)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# pull JSON C++ library
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

# define executables
set(CODESET
  local_controller
)

foreach(code ${CODESET})
  add_executable(${code} 
    src/${code}.cpp
  )

  target_include_directories(${code} PUBLIC
    include
  )

  target_link_libraries(${code} 
    Franka::Franka 
    Eigen3::Eigen 
    Threads::Threads 
    nlohmann_json::nlohmann_json
  )
endforeach()
