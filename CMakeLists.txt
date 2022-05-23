#init
cmake_minimum_required(VERSION 3.21)
project(DeepLProject)
set(CMAKE_CXX_STANDARD 14)

#copy the default settings file to the build directory
file(COPY defaultConfig.conf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY defaultPy.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


#include python libraries

set(Python3_FIND_VIRTUALENV ONLY)
set(Python3_ROOT_DIR "/Users/jordan/opt/anaconda3/envs/carla/")
find_package(Python3 COMPONENTS Interpreter Development 3.8)
#include_directories(${PYTHON_INCLUDE_DIRS})

#include the dependencies in the build
include_directories(${PROJECT_SOURCE_DIR}/lib/Eigen
        ${PROJECT_SOURCE_DIR}/lib/rapidJSON/include
        ${PROJECT_SOURCE_DIR}/modules)

add_executable(DeepLProject
        main.cpp
        modules/network/perceptron.cpp
        modules/MNist/MNist.cpp)

target_link_libraries(DeepLProject PUBLIC Python3::Python)