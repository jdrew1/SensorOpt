#init
cmake_minimum_required(VERSION 3.21)
project(DeepLProject)
set(CMAKE_CXX_STANDARD 14)

#include python libraries
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})


#include the dependencies in the build
include_directories(${PROJECT_SOURCE_DIR}/lib/Eigen
                    ${PROJECT_SOURCE_DIR}/lib/rapidJSON/include
                    ${PROJECT_SOURCE_DIR}/modules)


#copy the default settings file to the build directory
file(COPY defaultConfig.conf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(DeepLProject
        main.cpp
        modules/network/perceptron.cpp
        modules/MNist/MNist.cpp)

target_link_libraries(DeepLProject ${PYTHON_LIBRARIES})