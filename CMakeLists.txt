cmake_minimum_required(VERSION 3.21)
project(DeepLProject)

set(CMAKE_CXX_STANDARD 14)

add_executable(DeepLProject main.cpp perceptron.cpp MNist/MNist.cpp)
