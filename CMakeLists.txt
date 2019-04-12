project(UKF)

cmake_minimum_required(VERSION 3.5)
add_definitions(-std=c++0x)

set(sources src/main.cpp
            src/tools.cpp
            src/tools.hpp
            src/config.h
            src/update.hpp
            src/update.cpp
            src/statepredict.hpp
            src/statepredict.cpp
            src/stateupdate.hpp
            src/stateupdate.cpp
            src/measurementupdate.hpp
            src/measurementupdate.cpp)

add_executable(UKF ${sources})
