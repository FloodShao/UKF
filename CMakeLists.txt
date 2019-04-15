project(UKF)

cmake_minimum_required(VERSION 3.5)
add_definitions(-std=c++0x)

set(sources src/main.cpp
            src/tools.cpp
            src/tools.hpp
            src/config.h
            src/statepredict.hpp
            src/statepredict.cpp
            src/stateupdate.hpp
            src/stateupdate.cpp
            src/measurementupdate.hpp
            src/measurementupdate.cpp
            src/fusionukf.hpp
            src/fusionukf.cpp
            src/datapoint.hpp
            src/datapoint.cpp
            src/usagecheck.cpp
            src/usagecheck.hpp)

add_executable(UKF ${sources})
