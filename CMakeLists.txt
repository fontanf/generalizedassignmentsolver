cmake_minimum_required(VERSION 3.28.0)

project(GeneralizedAssignmentSolver LANGUAGES CXX)

# Build options.
option(GENERALIZEDASSIGNMENTSOLVER_BUILD_MAIN "Build main" ON)
option(GENERALIZEDASSIGNMENTSOLVER_BUILD_TEST "Build the unit tests" ON)

# Solver options.
option(GENERALIZEDASSIGNMENTSOLVER_USE_CLP "Use CLP" OFF)
option(GENERALIZEDASSIGNMENTSOLVER_USE_CBC "Use CBC" OFF)
option(GENERALIZEDASSIGNMENTSOLVER_USE_HIGHS "Use HIGHS" OFF)
option(GENERALIZEDASSIGNMENTSOLVER_USE_GUROBI "Use Gurobi" OFF)
option(GENERALIZEDASSIGNMENTSOLVER_USE_CPLEX "Use CPLEX" OFF)
option(GENERALIZEDASSIGNMENTSOLVER_USE_KNITRO "Use Knitro" OFF)
option(GENERALIZEDASSIGNMENTSOLVER_USE_DLIB "Use dlib" OFF)
option(GENERALIZEDASSIGNMENTSOLVER_USE_CONICBUNDLE "Use ConicBundle" OFF)

# Avoid FetchContent warning.
cmake_policy(SET CMP0135 NEW)

# Require C++14.
set(CMAKE_CXX_STANDARD 14)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set MSVC_RUNTIME_LIBRARY.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Add sub-directories.
add_subdirectory(extern)
add_subdirectory(src)
if(PACKINGSOLVER_BUILD_TEST)
    add_subdirectory(test)
endif()
