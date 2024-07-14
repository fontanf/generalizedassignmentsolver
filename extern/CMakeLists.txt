# Enable FetchContent.
include(FetchContent)

# Fetch boost.
set(BOOST_INCLUDE_LIBRARIES thread filesystem system program_options dynamic_bitset)
set(BOOST_ENABLE_CMAKE ON)
include(FetchContent)
FetchContent_Declare(
    Boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.84.0/boost-1.84.0.tar.xz
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(Boost)

# Fetch fontanf/optimizationtools.
FetchContent_Declare(
    optimizationtools
    GIT_REPOSITORY https://github.com/fontanf/optimizationtools.git
    GIT_TAG a0973a7dfa64b9d305f75879c80d252e714ce2cf
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../optimizationtools/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(optimizationtools)

# Fetch fontanf/mathoptsolverscmake.
if(GENERALIZEDASSIGNMENTSOLVER_USE_CBC)
    set(MATHOPTSOLVERSCMAKE_USE_CBC ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_CPLEX)
    set(MATHOPTSOLVERSCMAKE_USE_CPLEX ON)
endif()
FetchContent_Declare(
    mathoptsolverscmake
    GIT_REPOSITORY https://github.com/fontanf/mathoptsolverscmake.git
    GIT_TAG 80cd13a843e2c10684abbfe76cf9aae0d71ff79d
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../mathoptsolverscmake/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(mathoptsolverscmake)

# Fetch fontanf/columngenerationsolver.
if(GENERALIZEDASSIGNMENTSOLVER_USE_CBC)
    set(COLUMNGENERATIONSOLVER_USE_CLP ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_CPLEX)
    set(COLUMNGENERATIONSOLVER_USE_CPLEX ON)
endif()
FetchContent_Declare(
    columngenerationsolver
    GIT_REPOSITORY https://github.com/fontanf/columngenerationsolver.git
    GIT_TAG 9c57300135f8d1ec2a44de4aeb53e0056d24f2f3
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../columngenerationsolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(columngenerationsolver)

# Fetch fontanf/localsearchsolver.
FetchContent_Declare(
    localsearchsolver
    GIT_REPOSITORY https://github.com/fontanf/localsearchsolver.git
    GIT_TAG f7110fdd15fa1b9001006bfbf7a8ab53f925016d
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../localsearchsolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(localsearchsolver)

# Fetch fontanf/knitrocpp.
if (GENERALIZEDASSIGNMENTSOLVER_USE_KNITRO)
    FetchContent_Declare(
        knitrocpp
        GIT_REPOSITORY https://github.com/fontanf/knitrocpp.git
        GIT_TAG 6c63e3f3931329e2c1a2c22ce465fe8667516a36
        #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../knitrocpp/"
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(knitrocpp)
endif()
