# Enable FetchContent.
include(FetchContent)

# Fetch boost.
set(BOOST_INCLUDE_LIBRARIES thread filesystem system program_options dynamic_bitset)
if(GENERALIZEDASSIGNMENTSOLVER_USE_SMSPP)
    # Header-only Boost libraries #include'd directly by SMS++/BundleSolver
    # (e.g. SMS++'s AbstractBlock.cpp includes <boost/algorithm/string.hpp>).
    # Boost's modular CMake build only exposes headers for libraries listed
    # here, even though the boost-*.tar.xz release ships all of Boost's
    # sources.
    list(APPEND BOOST_INCLUDE_LIBRARIES algorithm any bimap function functional multi_array preprocessor)
endif()
set(BOOST_ENABLE_CMAKE ON)
include(FetchContent)
FetchContent_Declare(
    Boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.84.0/boost-1.84.0.tar.xz
    EXCLUDE_FROM_ALL
    OVERRIDE_FIND_PACKAGE)
FetchContent_MakeAvailable(Boost)

# Boost fetched via FetchContent only defines per-component targets (e.g.
# Boost::filesystem), and Boost::headers is just an (essentially empty) meta
# package, NOT an aggregate of every other component's headers -- unlike a
# classic system-wide Boost install, where one include path exposes all of
# Boost regardless of which libraries are actually used. The "Boost::boost"
# compatibility target normally provided by a plain find_package(Boost) is
# not created either, so define it here, linked against every library we
# fetched, so dependencies (e.g. SMS++) that link against Boost::boost
# directly and #include whichever Boost headers they need can still find
# them all.
if(NOT TARGET Boost::boost)
    add_library(Boost::boost INTERFACE IMPORTED)
    foreach(_boost_lib IN LISTS BOOST_INCLUDE_LIBRARIES)
        target_link_libraries(Boost::boost INTERFACE Boost::${_boost_lib})
    endforeach()
endif()

# Fetch fontanf/optimizationtools.
FetchContent_Declare(
    optimizationtools
    GIT_REPOSITORY https://github.com/fontanf/optimizationtools.git
    GIT_TAG e8c379203792fcad764cce239986b325619475fd
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../optimizationtools/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(optimizationtools)

# Fetch fontanf/mathoptsolverscmake.
if(GENERALIZEDASSIGNMENTSOLVER_USE_CBC)
    set(MATHOPTSOLVERSCMAKE_USE_CBC ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_CLP)
    set(MATHOPTSOLVERSCMAKE_USE_CLP ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_HIGHS)
    set(MATHOPTSOLVERSCMAKE_USE_HIGHS ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_CPLEX)
    set(MATHOPTSOLVERSCMAKE_USE_CPLEX ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_KNITRO)
    set(MATHOPTSOLVERSCMAKE_USE_KNITRO ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_DLIB)
    set(MATHOPTSOLVERSCMAKE_USE_DLIB ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_CONICBUNDLE)
    set(MATHOPTSOLVERSCMAKE_USE_CONICBUNDLE ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_SMSPP)
    set(MATHOPTSOLVERSCMAKE_USE_SMSPP ON)
endif()
FetchContent_Declare(
    mathoptsolverscmake
    GIT_REPOSITORY https://github.com/fontanf/mathoptsolverscmake.git
    GIT_TAG 7788a867fdfdd853ad9c0df7f0cdf15411a0f071
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../mathoptsolverscmake/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(mathoptsolverscmake)

# Fetch fontanf/columngenerationsolver.
if(GENERALIZEDASSIGNMENTSOLVER_USE_CLP)
    set(COLUMNGENERATIONSOLVER_USE_CLP ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_HIGHS)
    set(COLUMNGENERATIONSOLVER_USE_HIGHS ON)
endif()
if(GENERALIZEDASSIGNMENTSOLVER_USE_CPLEX)
    set(COLUMNGENERATIONSOLVER_USE_CPLEX ON)
endif()
FetchContent_Declare(
    columngenerationsolver
    GIT_REPOSITORY https://github.com/fontanf/columngenerationsolver.git
    GIT_TAG 556971624c92c23c961371ce5721ff9eb920a874
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
