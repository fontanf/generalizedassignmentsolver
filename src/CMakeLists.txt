add_library(GeneralizedAssignmentSolver_generalizedassignment)
target_sources(GeneralizedAssignmentSolver_generalizedassignment PRIVATE
    instance.cpp
    instance_builder.cpp
    solution.cpp
    algorithm_formatter.cpp)
target_include_directories(GeneralizedAssignmentSolver_generalizedassignment PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(GeneralizedAssignmentSolver_generalizedassignment PUBLIC
    OptimizationTools::utils)
add_library(GeneralizedAssignmentSolver::generalizedassignment ALIAS GeneralizedAssignmentSolver_generalizedassignment)

add_subdirectory(algorithms)

if(GENERALIZEDASSIGNMENTSOLVER_BUILD_MAIN)
    add_executable(GeneralizedAssignmentSolver_main)
    target_sources(GeneralizedAssignmentSolver_main PRIVATE
        main.cpp)
    target_link_libraries(GeneralizedAssignmentSolver_main PUBLIC
        GeneralizedAssignmentSolver_greedy
        GeneralizedAssignmentSolver_milp
        GeneralizedAssignmentSolver_lagrangian_relaxation
        GeneralizedAssignmentSolver_local_search
        GeneralizedAssignmentSolver_column_generation
        Boost::program_options)
    if(GENERALIZEDASSIGNMENTSOLVER_USE_KNITRO)
        target_compile_definitions(GeneralizedAssignmentSolver_main PUBLIC
            KNITRO_FOUND=1)
        target_link_libraries(GeneralizedAssignmentSolver_main PUBLIC
            GeneralizedAssignmentSolver_milp_knitro)
    endif()
    set_target_properties(GeneralizedAssignmentSolver_main PROPERTIES OUTPUT_NAME "generalizedassignmentsolver")
    install(TARGETS GeneralizedAssignmentSolver_main)

    add_executable(GeneralizedAssignmentSolver_checker)
    target_sources(GeneralizedAssignmentSolver_checker PRIVATE
        checker.cpp)
    target_link_libraries(GeneralizedAssignmentSolver_checker PUBLIC
        GeneralizedAssignmentSolver_generalizedassignment
        Boost::program_options)
    set_target_properties(GeneralizedAssignmentSolver_checker PROPERTIES OUTPUT_NAME "generalizedassignmentsolver_checker")
    install(TARGETS GeneralizedAssignmentSolver_checker)
endif()
