add_library(GeneralizedAssignmentSolver_greedy)
target_sources(GeneralizedAssignmentSolver_greedy PRIVATE
    greedy.cpp)
target_include_directories(GeneralizedAssignmentSolver_greedy PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(GeneralizedAssignmentSolver_greedy PUBLIC
    GeneralizedAssignmentSolver_generalizedassignment)
add_library(GeneralizedAssignmentSolver::greedy ALIAS GeneralizedAssignmentSolver_greedy)

add_library(GeneralizedAssignmentSolver_milp)
target_sources(GeneralizedAssignmentSolver_milp PRIVATE
    milp.cpp)
target_include_directories(GeneralizedAssignmentSolver_milp PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(GeneralizedAssignmentSolver_milp PUBLIC
    GeneralizedAssignmentSolver_generalizedassignment
    MathOptSolversCMake::milp)
add_library(GeneralizedAssignmentSolver::milp ALIAS GeneralizedAssignmentSolver_milp)

if(GENERALIZEDASSIGNMENTSOLVER_USE_KNITRO)
    add_library(GeneralizedAssignmentSolver_milp_knitro)
    target_sources(GeneralizedAssignmentSolver_milp_knitro PRIVATE
        milp_knitro.cpp)
    target_include_directories(GeneralizedAssignmentSolver_milp_knitro PUBLIC
        ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(GeneralizedAssignmentSolver_milp_knitro PUBLIC
        GeneralizedAssignmentSolver_generalizedassignment
        KnitroCpp::knitrocpp)
    add_library(GeneralizedAssignmentSolver::milp_knitro ALIAS GeneralizedAssignmentSolver_milp_knitro)
endif()

add_library(GeneralizedAssignmentSolver_local_search)
target_sources(GeneralizedAssignmentSolver_local_search PRIVATE
    local_search.cpp)
target_include_directories(GeneralizedAssignmentSolver_local_search PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(GeneralizedAssignmentSolver_local_search PUBLIC
    GeneralizedAssignmentSolver_generalizedassignment
    LocalSearchSolver::localsearchsolver)
add_library(GeneralizedAssignmentSolver::local_search ALIAS GeneralizedAssignmentSolver_local_search)

add_library(GeneralizedAssignmentSolver_column_generation)
target_sources(GeneralizedAssignmentSolver_column_generation PRIVATE
    column_generation.cpp)
target_include_directories(GeneralizedAssignmentSolver_column_generation PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(GeneralizedAssignmentSolver_column_generation PUBLIC
    GeneralizedAssignmentSolver_generalizedassignment
    ColumnGenerationSolver::columngenerationsolver
    KnapsackSolver::dynamic_programming_primal_dual)
add_library(GeneralizedAssignmentSolver::column_generation ALIAS GeneralizedAssignmentSolver_column_generation)

if(GENERALIZEDASSIGNMENTSOLVER_USE_DLIB)
    add_library(GeneralizedAssignmentSolver_lagrangian_relaxation_dlib)
    target_sources(GeneralizedAssignmentSolver_lagrangian_relaxation_dlib PRIVATE
        lagrangian_relaxation_dlib.cpp)
    target_include_directories(GeneralizedAssignmentSolver_lagrangian_relaxation_dlib PUBLIC
        ${PROJECT_SOURCE_DIR}/include)
    target_compile_definitions(GeneralizedAssignmentSolver_lagrangian_relaxation_dlib PUBLIC
        DLIB_FOUND=1)
    target_link_libraries(GeneralizedAssignmentSolver_lagrangian_relaxation_dlib PUBLIC
        GeneralizedAssignmentSolver_generalizedassignment
        dlib::dlib
        KnapsackSolver::dynamic_programming_bellman
        KnapsackSolver::dynamic_programming_primal_dual)
    add_library(GeneralizedAssignmentSolver::lagrangian_relaxation_dlib ALIAS GeneralizedAssignmentSolver_lagrangian_relaxation_dlib)
endif()
