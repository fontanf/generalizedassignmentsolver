add_library(GeneralizedAssignmentSolver_greedy)
target_sources(GeneralizedAssignmentSolver_greedy PRIVATE
    greedy.cpp)
target_include_directories(GeneralizedAssignmentSolver_greedy PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(GeneralizedAssignmentSolver_greedy PUBLIC
    GeneralizedAssignmentSolver_generalizedassignment)
add_library(GeneralizedAssignmentSolver::greedy ALIAS GeneralizedAssignmentSolver_greedy)

add_library(GeneralizedAssignmentSolver_random)
target_sources(GeneralizedAssignmentSolver_random PRIVATE
    random.cpp)
target_include_directories(GeneralizedAssignmentSolver_random PUBLIC
    ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(GeneralizedAssignmentSolver_random PUBLIC
    GeneralizedAssignmentSolver_generalizedassignment)
add_library(GeneralizedAssignmentSolver::random ALIAS GeneralizedAssignmentSolver_random)

if(GENERALIZEDASSIGNMENTSOLVER_USE_CBC)
    add_library(GeneralizedAssignmentSolver_milp_cbc)
    target_sources(GeneralizedAssignmentSolver_milp_cbc PRIVATE
        milp_cbc.cpp)
    target_include_directories(GeneralizedAssignmentSolver_milp_cbc PUBLIC
        ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(GeneralizedAssignmentSolver_milp_cbc PUBLIC
        GeneralizedAssignmentSolver_generalizedassignment
        MathOptSolversCMake::cbc)
    add_library(GeneralizedAssignmentSolver::milp_cbc ALIAS GeneralizedAssignmentSolver_milp_cbc)
endif()

if(GENERALIZEDASSIGNMENTSOLVER_USE_GUROBI)
    add_library(GeneralizedAssignmentSolver_milp_gurobi)
    target_sources(GeneralizedAssignmentSolver_milp_gurobi PRIVATE
        milp_gurobi.cpp)
    target_include_directories(GeneralizedAssignmentSolver_milp_gurobi PUBLIC
        ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(GeneralizedAssignmentSolver_milp_gurobi PUBLIC
        GeneralizedAssignmentSolver_generalizedassignment
        MathOptSolversCMake::gurobi)
    add_library(GeneralizedAssignmentSolver::milp_gurobi ALIAS GeneralizedAssignmentSolver_milp_gurobi)
endif()

if(GENERALIZEDASSIGNMENTSOLVER_USE_CPLEX)
    add_library(GeneralizedAssignmentSolver_milp_cplex)
    target_sources(GeneralizedAssignmentSolver_milp_cplex PRIVATE
        milp_cplex.cpp)
    target_include_directories(GeneralizedAssignmentSolver_milp_cplex PUBLIC
        ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(GeneralizedAssignmentSolver_milp_cplex PUBLIC
        GeneralizedAssignmentSolver_generalizedassignment
        MathOptSolversCMake::cplex)
    add_library(GeneralizedAssignmentSolver::milp_cplex ALIAS GeneralizedAssignmentSolver_milp_cplex)
endif()

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
    KnapsackSolver::knapsack::dynamic_programming_primal_dual)
add_library(GeneralizedAssignmentSolver::column_generation ALIAS GeneralizedAssignmentSolver_column_generation)
