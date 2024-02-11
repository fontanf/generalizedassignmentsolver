load("//generalizedassignmentsolver:variables.bzl", "STDCPP",
         "CBC_COPTS", "CBC_DEP",
         "CLP_COPTS", "CLP_DEP",
         "VOLUME_COPTS", "VOLUME_DEP",
         "CPLEX_COPTS", "CPLEX_DEP", "CPOPTIMIZER_DEP",
         "GUROBI_COPTS", "GUROBI_DEP",
         "GECODE_COPTS", "GECODE_DEP",
         "LOCALSOLVER_COPTS", "LOCALSOLVER_DEP",
         "KNITRO_COPTS", "KNITRO_DEP")

cc_library(
        name = "greedy",
        hdrs = ["greedy.hpp"],
        srcs = ["greedy.cpp"],
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        visibility = ["//visibility:public"],
)

cc_library(
        name = "random",
        hdrs = ["random.hpp"],
        srcs = ["random.cpp"],
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        visibility = ["//visibility:public"],
)

cc_library(
        name = "milp_cbc",
        hdrs = ["milp_cbc.hpp"],
        srcs = ["milp_cbc.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
        ] + CBC_DEP,
        copts = CBC_COPTS,
        visibility = ["//visibility:public"],
)

cc_library(
        name = "milp_cplex",
        hdrs = ["milp_cplex.hpp"],
        srcs = ["milp_cplex.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
        ] + CPLEX_DEP,
        copts = CPLEX_COPTS,
        visibility = ["//visibility:public"],
)

cc_library(
        name = "milp_gurobi",
        hdrs = ["milp_gurobi.hpp"],
        srcs = ["milp_gurobi.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
        ] + GUROBI_DEP,
        copts = GUROBI_COPTS,
        visibility = ["//visibility:public"],
)

cc_library(
        name = "milp_knitro",
        hdrs = ["milp_knitro.hpp"],
        srcs = ["milp_knitro.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
        ] + KNITRO_DEP,
        copts = KNITRO_COPTS,
        visibility = ["//visibility:public"],
)

cc_library(
        name = "local_search",
        hdrs = ["local_search.hpp"],
        srcs = ["local_search.cpp"],
        deps = [
                ":greedy",
                ":random",
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                "@localsearchsolver//localsearchsolver:localsearchsolver",
                "@optimizationtools//optimizationtools/containers:containers",
        ],
        visibility = ["//visibility:public"],
)

cc_library(
        name = "column_generation",
        hdrs = ["column_generation.hpp"],
        srcs = ["column_generation.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                "@columngenerationsolver//columngenerationsolver:columngenerationsolver",
                "@knapsacksolver//knapsacksolver/knapsack/algorithms:dynamic_programming_primal_dual",
                "@knapsacksolver//knapsacksolver/knapsack/algorithms:dynamic_programming_bellman",
        ],
        visibility = ["//visibility:public"],
)
