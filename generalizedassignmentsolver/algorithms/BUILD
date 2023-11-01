load("//generalizedassignmentsolver:variables.bzl", "STDCPP",
         "COINOR_COPTS", "COINOR_DEP",
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
        ] + COINOR_DEP,
        copts = COINOR_COPTS,
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
        name = "algorithms",
        hdrs = [
                "linrelax_clp.hpp",
                "lagrelax_volume.hpp",
                "lagrelax_lbfgs.hpp",
                "repair.hpp",
                "column_generation.hpp",
                "localsolver.hpp",
                "constraint_programming_cplex.hpp",
                "constraint_programming_gecode.hpp",
                "algorithms.hpp"],
        srcs = [
                "linrelax_clp.cpp",
                "lagrelax_volume.cpp",
                "lagrelax_lbfgs.cpp",
                "local_search.cpp",
                "repair.cpp",
                "column_generation.cpp",
                "constraint_programming_cplex.cpp",
                "constraint_programming_gecode.cpp",
                "algorithms.cpp"],
        deps = [
                ":greedy",
                ":random",
                ":milp_cbc",
                ":milp_cplex",
                ":milp_gurobi",
                ":milp_knitro",
                ":local_search",
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                "@columngenerationsolver//columngenerationsolver:columngenerationsolver",
                "@knapsacksolver//knapsacksolver/algorithms:dynamic_programming_primal_dual",
                "@knapsacksolver//knapsacksolver/algorithms:dynamic_programming_bellman",
                "@dlib//:dlib",
                "@boost//:program_options",
        ] + COINOR_DEP + CPLEX_DEP + GUROBI_DEP + CPOPTIMIZER_DEP + GECODE_DEP + LOCALSOLVER_DEP + KNITRO_DEP,
        copts = COINOR_COPTS + CPLEX_COPTS + GUROBI_COPTS + GECODE_COPTS + LOCALSOLVER_COPTS + KNITRO_COPTS,
        visibility = ["//visibility:public"],
)
