load("//generalizedassignmentsolver:variables.bzl", "STDCPP",
         "COINOR_COPTS", "COINOR_DEP",
         "CPLEX_COPTS", "CPLEX_DEP", "CPOPTIMIZER_DEP",
         "GUROBI_COPTS", "GUROBI_DEP",
         "GECODE_COPTS", "GECODE_DEP",
         "LOCALSOLVER_COPTS", "LOCALSOLVER_DEP")

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
        name = "localsearch",
        hdrs = ["localsearch.hpp"],
        srcs = ["localsearch.cpp"],
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
                "columngeneration.hpp",
                "localsolver.hpp",
                "constraintprogramming_cplex.hpp",
                "constraintprogramming_gecode.hpp",
                "algorithms.hpp"],
        srcs = [
                "linrelax_clp.cpp",
                "lagrelax_volume.cpp",
                "lagrelax_lbfgs.cpp",
                "localsearch.cpp",
                "repair.cpp",
                "columngeneration.cpp",
                "constraintprogramming_cplex.cpp",
                "constraintprogramming_gecode.cpp",
                "algorithms.cpp"],
        deps = [
                ":greedy",
                ":random",
                ":milp_cbc",
                ":milp_cplex",
                ":milp_gurobi",
                ":localsearch",
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                "@columngenerationsolver//columngenerationsolver:columngenerationsolver",
                "@knapsacksolver//knapsacksolver/algorithms:minknap",
                "@knapsacksolver//knapsacksolver/algorithms:bellman",
                "@dlib//:dlib",
                "@boost//:program_options",
        ] + COINOR_DEP + CPLEX_DEP + GUROBI_DEP + CPOPTIMIZER_DEP + GECODE_DEP + LOCALSOLVER_DEP,
        copts = COINOR_COPTS + CPLEX_COPTS + GUROBI_COPTS + GECODE_COPTS + LOCALSOLVER_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "test",
        srcs = [
                "linrelax_clp_test.cpp",
                "lagrelax_volume_test.cpp",
                "lagrelax_lbfgs_test.cpp",
                "columngeneration_test.cpp",
                "localsolver_test.cpp",
                "milp_cbc_test.cpp",
                "milp_cplex_test.cpp",
                "milp_gurobi_test.cpp",
                "constraintprogramming_cplex_test.cpp",
                "constraintprogramming_gecode_test.cpp",
        ],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":algorithms",
        ],
        copts = COINOR_COPTS + CPLEX_COPTS + GUROBI_COPTS + GECODE_COPTS + LOCALSOLVER_COPTS,
        tags = ["requires-network"],
        timeout = "moderate",
)

