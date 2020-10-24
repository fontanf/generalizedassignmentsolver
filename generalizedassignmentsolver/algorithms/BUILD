load("//generalizedassignmentsolver:variables.bzl", "STDCPP",
         "COINOR_COPTS", "COINOR_DEP",
         "CPLEX_COPTS", "CPLEX_DEP", "CPOPTIMIZER_DEP",
         "GUROBI_COPTS", "GUROBI_DEP",
         "GECODE_COPTS", "GECODE_DEP",
         "LOCALSOLVER_COPTS", "LOCALSOLVER_DEP")

cc_library(
        name = "linrelax_clp",
        hdrs = ["linrelax_clp.hpp"],
        srcs = ["linrelax_clp.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                ":branchandcut_cbc",
        ] + COINOR_DEP,
        copts = COINOR_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "linrelax_clp_test",
        srcs = ["linrelax_clp_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":linrelax_clp",
        ],
        copts = COINOR_COPTS,
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "linrelax_gurobi",
        hdrs = ["linrelax_gurobi.hpp"],
        srcs = ["linrelax_gurobi.cpp"],
        copts = GUROBI_COPTS,
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver"
        ] + GUROBI_DEP,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "linrelax_gurobi_test",
        srcs = ["linrelax_gurobi_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":linrelax_gurobi",
        ],
        copts = GUROBI_COPTS,
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "lagrelax_bundle",
        hdrs = ["lagrelax_bundle.hpp"],
        srcs = ["lagrelax_bundle.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                "@knapsacksolver//knapsacksolver/algorithms:minknap",
        ] + COINOR_DEP,
        copts = COINOR_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "lagrelax_bundle_test",
        srcs = ["lagrelax_bundle_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":lagrelax_bundle",
        ],
        copts = COINOR_COPTS,
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "lagrelax_volume",
        hdrs = ["lagrelax_volume.hpp"],
        srcs = ["lagrelax_volume.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                "@knapsacksolver//knapsacksolver/algorithms:minknap",
        ] + COINOR_DEP,
        copts = COINOR_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "lagrelax_volume_test",
        srcs = ["lagrelax_volume_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":lagrelax_volume",
        ],
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "lagrelax_lbfgs",
        hdrs = ["lagrelax_lbfgs.hpp"],
        srcs = ["lagrelax_lbfgs.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                "@knapsacksolver//knapsacksolver/algorithms:minknap",
                "@knapsacksolver//knapsacksolver/algorithms:bellman",
                "@dlib//:dlib",
        ],
        visibility = ["//visibility:public"],
)

cc_test(
        name = "lagrelax_lbfgs_test",
        srcs = ["lagrelax_lbfgs_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":lagrelax_lbfgs",
        ],
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "greedy",
        hdrs = ["greedy.hpp"],
        srcs = ["greedy.cpp"],
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        visibility = ["//visibility:public"],
)

################################################################################

cc_library(
        name = "random",
        hdrs = ["random.hpp"],
        srcs = ["random.cpp"],
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        visibility = ["//visibility:public"],
)

################################################################################

cc_library(
        name = "localsearch",
        hdrs = ["localsearch.hpp"],
        srcs = ["localsearch.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                ":linrelax_clp",
                ":random",
        ],
        visibility = ["//visibility:public"],
)

################################################################################

cc_library(
        name = "repair",
        hdrs = ["repair.hpp"],
        srcs = ["repair.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                ":linrelax_clp",
                ":lagrelax_lbfgs",
                ":random",
        ],
        copts = COINOR_COPTS,
        visibility = ["//visibility:public"],
)

################################################################################

cc_library(
        name = "columngeneration",
        hdrs = ["columngeneration.hpp"],
        srcs = ["columngeneration.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                "@knapsacksolver//knapsacksolver/algorithms:minknap",
                "@knapsacksolver//knapsacksolver/algorithms:bellman",
        ] + CPLEX_DEP + COINOR_DEP,
        copts = CPLEX_COPTS + COINOR_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "columngeneration_test",
        srcs = ["columngeneration_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":columngeneration",
        ],
        copts = CPLEX_COPTS + COINOR_COPTS,
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "colgenheuristics",
        hdrs = ["colgenheuristics.hpp"],
        srcs = ["colgenheuristics.cpp"],
        deps = [
                ":columngeneration",
                ":greedy",
        ],
        copts = STDCPP + COINOR_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "colgenheuristics_test",
        srcs = ["colgenheuristics_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":colgenheuristics",
        ],
        copts = COINOR_COPTS,
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "localsolver",
        hdrs = ["localsolver.hpp"],
        srcs = ["localsolver.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
        ] + LOCALSOLVER_DEP,
        copts = LOCALSOLVER_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "localsolver_test",
        srcs = ["localsolver_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":localsolver",
        ],
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "branchandcut_cbc",
        hdrs = ["branchandcut_cbc.hpp"],
        srcs = ["branchandcut_cbc.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
        ] + COINOR_DEP,
        copts = COINOR_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "branchandcut_cbc_test",
        srcs = ["branchandcut_cbc_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":branchandcut_cbc",
        ],
        copts = COINOR_COPTS,
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "branchandcut_cplex",
        hdrs = ["branchandcut_cplex.hpp"],
        srcs = ["branchandcut_cplex.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver"
        ] + CPLEX_DEP,
        copts = CPLEX_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "branchandcut_cplex_test",
        srcs = ["branchandcut_cplex_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":branchandcut_cplex",
        ],
        copts = CPLEX_COPTS,
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "branchandcut_gurobi",
        hdrs = ["branchandcut_gurobi.hpp"],
        srcs = ["branchandcut_gurobi.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
        ] + GUROBI_DEP,
        copts = GUROBI_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "branchandcut_gurobi_test",
        srcs = ["branchandcut_gurobi_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":branchandcut_gurobi",
        ],
        copts = GUROBI_COPTS,
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "branchandprice",
        hdrs = ["branchandprice.hpp"],
        srcs = ["branchandprice.cpp"],
        deps = [
                ":columngeneration",
                ":greedy",
        ],
        copts = STDCPP + COINOR_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "branchandprice_test",
        srcs = ["branchandprice_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":branchandprice",
        ],
        copts = COINOR_COPTS,
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "constraintprogramming_cplex",
        hdrs = ["constraintprogramming_cplex.hpp"],
        srcs = ["constraintprogramming_cplex.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
        ] + CPOPTIMIZER_DEP,
        copts = CPLEX_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "constraintprogramming_cplex_test",
        srcs = ["constraintprogramming_cplex_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":constraintprogramming_cplex",
        ],
        copts = COINOR_COPTS,
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "constraintprogramming_gecode",
        hdrs = ["constraintprogramming_gecode.hpp"],
        srcs = ["constraintprogramming_gecode.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver"
        ] + GECODE_DEP,
        copts = GECODE_COPTS,
        visibility = ["//visibility:public"],
)

cc_test(
        name = "constraintprogramming_gecode_test",
        srcs = ["constraintprogramming_gecode_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":constraintprogramming_gecode",
        ],
        copts = GECODE_COPTS,
        tags = ["requires-network"],
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "algorithms",
        hdrs = ["algorithms.hpp"],
        srcs = ["algorithms.cpp"],
        deps = [
                ":linrelax_clp",
                ":linrelax_gurobi",
                ":lagrelax_volume",
                ":lagrelax_bundle",
                ":lagrelax_lbfgs",
                ":columngeneration",
                ":branchandcut_cbc",
                ":branchandcut_cplex",
                ":branchandcut_gurobi",
                ":branchandprice",
                ":constraintprogramming_gecode",
                ":constraintprogramming_cplex",
                ":random",
                ":greedy",
                ":repair",
                ":localsearch",
                ":localsolver",
                ":colgenheuristics",
        ],
        copts = COINOR_COPTS + CPLEX_COPTS + GUROBI_COPTS + GECODE_COPTS + LOCALSOLVER_COPTS,
        visibility = ["//visibility:public"],
)

