cc_library(
        name = "linrelax_clp",
        hdrs = ["linrelax_clp.hpp"],
        srcs = ["linrelax_clp.cpp"],
        deps = [
                "//generalizedassignmentsolver:generalizedassignmentsolver",
                ":branchandcut_cbc",
        ],
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//generalizedassignmentsolver:coinor_build": [
                        "-lClp",
                        "-lOsiClp",
                        "-lCoinUtils",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "linrelax_clp_test",
        srcs = ["linrelax_clp_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":linrelax_clp",
        ],
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "linrelax_gurobi",
        hdrs = ["linrelax_gurobi.hpp"],
        srcs = ["linrelax_gurobi.cpp"],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//generalizedassignmentsolver:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        visibility = ["//visibility:public"],
)

cc_test(
        name = "linrelax_gurobi_test",
        srcs = ["linrelax_gurobi_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":linrelax_gurobi",
        ],
        copts = select({"//generalizedassignmentsolver:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        linkopts = select({"//generalizedassignmentsolver:gurobi_build": [
                        "-lgurobi_c++",
                        "-lgurobi81",
                ], "//conditions:default": [ ],
        }),
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
        ],
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//generalizedassignmentsolver:coinor_build": [
                        "-lClp",
                        "-lOsiClp",
                        "-lCoinUtils",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "lagrelax_bundle_test",
        srcs = ["lagrelax_bundle_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":lagrelax_bundle",
        ],
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
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
        ],
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//generalizedassignmentsolver:coinor_build": [
                        "-lVol",
                        "-lCoinUtils",
                ],
                "//conditions:default": [
                ],
        }),
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
        ],
        copts = select({"//generalizedassignmentsolver:dlib_build": ["-DDLIB_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//generalizedassignmentsolver:dlib_build": [
                        "-ldlib",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "lagrelax_lbfgs_test",
        srcs = ["lagrelax_lbfgs_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":lagrelax_lbfgs",
        ],
        copts = select({"//generalizedassignmentsolver:dlib_build": ["-DDLIB_FOUND"], "//conditions:default": []}),
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
                ":random",
        ],
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
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
        ],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//generalizedassignmentsolver:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []})
          + select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//generalizedassignmentsolver:cplex_build": [
                        "-lconcert",
                        "-lilocplex",
                        "-lcplex",
                        "-lm",
                        "-lpthread",
                        "-ldl",
                ],
                "//conditions:default": [
                ],
        }) + select({"//generalizedassignmentsolver:coinor_build": [
                        "-lClp",
                        "-lOsiClp",
                        "-lCoinUtils",
                ], "//conditions:default": []}),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "columngeneration_test",
        srcs = ["columngeneration_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":columngeneration",
        ],
        copts = select({"//generalizedassignmentsolver:cplex_build":  ["-DCPLEX_FOUND"],  "//conditions:default": []})
              + select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
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
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "colgenheuristics_test",
        srcs = ["colgenheuristics_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":colgenheuristics",
        ],
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "localsolver",
        hdrs = ["localsolver.hpp"],
        srcs = ["localsolver.cpp"],
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        copts = select({"//generalizedassignmentsolver:localsolver_build": ["-DLOCALSOLVER_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//generalizedassignmentsolver:localsolver_build": [
                        "-llocalsolver",
                        "-lpthread",
                ],
                "//conditions:default": [
                ],
        }),
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
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//generalizedassignmentsolver:coinor_build": [
                        "-lCbcSolver",
                        "-lCbc",
                        "-lCgl",
                        "-lOsiCbc",
                        "-lCoinUtils",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "branchandcut_cbc_test",
        srcs = ["branchandcut_cbc_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":branchandcut_cbc",
        ],
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "branchandcut_cplex",
        hdrs = ["branchandcut_cplex.hpp"],
        srcs = ["branchandcut_cplex.cpp"],
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//generalizedassignmentsolver:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//generalizedassignmentsolver:cplex_build": [
                        "-lconcert",
                        "-lilocplex",
                        "-lcplex",
                        "-lm",
                        "-lpthread",
                        "-ldl",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "branchandcut_cplex_test",
        srcs = ["branchandcut_cplex_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":branchandcut_cplex",
        ],
        copts = select({"//generalizedassignmentsolver:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "branchandcut_gurobi",
        hdrs = ["branchandcut_gurobi.hpp"],
        srcs = ["branchandcut_gurobi.cpp"],
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//generalizedassignmentsolver:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "branchandcut_gurobi_test",
        srcs = ["branchandcut_gurobi_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":branchandcut_gurobi",
        ],
        copts = select({"//generalizedassignmentsolver:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        linkopts = select({"//generalizedassignmentsolver:gurobi_build": [
                        "-lgurobi_c++",
                        "-lgurobi81",
                ], "//conditions:default": [ ],
        }),
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
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "branchandprice_test",
        srcs = ["branchandprice_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":branchandprice",
        ],
        copts = select({"//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "constraintprogramming_cplex",
        hdrs = ["constraintprogramming_cplex.hpp"],
        srcs = ["constraintprogramming_cplex.cpp"],
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//generalizedassignmentsolver:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//generalizedassignmentsolver:cplex_build": [
                        "-lcp",
                        "-lconcert",
                        "-lilocplex",
                        "-lcplex",
                        "-lm",
                        "-lpthread",
                        "-ldl",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "constraintprogramming_cplex_test",
        srcs = ["constraintprogramming_cplex_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":constraintprogramming_cplex",
        ],
        copts = select({"//generalizedassignmentsolver:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

################################################################################

cc_library(
        name = "constraintprogramming_gecode",
        hdrs = ["constraintprogramming_gecode.hpp"],
        srcs = ["constraintprogramming_gecode.cpp"],
        deps = ["//generalizedassignmentsolver:generalizedassignmentsolver"],
        copts = select({"//generalizedassignmentsolver:gecode_build": ["-DGECODE_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//generalizedassignmentsolver:gecode_build": [
                        "-lgecodeflatzinc",
                        "-lgecodedriver",
                        "-lgecodegist",
                        "-lgecodesearch",
                        "-lgecodeminimodel",
                        "-lgecodeset",
                        "-lgecodefloat",
                        "-lgecodeint",
                        "-lgecodekernel",
                        "-lgecodesupport",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
)

cc_test(
        name = "constraintprogramming_gecode_test",
        srcs = ["constraintprogramming_gecode_test.cpp"],
        deps = [
                "//generalizedassignmentsolver:tester",
                ":constraintprogramming_gecode",
        ],
        copts = select({"//generalizedassignmentsolver:gecode_build": ["-DGECODE_FOUND"], "//conditions:default": []}),
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
        copts = [] +
            select({"//generalizedassignmentsolver:dlib_build":        ["-DDLIB_FOUND"],        "//conditions:default": []}) +
            select({"//generalizedassignmentsolver:coinor_build":      ["-DCOINOR_FOUND"],      "//conditions:default": []}) +
            select({"//generalizedassignmentsolver:cplex_build":       ["-DCPLEX_FOUND"],       "//conditions:default": []}) +
            select({"//generalizedassignmentsolver:gurobi_build":      ["-DGUROBI_FOUND"],      "//conditions:default": []}) +
            select({"//generalizedassignmentsolver:gecode_build":      ["-DGECODE_FOUND"],      "//conditions:default": []}) +
            select({"//generalizedassignmentsolver:localsolver_build": ["-DLOCALSOLVER_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
)

