cc_library(
        name = "colgen",
        hdrs = ["colgen.hpp"],
        srcs = ["colgen.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
                "@knapsack//opt_bellman:bellman",
        ],
        copts = [
                "-m64",
                "-DIL_STD"
        ] + select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}) +
            select({"//lib:cplex_build":  ["-DCPLEX_FOUND"],  "//conditions:default": []}) +
            select({"//lib:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        linkopts = [] +
            select({"//lib:cplex_build": [
                        "-lconcert",
                        "-lilocplex",
                        "-lcplex",
                        "-lm",
                        "-lpthread",
                ], "//conditions:default": []}) +
            select({"//lib:coinor_build": [
                        "-lCbcSolver",
                        "-lCbc",
                        "-lCgl",
                        "-lOsiCbc",
                        "-lCoinUtils",
                ], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "gap/lb_colgen/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":colgen",
        ],
        copts = [
        ] + select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}) +
            select({"//lib:cplex_build":  ["-DCPLEX_FOUND"],  "//conditions:default": []}) +
            select({"//lib:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        linkopts = select({"//lib:gurobi_build": [
                        "-lgurobi_c++",
                        "-lgurobi81",
                ], "//conditions:default": [ ],
        }),
        timeout = "moderate",
)

