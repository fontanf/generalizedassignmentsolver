cc_library(
        name = "colgen_cplex",
        hdrs = ["colgen_cplex.hpp"],
        srcs = ["colgen_cplex.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
                "@knapsack//opt_bellman:bellman",
        ],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//lib:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:cplex_build": [
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
        include_prefix = "gap/lb_colgen_cplex/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":colgen_cplex",
        ],
        copts = select({"//lib:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

