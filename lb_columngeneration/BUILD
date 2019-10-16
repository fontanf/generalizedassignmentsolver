cc_library(
        name = "columngeneration",
        hdrs = ["columngeneration.hpp"],
        srcs = ["columngeneration.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
                "@knapsack//opt_bellman:bellman",
        ],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//lib:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []})
          + select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
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
        }) + select({"//lib:coinor_build": [
                        "-lClp",
                        "-lOsiClp",
                        "-lCoinUtils",
                ], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "gap/lb_columngeneration/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":columngeneration",
        ],
        copts = select({"//lib:cplex_build":  ["-DCPLEX_FOUND"],  "//conditions:default": []})
              + select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

