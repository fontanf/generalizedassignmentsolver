cc_library(
        name = "colgen_clp",
        hdrs = ["colgen_clp.hpp"],
        srcs = ["colgen_clp.cpp"],
        deps = [
                "//lib:cpp",
                "//ub_ls_shiftswap:ls_shiftswap",
                "@knapsack//opt_minknap:minknap",
                "@knapsack//opt_bellman:bellman",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = [] +
            select({"//lib:coinor_build": [
                        "-lClp",
                        "-lOsiClp",
                        "-lCoinUtils",
                ], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "gap/lb_colgen_clp/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":colgen_clp",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

