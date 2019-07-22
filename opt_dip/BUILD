cc_library(
        name = "dip",
        hdrs = ["dip.hpp"],
        srcs = ["dip.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:coinor_build": [
                        "-lCbcSolver",
                        "-lCbc",
                        "-lCgl",
                        "-lOsiCbc",
                        "-lCoinUtils",
                        "-lAlps",
                        "-lDecomp",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
        include_prefix = "gap/opt_dip/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//opt_dip:dip",
        ],
        timeout = "moderate",
)

