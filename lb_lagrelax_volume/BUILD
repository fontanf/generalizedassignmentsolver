cc_library(
        name = "lagrelax_volume",
        hdrs = ["lagrelax_volume.hpp"],
        srcs = ["lagrelax_volume.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:cplex_build": [
                        "-lVol",
                        "-lCoinUtils",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
        include_prefix = "gap/lb_lagrelax_volume/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//lb_lagrelax_volume:lagrelax_volume",
        ],
        timeout = "moderate",
)

