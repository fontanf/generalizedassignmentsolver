cc_library(
        name = "dip",
        hdrs = ["dip.hpp"],
        srcs = ["dip.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
        ],
        linkopts = [
                "-lVol",
                "-lCoinUtils",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lb_dip/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//lb_dip:dip",
        ],
        timeout = "moderate",
)

