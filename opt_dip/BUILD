cc_library(
        name = "dip",
        hdrs = ["dip.hpp"],
        srcs = ["dip.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
        ],
        linkopts = [
                "-lDecomp",
        ],
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

