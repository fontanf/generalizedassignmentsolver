cc_library(
        name = "lagrelax_bundle",
        hdrs = ["lagrelax_bundle.hpp"],
        srcs = ["lagrelax_bundle.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:coinor_build": [
                        "-lClp",
                        "-lOsiClp",
                        "-lCoinUtils",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
        include_prefix = "gap/lb_lagrelax_bundle/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":lagrelax_bundle",
        ],
        timeout = "moderate",
)

