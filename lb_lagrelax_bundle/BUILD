cc_library(
        name = "lagrelax_bundle",
        hdrs = ["lagrelax_bundle.hpp"],
        srcs = ["lagrelax_bundle.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
        ],
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

