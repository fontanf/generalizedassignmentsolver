cc_library(
        name = "lagrelax_lbfgs",
        hdrs = ["lagrelax_lbfgs.hpp"],
        srcs = ["lagrelax_lbfgs.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
        ],
        linkopts = [
                "-ldlib",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lb_lagrelax_lbfgs/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":lagrelax_lbfgs",
        ],
        timeout = "moderate",
)

