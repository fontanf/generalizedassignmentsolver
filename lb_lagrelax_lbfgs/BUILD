cc_library(
        name = "lagrelax_lbfgs",
        hdrs = ["lagrelax_lbfgs.hpp"],
        srcs = ["lagrelax_lbfgs.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
        ],
        copts = select({"//lib:dlib_build": ["-DDLIB_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:dlib_build": [
                        "-ldlib",
                ],
                "//conditions:default": [
                ],
        }),
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
        copts = select({"//lib:dlib_build": ["-DDLIB_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

