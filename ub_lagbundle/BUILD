cc_library(
        name = "lagbundle",
        hdrs = ["lagbundle.hpp"],
        srcs = ["lagbundle.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:minknap",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/ub_lagbundle/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":lagbundle",
        ],
        data = [
                "//data_tests:data",
        ],
        timeout = "moderate",
)

