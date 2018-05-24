cc_library(
        name = "liblagrangian",
        hdrs = ["lagrangian.hpp"],
        srcs = ["lagrangian.cpp"],
        deps = [
                "//lib:cpp",
                "@knapsack//opt_minknap:libminknap",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/ub_lagrangian/",
)

cc_binary(
        name = "main",
        srcs = ["main.cpp"],
        deps = [
                ":liblagrangian",
        ],
        linkopts = [
                "-lboost_system",
                "-lboost_filesystem",
                "-lboost_program_options"
        ],
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":liblagrangian",
                "//opt_cplex:libcplex",
        ],
        data = [
                ":main",
                "//data_tests:data",
        ],
        timeout = "moderate",
)
