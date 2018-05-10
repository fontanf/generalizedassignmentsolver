cc_library(
        name = "cpp",
        hdrs = [
                "instance.hpp",
                "solution.hpp",
        ],
        srcs = ["instance.cpp", "solution.cpp"],
        visibility = ["//visibility:public"],
        deps = [],
)

cc_library(
        name = "hpp",
        hdrs = ["instance.hpp", "solution.hpp"],
        visibility = ["//visibility:public"],
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                ":cpp",
                "//lb_greedy:libgreedy",
                "@googletest//:gtest_main",
        ],
        data = [
                "@KnapsackPisingerInstances//:data",
        ],
        linkopts = [
                "-lboost_system",
                "-lboost_filesystem",
                "-lboost_program_options",
                "-lboost_regex",
        ],
        timeout = "long",
)

cc_library(
        name = "tester",
        hdrs = ["tester.hpp"],
        srcs = ["tester.cpp"],
        visibility = ["//visibility:public"],
        linkopts = [
                "-lboost_system",
                "-lboost_filesystem",
        ],
        deps = [
                ":cpp", 
                "@googletest//:gtest_main",
        ],
)

