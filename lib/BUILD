cc_library(
        name = "cpp",
        hdrs = [
                "instance.hpp",
                "solution.hpp",
                "subinstance.hpp",
        ],
        srcs = [
                "instance.cpp",
                "solution.cpp",
                "subinstance.cpp",
        ],
        deps = ["@benchtools//:info"],
        linkopts = [
                "-lboost_system",
                "-lboost_filesystem",
                "-lboost_program_options",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lib/",
)

cc_library(
        name = "hpp",
        hdrs = [
                "instance.hpp",
                "solution.hpp",
                "subinstance.hpp",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lib/",
)

cc_library(
        name = "tester",
        hdrs = ["tester.hpp"],
        srcs = ["tester.cpp"],
        deps = [
                ":cpp",
                ":libgenerator",
                "@googletest//:gtest_main",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lib/",
)

cc_library(
        name = "libgenerator",
        hdrs = ["generator.hpp"],
        srcs = ["generator.cpp"],
        deps = ["//lib:cpp"],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lib/",
)

cc_binary(
        name = "main",
        srcs = ["main.cpp"],
        deps = [
                "//opt_milp:milp",
                "//ub_random:random",
                "//ub_lssimple:lssimple",
        ],
)

cc_binary(
        name = "test_main",
        srcs = ["test_main.cpp"],
        deps = [
                "//opt_milp:milp",
                ":tester",
        ],
)

