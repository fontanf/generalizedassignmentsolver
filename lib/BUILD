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
                "@googletest//:gtest_main",
        ],
        linkopts = [
                "-lboost_system",
                "-lboost_filesystem",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lib/",
)

