config_setting(name = "coinor_build",      values = {"define": "coinor=true"},      visibility = ["//visibility:public"])
config_setting(name = "cplex_build",       values = {"define": "cplex=true"},       visibility = ["//visibility:public"])
config_setting(name = "gurobi_build",      values = {"define": "gurobi=true"},      visibility = ["//visibility:public"])
config_setting(name = "localsolver_build", values = {"define": "localsolver=true"}, visibility = ["//visibility:public"])
config_setting(name = "gecode_build",      values = {"define": "gecode=true"},      visibility = ["//visibility:public"])

load("//generalizedassignmentsolver:variables.bzl", "STDCPP")

cc_library(
        name = "generalizedassignmentsolver",
        hdrs = [
                "instance.hpp",
                "solution.hpp",
                "desirability.hpp",
        ],
        srcs = [
                "instance.cpp",
                "solution.cpp",
                "desirability.cpp",
        ],
        copts = STDCPP,
        deps = ["@optimizationtools//optimizationtools:info"],
        visibility = ["//visibility:public"],
)

cc_library(
        name = "hpp",
        hdrs = [
                "instance.hpp",
                "solution.hpp",
        ],
        visibility = ["//visibility:public"],
)

cc_library(
        name = "tester",
        hdrs = ["tester.hpp"],
        srcs = ["tester.cpp"],
        deps = [
                ":generalizedassignmentsolver",
                ":generator",
                "@googletest//:gtest_main",
        ],
        visibility = ["//visibility:public"],
)

cc_library(
        name = "generator",
        hdrs = ["generator.hpp"],
        srcs = ["generator.cpp"],
        deps = [":generalizedassignmentsolver"],
        visibility = ["//visibility:public"],
)

cc_binary(
        name = "main",
        srcs = ["main.cpp"],
        deps = [
                "//generalizedassignmentsolver/algorithms:algorithms",
                "@boost//:program_options",
        ],
        data = ["//data:data"],
)

cc_binary(
        name = "checker",
        srcs = ["checker.cpp"],
        deps = [":generalizedassignmentsolver"],
        data = ["//data:data"],
)

cc_binary(
        name = "test_main",
        srcs = ["test_main.cpp"],
        deps = [
                "//generalizedassignmentsolver/algorithms:algorithms",
                ":tester",
        ],
)

cc_binary(
        name = "generator_main",
        srcs = ["generator_main.cpp"],
        deps = [
                ":generator",
                "@boost//:program_options",
        ],
)

