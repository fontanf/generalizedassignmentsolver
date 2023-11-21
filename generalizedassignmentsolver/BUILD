load("@bazel_skylib//lib:selects.bzl", "selects")

config_setting(
        name = "clp_build",
        values = {"define": "clp=true"},
        visibility = ["//visibility:public"],
)

selects.config_setting_group(
    name = "clp_linux",
    match_all = [":clp_build", "@bazel_tools//src/conditions:linux"],
)

selects.config_setting_group(
    name = "clp_darwin",
    match_all = [":clp_build", "@bazel_tools//src/conditions:darwin"],
)

selects.config_setting_group(
    name = "clp_windows",
    match_all = [":clp_build", "@bazel_tools//src/conditions:windows"],
)

config_setting(
        name = "cbc_build",
        values = {"define": "cbc=true"},
        visibility = ["//visibility:public"],
)

selects.config_setting_group(
    name = "cbc_linux",
    match_all = [":cbc_build", "@bazel_tools//src/conditions:linux"],
)

selects.config_setting_group(
    name = "cbc_darwin",
    match_all = [":cbc_build", "@bazel_tools//src/conditions:darwin"],
)

selects.config_setting_group(
    name = "cbc_windows",
    match_all = [":cbc_build", "@bazel_tools//src/conditions:windows"],
)

config_setting(
        name = "volume_build",
        values = {"define": "volume=true"},
        visibility = ["//visibility:public"],
)

selects.config_setting_group(
    name = "volume_linux",
    match_all = [":volume_build", "@bazel_tools//src/conditions:linux"],
)

selects.config_setting_group(
    name = "volume_darwin",
    match_all = [":volume_build", "@bazel_tools//src/conditions:darwin"],
)

selects.config_setting_group(
    name = "volume_windows",
    match_all = [":volume_build", "@bazel_tools//src/conditions:windows"],
)

config_setting(
    name = "cplex_build",
    values = {"define": "cplex=true"},
    visibility = ["//visibility:public"],
)

config_setting(
    name = "gurobi_build",
    values = {"define": "gurobi=true"},
    visibility = ["//visibility:public"],
)

config_setting(
    name = "localsolver_build",
    values = {"define": "localsolver=true"},
    visibility = ["//visibility:public"],
)

config_setting(
    name = "knitro_build",
    values = {"define": "knitro=true"},
    visibility = ["//visibility:public"],
)

config_setting(
    name = "gecode_build",
    values = {"define": "gecode=true"},
    visibility = ["//visibility:public"],
)

load("//generalizedassignmentsolver:variables.bzl", "STDCPP")

cc_library(
        name = "generalizedassignmentsolver",
        hdrs = [
                "instance.hpp",
                "instance_builder.hpp",
                "solution.hpp",
                "desirability.hpp",
        ],
        srcs = [
                "instance.cpp",
                "instance_builder.cpp",
                "solution.cpp",
                "desirability.cpp",
        ],
        copts = STDCPP,
        deps = [
                "@optimizationtools//optimizationtools/utils:info",
                "@optimizationtools//optimizationtools/utils:utils",
        ],
        visibility = ["//visibility:public"],
)

cc_binary(
        name = "main",
        srcs = ["main.cpp"],
        deps = [
                "//generalizedassignmentsolver/algorithms:algorithms",
        ],
        data = ["//data:data"],
)

cc_binary(
        name = "checker",
        srcs = ["checker.cpp"],
        deps = [":generalizedassignmentsolver"],
        data = ["//data:data"],
)

