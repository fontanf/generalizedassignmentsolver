load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

new_git_repository(
    name = "json",
    build_file_content = """
cc_library(
        name = "json",
        hdrs = ["single_include/nlohmann/json.hpp"],
        visibility = ["//visibility:public"],
        strip_include_prefix = "single_include/"
)
""",
    remote = "https://github.com/nlohmann/json",
    tag = "v3.7.0",
)

git_repository(
    name = "benchtools",
    remote = "https://github.com/fontanf/benchtools.git",
    commit = "0257329b5401096c1cdee352f2e1d8f3b2335af2",
)

local_repository(
    name = "benchtools_",
    path = "/home/florian/Dev/benchtools/",
)

new_git_repository(
    name = "googletest",
    build_file_content = """
cc_library(
        name = "gtest",
        srcs = ["googletest/src/gtest-all.cc", "googlemock/src/gmock-all.cc",],
        hdrs = glob(["**/*.h", "googletest/src/*.cc", "googlemock/src/*.cc",]),
        includes = ["googlemock", "googletest", "googletest/include", "googlemock/include",],
        linkopts = ["-pthread"],
        visibility = ["//visibility:public"],
)

cc_library(
        name = "gtest_main",
        srcs = ["googlemock/src/gmock_main.cc"],
        linkopts = ["-pthread"],
        visibility = ["//visibility:public"],
        deps = [":gtest"],
)
""",
    remote = "https://github.com/google/googletest",
    tag = "release-1.8.0",
)

git_repository(
    name = "knapsack",
    remote = "https://github.com/fontanf/knapsack.git",
    commit = "98da719284d0b56a48f36ada24e91cf6671beb94",
)

