load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")
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
    commit = "f6399dc14d063d728019ed3be1593c7d2a6f50f8",
)

git_repository(
    name = "benchtools",
    remote = "https://github.com/fontanf/benchtools.git",
    commit = "8a78dd897d84118bb2bb1bdc4c8ae66194ca2e2e",
)

