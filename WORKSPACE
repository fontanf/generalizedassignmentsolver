load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

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
    commit = "abb80bd06abebe90639c3680f203854ebdaf83ec",
)

git_repository(
    name = "benchtools",
    remote = "https://github.com/fontanf/benchtools.git",
    commit = "05883deff50e329875ea1bd8ddd5f6076da9d7d4",
)

git_repository(
    name = "ortools",
    remote = "https://github.com/google/or-tools",
    commit = "4a0e9b1860276a021335aacb8b69e10a0d08942c",
)

# OR-Tools

git_repository(
    name = "com_github_gflags_gflags",
    commit = "e171aa2",  # release v2.2.2
    remote = "https://github.com/gflags/gflags.git",
)

git_repository(
    name = "com_github_glog_glog",
    commit = "96a2f23",  # release v0.4.0
    remote = "https://github.com/google/glog.git",
)

git_repository(
    name = "bazel_skylib",
    commit = "3721d32",  # release 0.8.0
    remote = "https://github.com/bazelbuild/bazel-skylib.git",
)

git_repository(
    name = "com_google_protobuf",
    commit = "0974557",  # release v3.8.0
    remote = "https://github.com/protocolbuffers/protobuf.git",
)

git_repository(
    name = "com_google_protobuf_cc",
    commit = "0974557",  # release v3.8.0
    remote = "https://github.com/protocolbuffers/protobuf.git",
)

load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")
# Load common dependencies.
protobuf_deps()

git_repository(
    name = "com_google_absl",
    commit = "bf29470",
    remote = "https://github.com/abseil/abseil-cpp.git",
)

http_archive(
    name = "gtest",
    build_file = "//bazel:gtest.BUILD",
    strip_prefix = "googletest-release-1.8.0/googletest",
    url = "https://github.com/google/googletest/archive/release-1.8.0.zip",
)

http_archive(
    name = "glpk",
    build_file = "//bazel:glpk.BUILD",
    sha256 = "9a5dab356268b4f177c33e00ddf8164496dc2434e83bd1114147024df983a3bb",
    url = "http://ftp.gnu.org/gnu/glpk/glpk-4.52.tar.gz",
)

