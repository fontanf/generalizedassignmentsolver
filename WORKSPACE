load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

git_repository(
    name = "googletest",
    remote = "https://github.com/google/googletest.git",
    commit = "703bd9caab50b139428cea1aaff9974ebee5742e",
    shallow_since = "1570114335 -0400",
)

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "9f9fb8b2f0213989247c9d5c0e814a8451d18d7f",
    remote = "https://github.com/nelhage/rules_boost",
    shallow_since = "1570056263 -0700",
)
load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

http_archive(
    name = "json",
    build_file_content = """
cc_library(
        name = "json",
        hdrs = ["single_include/nlohmann/json.hpp"],
        visibility = ["//visibility:public"],
        strip_include_prefix = "single_include/"
)
""",
    urls = ["https://github.com/nlohmann/json/releases/download/v3.7.3/include.zip"],
    sha256 = "87b5884741427220d3a33df1363ae0e8b898099fbc59f1c451113f6732891014",
)

http_archive(
    name = "dlib",
    build_file_content = """
cc_library(
        name = "dlib",
        hdrs = ["dlib-19.19/dlib/all/source.cpp"],
        visibility = ["//visibility:public"],
        strip_include_prefix = "dlib-19.19/",
)
""",
    urls = ["http://dlib.net/files/dlib-19.19.tar.bz2"],
    sha256 = "1decfe883635ce51acd72869cebe870ab9b85eb094d417adc8f48aa7b8c60cd7",
)

git_repository(
    name = "benchtools",
    remote = "https://github.com/fontanf/benchtools.git",
    commit = "fe56ed683d32f70cd248d77cd4107e57eee05758",
    shallow_since = "1576623294 +0100",
)

git_repository(
    name = "knapsacksolver",
    remote = "https://github.com/fontanf/knapsacksolver.git",
    commit = "29329dad9584ba92aa0093d3488b1a0014e806e5",
    shallow_since = "1584187590 +0100"
)

