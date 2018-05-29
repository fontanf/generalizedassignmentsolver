cc_library(
        name = "libbabposta",
        hdrs = ["babposta.hpp"],
        srcs = ["babposta.cpp"],
        deps = [
                "//lib:cpp",
                "//ub_lagrangian:liblagrangian"
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/opt_babposta/",
)

cc_binary(
        name = "main",
        srcs = ["main.cpp"],
        deps = [":libbabposta"],
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = ["//lib:tester"],
        data = [
                ":main",
                "//data_tests:data",
        ],
        timeout = "short",
)

