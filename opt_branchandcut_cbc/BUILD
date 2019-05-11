cc_library(
        name = "branchandcut_cbc",
        hdrs = ["branchandcut_cbc.hpp"],
        srcs = ["branchandcut_cbc.cpp"],
        deps = ["//lib:cpp"],
        linkopts = [
                "-lCbcSolver",
                "-lCbc",
                "-lCgl",
                "-lOsiCbc",
                "-lCoinUtils",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/opt_branchandcut_cbc/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//opt_branchandcut_cbc:branchandcut_cbc",
        ],
        timeout = "moderate",
)

