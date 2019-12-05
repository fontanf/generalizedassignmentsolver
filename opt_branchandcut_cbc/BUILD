cc_library(
        name = "branchandcut_cbc",
        hdrs = ["branchandcut_cbc.hpp"],
        srcs = ["branchandcut_cbc.cpp"],
        deps = ["//lib:cpp"],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:coinor_build": [
                        "-lCbcSolver",
                        "-lCbc",
                        "-lCgl",
                        "-lOsiCbc",
                        "-lCoinUtils",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
        include_prefix = "generalizedassignment/opt_branchandcut_cbc/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//opt_branchandcut_cbc:branchandcut_cbc",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

