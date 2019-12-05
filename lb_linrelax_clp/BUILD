cc_library(
        name = "linrelax_clp",
        hdrs = ["linrelax_clp.hpp"],
        srcs = ["linrelax_clp.cpp"],
        deps = [
                "//lib:cpp",
                "//opt_branchandcut_cbc:branchandcut_cbc",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:coinor_build": [
                        "-lClp",
                        "-lOsiClp",
                        "-lCoinUtils",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
        include_prefix = "generalizedassignment/lb_linrelax_clp/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//lb_linrelax_clp:linrelax_clp",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

