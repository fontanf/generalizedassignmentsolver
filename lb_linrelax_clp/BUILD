cc_library(
        name = "linrelax_clp",
        hdrs = ["linrelax_clp.hpp"],
        srcs = ["linrelax_clp.cpp"],
        deps = [
                "//lib:cpp",
                "//opt_branchandcut_cbc:branchandcut_cbc",
        ],
        linkopts = [
                "-lClp",
                "-lOsiClp",
                "-lCoinUtils",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lb_linrelax_clp/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//lb_linrelax_clp:linrelax_clp",
        ],
        timeout = "moderate",
)

