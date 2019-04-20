cc_library(
        name = "milp",
        hdrs = ["milp.hpp"],
        srcs = ["milp.cpp"],
        deps = ["//lib:cpp"],
        linkopts = [
                "-lCbcSolver",
                "-lCbc",
                "-lOsiCbc",
                "-lCoinUtils",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/opt_milp/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//opt_milp:milp",
        ],
        timeout = "moderate",
)

