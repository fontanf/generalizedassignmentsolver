cc_library(
        name = "linrelax_gurobi",
        hdrs = ["linrelax_gurobi.hpp"],
        srcs = ["linrelax_gurobi.cpp"],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//lib:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        deps = ["//lib:cpp"],
        visibility = ["//visibility:public"],
        include_prefix = "generalizedassignment/lb_linrelax_gurobi/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//lb_linrelax_gurobi:linrelax_gurobi",
        ],
        copts = select({"//lib:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        linkopts = select({"//lib:gurobi_build": [
                        "-lgurobi_c++",
                        "-lgurobi81",
                ], "//conditions:default": [ ],
        }),
        timeout = "moderate",
)

