cc_library(
        name = "branchandcut_gurobi",
        hdrs = ["branchandcut_gurobi.hpp"],
        srcs = ["branchandcut_gurobi.cpp"],
        deps = ["//lib:cpp"],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//lib:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "generalizedassignment/opt_branchandcut_gurobi/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//opt_branchandcut_gurobi:branchandcut_gurobi",
        ],
        copts = select({"//lib:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        linkopts = select({"//lib:gurobi_build": [
                        "-lgurobi_c++",
                        "-lgurobi81",
                ], "//conditions:default": [ ],
        }),
        timeout = "moderate",
)

