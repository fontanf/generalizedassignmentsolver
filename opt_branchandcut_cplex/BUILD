cc_library(
        name = "branchandcut_cplex",
        hdrs = ["branchandcut_cplex.hpp"],
        srcs = ["branchandcut_cplex.cpp"],
        deps = ["//lib:cpp"],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//lib:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:cplex_build": [
                        "-lconcert",
                        "-lilocplex",
                        "-lcplex",
                        "-lm",
                        "-lpthread",
                        "-ldl",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
        include_prefix = "generalizedassignment/opt_branchandcut_cplex/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":branchandcut_cplex",
        ],
        copts = select({"//lib:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

