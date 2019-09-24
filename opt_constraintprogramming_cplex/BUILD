cc_library(
        name = "constraintprogramming_cplex",
        hdrs = ["constraintprogramming_cplex.hpp"],
        srcs = ["constraintprogramming_cplex.cpp"],
        deps = ["//lib:cpp"],
        copts = [
                "-m64",
                "-DIL_STD",
        ] + select({"//lib:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:cplex_build": [
                        "-lcp",
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
        include_prefix = "gap/opt_constraintprogramming_cplex/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//opt_constraintprogramming_cplex:constraintprogramming_cplex",
        ],
        copts = select({"//lib:cplex_build": ["-DCPLEX_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

