cc_library(
        name = "constraintprogramming_cplex",
        hdrs = ["constraintprogramming_cplex.hpp"],
        srcs = ["constraintprogramming_cplex.cpp"],
        deps = ["//lib:cpp"],
        copts = [
                "-m64",
                "-DIL_STD",
        ],
        linkopts = [
                "-lcp",
                "-lconcert",
                "-lilocplex",
                "-lcplex",
                "-lm",
                "-lpthread",
        ],
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
        timeout = "moderate",
)

