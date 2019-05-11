cc_library(
        name = "branchandcut_cplex",
        hdrs = ["branchandcut_cplex.hpp"],
        srcs = ["branchandcut_cplex.cpp"],
        deps = ["//lib:cpp"],
        copts = [
                "-m64",
                "-DIL_STD",
        ],
        linkopts = [
                "-L/opt/ibm/ILOG/CPLEX_Studio126/cplex/lib/x86-64_linux/static_pic",
                "-L/opt/ibm/ILOG/CPLEX_Studio126/concert/lib/x86-64_linux/static_pic/",
                "-lconcert",
                "-lilocplex",
                "-lcplex",
                "-lm",
                "-lpthread",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/opt_branchandcut_cplex/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//opt_branchandcut_cplex:branchandcut_cplex",
        ],
        timeout = "moderate",
)

