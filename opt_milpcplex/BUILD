cc_library(
        name = "milpcplex",
        hdrs = ["milpcplex.hpp"],
        srcs = ["milpcplex.cpp"],
        deps = ["//lib:cpp"],
        copts = [
                "-m64",
                "-O",
                "-fPIC",
                "-fno-strict-aliasing",
                "-fexceptions",
                "-DNDEBUG",
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
        include_prefix = "gap/opt_milpcplex/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//opt_milpcplex:milpcplex",
        ],
        timeout = "moderate",
)

