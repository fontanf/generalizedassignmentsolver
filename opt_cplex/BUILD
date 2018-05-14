cc_library(
        name = "libcplex",
        hdrs = ["cplex.hpp"],
        srcs = ["cplex.cpp"],
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
        include_prefix = "gap/opt_cplex/",
)

cc_binary(
        name = "main",
        srcs = ["main.cpp"],
        deps = [":libcplex"],
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = ["//lib:tester"],
        data = [
                ":main",
                "//data_tests:data",
        ],
        timeout = "short",
)

