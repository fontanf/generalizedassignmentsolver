cc_library(
        name = "vnsbranching_cplex",
        hdrs = ["vnsbranching_cplex.hpp"],
        srcs = ["vnsbranching_cplex.cpp"],
        deps = [
                "//lib:cpp",
                "//ub_random:random",
                "//ub_repair:repair",
        ],
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
        include_prefix = "gap/ub_vnsbranching_cplex/",
)

