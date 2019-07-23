cc_library(
        name = "vnsbranching_cplex",
        hdrs = ["vnsbranching_cplex.hpp"],
        srcs = ["vnsbranching_cplex.cpp"],
        deps = [
                "//lib:cpp",
                "//ub_random:random",
                "//ub_ls_shiftswap:ls_shiftswap",
        ],
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
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
        include_prefix = "gap/ub_vnsbranching_cplex/",
)

