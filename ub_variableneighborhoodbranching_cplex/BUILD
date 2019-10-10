cc_library(
        name = "variableneighborhoodbranching_cplex",
        hdrs = ["variableneighborhoodbranching_cplex.hpp"],
        srcs = ["variableneighborhoodbranching_cplex.cpp"],
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
                        "-ldl",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
        include_prefix = "gap/ub_variableneighborhoodbranching_cplex/",
)

