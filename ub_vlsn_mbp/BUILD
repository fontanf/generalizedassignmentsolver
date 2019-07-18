cc_library(
        name = "vlsn_mbp",
        hdrs = ["vlsn_mbp.hpp"],
        srcs = ["vlsn_mbp.cpp"],
        deps = [
                "//lib:cpp",
                "//lb_lagrelax_lbfgs:lagrelax_lbfgs",
                "//ub_random:random",
                "//ub_repair:repair",
                "//ub_ls_shiftswap:ls_shiftswap",
        ],
        copts = [
                "-m64",
                "-DIL_STD",
        ],
        linkopts = [
                "-lconcert",
                "-lilocplex",
                "-lcplex",
                "-lm",
                "-lpthread",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/ub_vlsn_mbp/",
)

