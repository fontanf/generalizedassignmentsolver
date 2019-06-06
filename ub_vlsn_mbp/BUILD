cc_library(
        name = "vlsn_mbp",
        hdrs = ["vlsn_mbp.hpp"],
        srcs = ["vlsn_mbp.cpp"],
        deps = [
                "//lib:cpp",
                "//lb_lagrelax_volume:lagrelax_volume",
                "//ub_random:random",
                "//ub_repair:repair",
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

