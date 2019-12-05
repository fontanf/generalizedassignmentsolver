cc_library(
        name = "repair",
        hdrs = ["repair.hpp"],
        srcs = ["repair.cpp"],
        deps = [
                "//lib:cpp",
                "//lb_linrelax_clp:linrelax_clp",
                "//ub_random:random",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "generalizedassignment/ub_repair/",
)

