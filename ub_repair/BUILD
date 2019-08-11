cc_library(
        name = "repair",
        hdrs = ["repair.hpp"],
        srcs = ["repair.cpp"],
        deps = [
                "//lib:cpp",
                "//lb_linrelax_clp:linrelax_clp",
                "//lb_linrelax_gurobi:linrelax_gurobi",
                "//ub_random:random",
        ],
        copts = [] +
            select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}) +
            select({"//lib:gurobi_build": ["-DGUROBI_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "gap/ub_repair/",
)

