cc_library(
        name = "colgenheuristics",
        hdrs = ["colgenheuristics.hpp"],
        srcs = ["colgenheuristics.cpp"],
        deps = [
                "//lb_columngeneration:columngeneration",
                "//ub_greedy:greedy",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "gap/ub_colgenheuristics/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":colgenheuristics",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

