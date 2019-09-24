cc_library(
        name = "branchandprice_clp",
        hdrs = ["branchandprice_clp.hpp"],
        srcs = ["branchandprice_clp.cpp"],
        deps = [
                "//lb_colgen_clp:colgen_clp",
                "//ub_greedy:greedy",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "gap/opt_branchandprice_clp/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":branchandprice_clp",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

