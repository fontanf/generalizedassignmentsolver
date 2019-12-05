cc_library(
        name = "branchandprice",
        hdrs = ["branchandprice.hpp"],
        srcs = ["branchandprice.cpp"],
        deps = [
                "//lb_columngeneration:columngeneration",
                "//ub_greedy:greedy",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "generalizedassignment/opt_branchandprice/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                ":branchandprice",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        timeout = "moderate",
)

