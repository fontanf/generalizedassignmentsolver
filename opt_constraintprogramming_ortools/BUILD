cc_library(
        name = "constraintprogramming_ortools",
        hdrs = ["constraintprogramming_ortools.hpp"],
        srcs = ["constraintprogramming_ortools.cpp"],
        copts = select({"//lib:ortools_build": ["-DORTOOLS_FOUND"], "//conditions:default": []}),
        deps = select({
                "//lib:ortools_build": [
                        "//lib:cpp",
                        "@ortools//ortools/base",
                        "@ortools//ortools/base:file",
                        "@ortools//ortools/base:timer",
                        "@ortools//ortools/sat:cp_model",
                        "@ortools//ortools/sat:cp_model_solver",
                        "@ortools//ortools/sat:disjunctive",
                        "@ortools//ortools/sat:integer",
                        "@ortools//ortools/sat:intervals",
                        "@ortools//ortools/sat:model",
                        "@ortools//ortools/sat:optimization",
                        "@ortools//ortools/sat:precedences",
                        "@ortools//ortools/sat:sat_solver",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
        include_prefix = "gap/opt_constraintprogramming_ortools/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//opt_constraintprogramming_ortools:constraintprogramming_ortools",
        ],
        timeout = "moderate",
)

