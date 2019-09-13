config_setting(name = "coinor_build",      values = {"define": "coinor=true"},      visibility = ["//visibility:public"])
config_setting(name = "dlib_build",        values = {"define": "dlib=true"},        visibility = ["//visibility:public"])
config_setting(name = "cplex_build",       values = {"define": "cplex=true"},       visibility = ["//visibility:public"])
config_setting(name = "gurobi_build",      values = {"define": "gurobi=true"},      visibility = ["//visibility:public"])
config_setting(name = "localsolver_build", values = {"define": "localsolver=true"}, visibility = ["//visibility:public"])
config_setting(name = "gecode_build",      values = {"define": "gecode=true"},      visibility = ["//visibility:public"])

cc_library(
        name = "cpp",
        hdrs = [
                "instance.hpp",
                "solution.hpp",
                "desirability.hpp",
                "datasets.hpp",
        ],
        srcs = [
                "instance.cpp",
                "solution.cpp",
                "desirability.cpp",
        ],
        deps = ["@benchtools//:info"],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lib/",
)

cc_library(
        name = "hpp",
        hdrs = [
                "instance.hpp",
                "solution.hpp",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lib/",
)

cc_library(
        name = "tester",
        hdrs = ["tester.hpp"],
        srcs = ["tester.cpp"],
        deps = [
                ":cpp",
                ":generator",
                "@googletest//:gtest_main",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lib/",
)

cc_library(
        name = "generator",
        hdrs = ["generator.hpp"],
        srcs = ["generator.cpp"],
        deps = ["//lib:cpp"],
        visibility = ["//visibility:public"],
        include_prefix = "gap/lib/",
)

cc_library(
        name = "algorithms",
        hdrs = ["algorithms.hpp"],
        srcs = ["algorithms.cpp"],
        deps = [
                "//lb_linrelax_clp:linrelax_clp",
                "//lb_linrelax_gurobi:linrelax_gurobi",
                "//lb_lagrelax_volume:lagrelax_volume",
                "//lb_lagrelax_bundle:lagrelax_bundle",
                "//lb_lagrelax_lbfgs:lagrelax_lbfgs",
                "//lb_colgen:colgen",
                "//opt_branchandcut_cbc:branchandcut_cbc",
                "//opt_branchandcut_cplex:branchandcut_cplex",
                "//opt_branchandcut_gurobi:branchandcut_gurobi",
                "//opt_dip:dip",
                "//opt_constraintprogramming_gecode:constraintprogramming_gecode",
                "//opt_constraintprogramming_cplex:constraintprogramming_cplex",
                "//ub_random:random",
                "//ub_greedy:greedy",
                "//ub_repair:repair",
                "//ub_ls_shiftswap:ls_shiftswap",
                "//ub_ls_ejectionchain:ls_ejectionchain",
                "//ub_localsolver:localsolver",
                "//ub_vdns_simple:vdns_simple",
                "//ub_vnsbranching_cbc:vnsbranching_cbc",
                "//ub_vnsbranching_cplex:vnsbranching_cplex",
        ],
        copts = [] +
            select({":dlib_build":        ["-DDLIB_FOUND"],        "//conditions:default": []}) +
            select({":coinor_build":      ["-DCOINOR_FOUND"],      "//conditions:default": []}) +
            select({":cplex_build":       ["-DCPLEX_FOUND"],       "//conditions:default": []}) +
            select({":gurobi_build":      ["-DGUROBI_FOUND"],      "//conditions:default": []}) +
            select({":gecode_build":      ["-DGECODE_FOUND"],      "//conditions:default": []}) +
            select({":localsolver_build": ["-DLOCALSOLVER_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "gap/lib/",
)

cc_binary(
        name = "main",
        srcs = ["main.cpp"],
        deps = [":algorithms"],
        linkopts = ["-lboost_program_options"] +
            select({"//lib:gurobi_build": [
                        "-lgurobi_c++",
                        "-lgurobi81",
                ], "//conditions:default": [ ], }),
        data = ["//data:data"],
)

cc_binary(
        name = "checker",
        srcs = ["checker.cpp"],
        deps = [":cpp"],
        data = ["//data:data"],
)

cc_binary(
        name = "bench",
        srcs = ["bench.cpp"],
        deps = [
                ":algorithms",
                ":generator",
        ],
        linkopts = [
                "-lstdc++fs",
                "-lboost_program_options",
            ] + select({"//lib:gurobi_build": [
                        "-lgurobi_c++",
                        "-lgurobi81",
                ], "//conditions:default": [ ], }),
        data = ["//data:data"],
)

cc_binary(
        name = "test_main",
        srcs = ["test_main.cpp"],
        deps = [
                "//opt_branchandcut_cbc:branchandcut_cbc",
                ":tester",
        ],
)

cc_binary(
        name = "generator_main",
        srcs = ["generator_main.cpp"],
        deps = [":generator"],
        linkopts = ["-lboost_program_options"],
)

