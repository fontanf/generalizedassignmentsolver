cc_library(
        name = "cpp",
        hdrs = [
                "instance.hpp",
                "solution.hpp",
                "desirability.hpp",
        ],
        srcs = [
                "instance.cpp",
                "solution.cpp",
                "desirability.cpp",
        ],
        deps = ["@benchtools//:info"],
        linkopts = [
                "-lboost_system",
                "-lboost_filesystem",
                "-lboost_program_options",
        ],
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

cc_binary(
        name = "main",
        srcs = ["main.cpp"],
        deps = [
                "//lb_linrelax_clp:linrelax_clp",
                "//lb_lagrelax_volume:lagrelax_volume",
                "//lb_lagrelax_bundle:lagrelax_bundle",
                "//opt_branchandcut_cbc:branchandcut_cbc",
                #"//opt_branchandcut_cplex:branchandcut_cplex",
                "//opt_dip:dip",
                "//opt_constraintprogramming_gecode:constraintprogramming_gecode",
                #"//opt_constraintprogramming_cplex:constraintprogramming_cplex",
                "//ub_random:random",
                "//ub_greedy:greedy",
                "//ub_repair:repair",
                "//ub_ls_shiftswap:ls_shiftswap",
                "//ub_ls_ejectionchain:ls_ejectionchain",
                "//ub_vdns_simple:vdns_simple",
                "//ub_vnsbranching_cbc:vnsbranching_cbc",
                #"//ub_vnsbranching_cplex:vnsbranching_cplex",
        ],
)

cc_binary(
        name = "checker",
        srcs = ["checker.cpp"],
        deps = [":cpp"],
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
)

