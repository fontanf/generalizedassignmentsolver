cc_library(
        name = "constraintprogramming_gecode",
        hdrs = ["constraintprogramming_gecode.hpp"],
        srcs = ["constraintprogramming_gecode.cpp"],
        deps = ["//lib:cpp"],
        linkopts = [
                "-lgecodeflatzinc",
                "-lgecodedriver",
                "-lgecodegist",
                "-lgecodesearch",
                "-lgecodeminimodel",
                "-lgecodeset",
                "-lgecodefloat",
                "-lgecodeint",
                "-lgecodekernel",
                "-lgecodesupport",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/opt_constraintprogramming_gecode/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//opt_constraintprogramming_gecode:constraintprogramming_gecode",
        ],
        timeout = "moderate",
)

