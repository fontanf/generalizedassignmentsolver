cc_library(
        name = "constraintprogramming_gecode",
        hdrs = ["constraintprogramming_gecode.hpp"],
        srcs = ["constraintprogramming_gecode.cpp"],
        deps = ["//lib:cpp"],
        copts = select({"//lib:gecode_build": ["-DGECODE_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:gecode_build": [
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
                "//conditions:default": [
                ],
        }),
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
        copts = select({"//lib:gecode_build": ["-DGECODE_FOUND"], "//conditions:default": []}),
        tags = ["requires-network"],
        timeout = "moderate",
)

