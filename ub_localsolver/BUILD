cc_library(
        name = "localsolver",
        hdrs = ["localsolver.hpp"],
        srcs = ["localsolver.cpp"],
        deps = ["//lib:cpp"],
        copts = select({"//lib:localsolver_build": ["-DLOCALSOLVER_FOUND"], "//conditions:default": []}),
        linkopts = select({
                "//lib:localsolver_build": [
                        "-llocalsolver",
                        "-lpthread",
                ],
                "//conditions:default": [
                ],
        }),
        visibility = ["//visibility:public"],
        include_prefix = "generalizedassignment/ub_localsolver/",
)

cc_test(
        name = "test",
        srcs = ["test.cpp"],
        deps = [
                "//lib:tester",
                "//ub_localsolver:localsolver",
        ],
        timeout = "moderate",
)

