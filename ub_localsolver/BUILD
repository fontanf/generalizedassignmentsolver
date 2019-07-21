cc_library(
        name = "localsolver",
        hdrs = ["localsolver.hpp"],
        srcs = ["localsolver.cpp"],
        deps = ["//lib:cpp"],
        linkopts = [
                "-llocalsolver",
                "-lpthread",
        ],
        visibility = ["//visibility:public"],
        include_prefix = "gap/ub_localsolver/",
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

