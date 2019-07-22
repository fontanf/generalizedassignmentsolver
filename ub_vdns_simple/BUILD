cc_library(
        name = "vdns_simple",
        hdrs = ["vdns_simple.hpp"],
        srcs = ["vdns_simple.cpp"],
        deps = [
                "//lib:cpp",
                "//ub_random:random",
                "//ub_repair:repair",
                "//opt_branchandcut_cbc:branchandcut_cbc",
        ],
        copts = select({"//lib:coinor_build": ["-DCOINOR_FOUND"], "//conditions:default": []}),
        visibility = ["//visibility:public"],
        include_prefix = "gap/ub_vdns_simple/",
)

