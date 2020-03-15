STDCPP = select({
            "@bazel_tools//src/conditions:windows": ['/std:c++latest'],
            "//conditions:default":                 ["-std=c++14"],})

DLIB_COPTS = select({
            "//generalizedassignmentsolver:dlib_build": ["-DDLIB_FOUND"],
            "//conditions:default":                     []})
COINOR_COPTS = select({
            "//generalizedassignmentsolver:coinor_build": ["-DCOINOR_FOUND"],
            "//conditions:default":                       []})
CPLEX_COPTS = select({
            "//generalizedassignmentsolver:cplex_build": [
                    "-DCPLEX_FOUND",
                    "-m64",
                    "-DIL_STD"],
            "//conditions:default": []})
GUROBI_COPTS = select({
            "//generalizedassignmentsolver:gurobi_build": ["-DGUROBI_FOUND"],
            "//conditions:default": []})
GECODE_COPTS = select({
            "//generalizedassignmentsolver:gecode_build": ["-DGECODE_FOUND"],
            "//conditions:default": []})
LOCALSOLVER_COPTS = select({
            "//generalizedassignmentsolver:localsolver_build": ["-DLOCALSOLVER_FOUND"],
            "//conditions:default": []})

DLIB_LINKOPTS = select({
            "//generalizedassignmentsolver:dlib_build": ["-ldlib"],
            "//conditions:default":                     []})
COINOR_LINKOPTS = select({
            "//generalizedassignmentsolver:coinor_build": [
                    "-lClp",
                    "-lOsiClp",
                    "-lCoinUtils"],
            "//conditions:default": []})
CPLEX_LINKOPTS = select({
            "//generalizedassignmentsolver:cplex_build": [
                    "-lconcert",
                    "-lilocplex",
                    "-lcp",
                    "-lcplex",
                    "-lm",
                    "-lpthread",
                    "-ldl"],
            "//conditions:default": []})
GUROBI_LINKOPTS = select({
            "//generalizedassignmentsolver:gurobi_build": [
                    "-lgurobi_c++",
                    "-lgurobi81"],
            "//conditions:default": []})
GECODE_LINKOPTS = select({
            "//generalizedassignmentsolver:gecode_build": [
                    "-lgecodeflatzinc",
                    "-lgecodedriver",
                    "-lgecodegist",
                    "-lgecodesearch",
                    "-lgecodeminimodel",
                    "-lgecodeset",
                    "-lgecodefloat",
                    "-lgecodeint",
                    "-lgecodekernel",
                    "-lgecodesupport"],
            "//conditions:default": []})
LOCALSOLVER_LINKOPTS = select({
            "//generalizedassignmentsolver:localsolver_build": [
                    "-llocalsolver",
                    "-lpthread"],
            "//conditions:default": []})

