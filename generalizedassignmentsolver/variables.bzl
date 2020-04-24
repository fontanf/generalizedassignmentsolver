STDCPP = select({
            "@bazel_tools//src/conditions:windows": ['/std:c++latest'],
            "//conditions:default":                 ["-std=c++14"],})

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

COINOR_DEP = select({
            "//generalizedassignmentsolver:coinor_build": ["@coinor//:coinor"],
            "//conditions:default": []})
CPLEX_DEP = select({
            "//generalizedassignmentsolver:cplex_build": ["@cplex//:cplex"],
            "//conditions:default": []})
CPOPTIMIZER_DEP = select({
            "//generalizedassignmentsolver:cplex_build": ["@cplex//:cpoptimizer"],
            "//conditions:default": []})
GUROBI_DEP = select({
            "//generalizedassignmentsolver:gurobi_build": ["@gurobi//:gurobi"],
            "//conditions:default": []})
GECODE_DEP = select({
            "//generalizedassignmentsolver:gecode_build": ["@gecode//:gecode"],
            "//conditions:default": []})
LOCALSOLVER_DEP = select({
            "//generalizedassignmentsolver:localsolver_build": ["@localsolver//:localsolver"],
            "//conditions:default": []})

