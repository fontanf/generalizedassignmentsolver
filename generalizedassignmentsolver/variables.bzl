STDCPP = select({
            "@bazel_tools//src/conditions:windows": ['/std:c++latest'],
            "//conditions:default":                 ["-std=c++14"],})

CLP_COPTS = select({
            "//generalizedassignmentsolver:clp_build": ["-DCLP_FOUND"],
            "//conditions:default": []})
CLP_DEP = select({
            "//generalizedassignmentsolver:clp_windows": ["@clp_windows//:clp"],
            "//conditions:default": []
        }) + select({
            "//generalizedassignmentsolver:clp_linux": ["@clp_linux//:clp"],
            "//conditions:default": []
        }) + select({
            "//generalizedassignmentsolver:clp_darwin": ["@clp_darwin//:clp"],
            "//conditions:default": []})

CBC_COPTS = select({
            "//generalizedassignmentsolver:cbc_build": ["-DCBC_FOUND"],
            "//conditions:default": []})
CBC_DEP = select({
            "//generalizedassignmentsolver:cbc_windows": ["@cbc_windows//:cbc"],
            "//conditions:default": []
        }) + select({
            "//generalizedassignmentsolver:cbc_linux": ["@cbc_linux//:cbc"],
            "//conditions:default": []
        }) + select({
            "//generalizedassignmentsolver:cbc_darwin": ["@cbc_darwin//:cbc"],
            "//conditions:default": []})

VOLUME_COPTS = select({
            "//generalizedassignmentsolver:volume_build": ["-DVOLUME_FOUND"],
            "//conditions:default": []})
VOLUME_DEP = select({
            "//generalizedassignmentsolver:volume_windows": ["@volume_windows//:volume"],
            "//conditions:default": []
        }) + select({
            "//generalizedassignmentsolver:volume_linux": ["@volume_linux//:volume"],
            "//conditions:default": []
        }) + select({
            "//generalizedassignmentsolver:volume_darwin": ["@volume_darwin//:volume"],
            "//conditions:default": []})

CPLEX_COPTS = select({
            "//generalizedassignmentsolver:cplex_build": [
                    "-DCPLEX_FOUND",
                    "-m64",
                    "-DIL_STD"],
            "//conditions:default": []})
CPLEX_DEP = select({
            "//generalizedassignmentsolver:cplex_build": ["@cplex//:cplex"],
            "//conditions:default": []})
CPOPTIMIZER_DEP = select({
            "//generalizedassignmentsolver:cplex_build": ["@cplex//:cpoptimizer"],
            "//conditions:default": []})

GUROBI_COPTS = select({
            "//generalizedassignmentsolver:gurobi_build": ["-DGUROBI_FOUND"],
            "//conditions:default": []})
GUROBI_DEP = select({
            "//generalizedassignmentsolver:gurobi_build": ["@gurobi//:gurobi"],
            "//conditions:default": []})

GECODE_COPTS = select({
            "//generalizedassignmentsolver:gecode_build": ["-DGECODE_FOUND"],
            "//conditions:default": []})
GECODE_DEP = select({
            "//generalizedassignmentsolver:gecode_build": ["@gecode//:gecode"],
            "//conditions:default": []})

LOCALSOLVER_COPTS = select({
            "//generalizedassignmentsolver:localsolver_build": ["-DLOCALSOLVER_FOUND"],
            "//conditions:default": []})
LOCALSOLVER_DEP = select({
            "//generalizedassignmentsolver:localsolver_build": ["@localsolver//:localsolver"],
            "//conditions:default": []})

KNITRO_COPTS = select({
            "//generalizedassignmentsolver:knitro_build": ["-DKNITRO_FOUND"],
            "//conditions:default": []})
KNITRO_DEP = select({
            "//generalizedassignmentsolver:knitro_build": [
                    "@knitro//:knitro",
                    "@knitrocpp//knitrocpp:knitrocpp",
            ],
            "//conditions:default": []})

