def Settings( **kwargs ):
    return {
            'flags': [
                '-x', 'c++',
                '-Wall', '-Wextra', '-Werror',
                # '-DCOINOR_FOUND',
                '-DCPLEX_FOUND',
                '-DGUROBI_FOUND',
                '-DGECODE_FOUND',
                '-DIL_STD', # Cplex
                '-I', '.',
                '-I', '/home/florian/Programmes/gurobi811/linux64/include/',
                '-I', '/opt/ibm/ILOG/CPLEX_Studio129/concert/include/',
                '-I', '/opt/ibm/ILOG/CPLEX_Studio129/cplex/include/',
                '-I', '/opt/ibm/ILOG/CPLEX_Studio129/cpoptimizer/include/',
                '-I', './bazel-generalizedassignmentsolver/external/json/single_include/',
                '-I', './bazel-generalizedassignmentsolver/external/googletest/googletest/include/',
                '-I', './bazel-generalizedassignmentsolver/external/knapsacksolver/',
                '-I', './bazel-generalizedassignmentsolver/external/optimizationtools/',
                '-I', './bazel-generalizedassignmentsolver/external/dlib/dlib-19.19/',
                ],
            }

