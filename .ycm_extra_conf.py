def Settings( **kwargs ):
    return {
            'flags': [
                '-x', 'c++',
                '-Wall', '-Wextra', '-Werror',
                '-DCOINOR_FOUND',
                '-DDLIB_FOUND',
                '-DCPLEX_FOUND',
                '-DGUROBI_FOUND',
                '-DGECODE_FOUND',
                '-DIL_STD', # Cplex
                '-I', '.',
                '-I', './bazel-generalizedassignmentsolver/external/json/single_include',
                '-I', './bazel-generalizedassignmentsolver/external/googletest/googletest/include/',
                '-I', './bazel-generalizedassignmentsolver/external/knapsacksolver',
                '-I', './bazel-generalizedassignmentsolver/external/',
                ],
            }

