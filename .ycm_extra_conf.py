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
                '-I', '..',
                '-I', './bazel-gap/external/json/single_include',
                '-I', './bazel-gap/external/googletest/googletest/include/',
                '-I', './bazel-gap/external/',
                ],
            }

