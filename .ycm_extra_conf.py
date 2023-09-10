import os


def Settings(**kwargs):
    return {
            'flags': [
                '-x', 'c++',
                '-Wall', '-Wextra', '-Werror',
                '-I', '.',

                '-I', './bazel-generalizedassignmentsolver/external/'
                'json/single_include/',

                '-I', './bazel-generalizedassignmentsolver/external/'
                'googletest/googletest/include/',

                '-I', './bazel-generalizedassignmentsolver/external/'
                'boost/',

                # optimizationtools
                '-I', './bazel-generalizedassignmentsolver/external/'
                # '-I', './../'
                'optimizationtools/',

                # localsearchsolver
                '-I', './bazel-generalizedassignmentsolver/external/'
                # '-I', './../'
                'localsearchsolver/',

                # columngenerationsolver
                '-I', './bazel-generalizedassignmentsolver/external/'
                # '-I', './../'
                'columngenerationsolver/',

                # knapsacksolver
                '-I', './bazel-generalizedassignmentsolver/external/'
                # '-I', './../'
                'knapsacksolver/',

                # dlib
                '-I', './bazel-generalizedassignmentsolver/external/'
                'dlib/dlib-19.19/',

                # COINOR
                '-DCOINOR_FOUND',
                '-I', '/home/florian/Programmes/coinbrew/dist/include/',

                # Gurobi
                '-DGUROBI_FOUND',
                '-I', '/home/florian/Programmes/gurobi811/linux64/include/',

                # CPLEX
                '-DCPLEX_FOUND',
                '-DIL_STD',
                '-I', '/opt/ibm/ILOG/CPLEX_Studio129/concert/include/',
                '-I', '/opt/ibm/ILOG/CPLEX_Studio129/cplex/include/',
                '-I', '/opt/ibm/ILOG/CPLEX_Studio129/cpoptimizer/include/',

                # Gecode
                '-DGECODE_FOUND',
                '-I', '/home/florian/Programmes/gecode-release-6.2.0/',

                # Knitro
                '-DKNITRO_FOUND',
                '-I', os.getenv('KNITRODIR') + '/include/',

                # knitrocpp
                '-I', './bazel-generalizedassignmentsolver/external/'
                # '-I', './../'
                'knitrocpp/',

                ],
            }
