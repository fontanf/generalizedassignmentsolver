#if CPLEX_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/milp_cplex.hpp"

using namespace generalizedassignmentsolver;

Output milp_cplex_test(Instance& instance)
{
    optimizationtools::Info info = optimizationtools::Info()
        .set_verbosity_level(1)
        ;
    MilpCplexParameters parameters;
    parameters.info = info;
    return milp_cplex(instance, parameters);
}

std::vector<Output (*)(Instance&)> milp_cplex_tests = {
        milp_cplex_test,
};

TEST(MilpCplex, TEST) { test(TEST, milp_cplex_tests, SOPT); }

#endif

