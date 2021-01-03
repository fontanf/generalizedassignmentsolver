#if CPLEX_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/branchandcut_cplex.hpp"

using namespace generalizedassignmentsolver;

Output branchandcut_cplex_test(Instance& instance)
{
    Info info = Info()
        .set_verbose(true)
        ;
    BranchAndCutCplexOptionalParameters parameters;
    parameters.info = info;
    return branchandcut_cplex(instance, parameters);
}

std::vector<Output (*)(Instance&)> branchandcut_cplex_tests = {
        branchandcut_cplex_test,
};

TEST(BranchAndCutCplex, TEST) { test(TEST, branchandcut_cplex_tests, SOPT); }

#endif

