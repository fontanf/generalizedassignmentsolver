#if CPLEX_FOUND

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/algorithms/branchandcut_cplex.hpp"

using namespace generalizedassignmentsolver;

Output branchandcut_cplex_test(Instance& ins)
{
    Info info = Info()
        .set_verbose(true)
        ;
    BranchAndCutCplexOptionalParameters p;
    p.info = info;
    return branchandcut_cplex(ins, p);
}

std::vector<Output (*)(Instance&)> f = {
        branchandcut_cplex_test,
};

TEST(BranchAndCutCplex, TEST) { test(TEST, f, SOPT); }

#endif

