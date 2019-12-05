#if CPLEX_FOUND

#include "generalizedassignment/lib/tester.hpp"
#include "generalizedassignment/opt_branchandcut_cplex/branchandcut_cplex.hpp"

using namespace generalizedassignment;

Output sopt_branchandcut_cplex_test(Instance& ins)
{
    Info info = Info()
        .set_verbose(true)
        ;
    BranchAndCutCplexOptionalParameters p;
    p.info = info;
    return sopt_branchandcut_cplex(ins, p);
}

std::vector<Output (*)(Instance&)> f = {
        sopt_branchandcut_cplex_test,
};

TEST(BranchAndCutCplex, TEST) { test(TEST, f, SOPT); }

#endif

