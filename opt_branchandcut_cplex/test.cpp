#if CPLEX_FOUND

#include "gap/lib/tester.hpp"
#include "gap/opt_branchandcut_cplex/branchandcut_cplex.hpp"

using namespace gap;

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

