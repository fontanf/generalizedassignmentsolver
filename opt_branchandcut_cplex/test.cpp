#include "gap/lib/tester.hpp"
#include "gap/opt_branchandcut_cplex/branchandcut_cplex.hpp"

using namespace gap;

Cost opt_branchandcut_cplex_test(Instance& ins)
{
    Solution sol(ins);
    BranchAndCutCplexData d {.ins = ins, .sol = sol, .info = Info().set_verbose(true)};
    return sopt_branchandcut_cplex(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        opt_branchandcut_cplex_test,
};

TEST(MILP, TEST) { test(TEST, f); }

