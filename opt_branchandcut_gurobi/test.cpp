#if GUROBI_FOUND

#include "gap/lib/tester.hpp"
#include "gap/opt_branchandcut_gurobi/branchandcut_gurobi.hpp"

using namespace gap;

Cost opt_branchandcut_gurobi_test(Instance& ins)
{
    Solution sol(ins);
    Cost lb = 0;
    BranchAndCutGurobiData d {.ins = ins, .sol = sol, .lb = lb, .info = Info().set_verbose(true)};
    return sopt_branchandcut_gurobi(d).cost();
}

std::vector<Cost (*)(Instance&)> f = {
        opt_branchandcut_gurobi_test,
};

TEST(MILP, TEST) { test(TEST, f); }

#endif

