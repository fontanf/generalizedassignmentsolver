#include "gap/lib/tester.hpp"
#include "gap/lib/generator.hpp"
#include "gap/ub_lagrangian/lagrangian.hpp"
#include "gap/opt_cplex/cplex.hpp"

using namespace gap;

Profit opt_cplex_test(Instance &ins)
{
    Solution sol = sopt_cplex(ins, NULL);
    return (sol.item_number() == ins.item_number())? sol.profit(): -1;
}

Profit ub_lagrangian_test(Instance& ins)
{
    Info info;
    info.verbose(true);
    return ub_lagrangian(ins, &info).u;
}

std::vector<Profit (*)(Instance&)> tested_functions()
{
    return {
        opt_cplex_test,
        ub_lagrangian_test,
    };
}

TEST(LR, DataPisingerSmall)
{
    test_gen(
        {"c", "d"},
        {2, 3},
        {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20},
        {0,1,2},
        1,
        {tested_functions()}, 1);
}

TEST(LR, DataPisingerMedium)
{
    test_gen(
        {"c", "d"},
        {5, 10},
        {10, 20, 30, 40},
        {0,1,2},
        1,
        {tested_functions()}, 1);
}
