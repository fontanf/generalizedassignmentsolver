#include "gap/lib/tester.hpp"

#include "gap/opt_babposta/babposta.hpp"
#include "gap/opt_cplex/cplex.hpp"

using namespace gap;

TEST(Babalt, TestInstances)
{
    boost::filesystem::path p = boost::filesystem::current_path();
    p /= boost::filesystem::path("opt_babposta");
    p /= boost::filesystem::path("main");
    test(p.string() + " -v", "sopt");
}

Value opt_cplex_test(Instance &ins)
{
    Solution sol = sopt_cplex(ins, NULL);
    //std::cout << "SOL " << std::endl;
    //std::cout << sol << std::endl;
    return (sol.item_number() == ins.item_number())? sol.value(): -1;
}

Value opt_babposta_test(Instance& ins)
{
    Info info;
    info.verbose(true);
    Solution sol = sopt_babposta(ins, &info);
    return (sol.item_number() == ins.item_number())? sol.value(): -1;
}

std::vector<Value (*)(Instance&)> tested_functions()
{
    return {
        opt_cplex_test,
        opt_babposta_test,
    };
}

TEST(LR, DataPisingerSmall)
{
    test_gen(
        {"c", "d"},
        {2, 3, 4},
        {1, 2, 3, 4},
        {0, 1, 2, 3, 4, 5, 6, 7},
        1,
        {tested_functions()});
}

TEST(LR, DataPisingerSmallMin)
{
    test_gen(
        {"c", "d"},
        {2, 3, 4},
        {1, 2, 3, 4},
        {0, 1, 2, 3, 4, 5, 6, 7},
        -1,
        {tested_functions()});
}

TEST(LR, DataPisingerSmall2)
{
    test_gen(
        {"c", "d"},
        {2, 3, 4},
        {5, 6},
        {0, 1, 2, 3, 4, 5, 6, 7},
        1,
        {tested_functions()});
}

TEST(LR, DataPisingerSmallMin2)
{
    test_gen(
        {"c", "d"},
        {2, 3, 4},
        {5, 6},
        {0, 1, 2, 3, 4, 5, 6, 7},
        -1,
        {tested_functions()});
}

//TEST(LR, DataPisingerSmall2)
//{
    //test_gen(
        //{"c", "d"},
        //{2, 3, 4},
        //{9, 10, 11, 12},
        //{0, 1, 2, 3},
        //1,
        //{tested_functions()});
//}

//TEST(LR, DataPisingerSmall3)
//{
    //test_gen(
        //{"c", "d"},
        //{2, 3, 4},
        //{13, 14, 15, 16},
        //{0, 1},
        //1,
        //{tested_functions()});
//}

//TEST(LR, DataPisingerMedium)
//{
    //test_gen(
        //{"c", "d"},
        //{5, 10},
        //{20, 30},
        //{0},
        //1,
        //{tested_functions()});
//}

