#include "gap/lib/tester.hpp"

TEST(Cplex, DataTests)
{
    boost::filesystem::path p = boost::filesystem::current_path();
    p /= boost::filesystem::path("opt_cplex");
    p /= boost::filesystem::path("main");

    test(p.string(), "sopt");
}

