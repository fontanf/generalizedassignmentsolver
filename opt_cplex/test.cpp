#include "gap/lib/tester.hpp"

TEST(Cplex, DataTests)
{
    auto p = boost::filesystem::current_path() / "opt_cplex" / "main";
    gap::test(p.string(), "sopt");
}

