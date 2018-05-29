#include "gap/lib/tester.hpp"

using namespace gap;

TEST(Babalt, TestInstances)
{
    boost::filesystem::path p = boost::filesystem::current_path();
    p /= boost::filesystem::path("opt_babposta");
    p /= boost::filesystem::path("main");
    test(p.string() + " -v", "sopt");
}

