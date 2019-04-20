#include "gap/opt_milp/milp.hpp"

#include "gap/lib/tester.hpp"
#include "gap/lib/generator.hpp"

using namespace gap;

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    for (int i=0; i<=9; ++i) {
        Instance ins = test_instance(i);
        std::cout << ins << std::endl;
        Solution sol = sopt_milp(ins);
        std::cout << sol.to_string() << std::endl;
    }

}

