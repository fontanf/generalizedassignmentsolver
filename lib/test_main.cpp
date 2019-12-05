#include "generalizedassignment/opt_branchandcut_cbc/branchandcut_cbc.hpp"

#include "generalizedassignment/lib/tester.hpp"
#include "generalizedassignment/lib/generator.hpp"

using namespace generalizedassignment;

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    for (int i=0; i<=9; ++i) {
        Instance ins = test_instance(i);
        std::cout << ins << std::endl;
        //Solution sol = sopt_milp(ins);
        //std::cout << sol << std::endl;
    }

}

