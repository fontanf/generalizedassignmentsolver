#include "generalizedassignmentsolver/algorithms/branchandcut_cbc.hpp"

#include "generalizedassignmentsolver/tester.hpp"
#include "generalizedassignmentsolver/generator.hpp"

using namespace generalizedassignmentsolver;

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    for (int i=0; i<=9; ++i) {
        Instance ins = test_instance(i);
        std::cout << ins << std::endl;
        //Solution sol = milp(ins);
        //std::cout << sol << std::endl;
    }

}

