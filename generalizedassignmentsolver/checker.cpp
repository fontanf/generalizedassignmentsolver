#include "generalizedassignmentsolver/solution.hpp"

#include <iomanip>

using namespace generalizedassignmentsolver;

int main(int, char *argv[])
{
    std::string instance_path = argv[1];
    std::string solution_path = argv[2];
    Instance instance(instance_path);
    optimizationtools::Info info;
    info.set_verbosity_level(1);
    init_display(instance, info);

    Solution solution(instance, solution_path);
    info.os()
            << "Solution" << std::endl
            << "--------" << std::endl ;
    solution.print(info.os(), info.verbosity_level());

    return 0;
}

