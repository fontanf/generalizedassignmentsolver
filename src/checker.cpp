#include "generalizedassignmentsolver/solution.hpp"
#include "generalizedassignmentsolver/instance_builder.hpp"

using namespace generalizedassignmentsolver;

int main(int, char *argv[])
{
    std::string instance_path = argv[1];
    std::string solution_path = argv[2];
    InstanceBuilder instance_builder;
    instance_builder.read(
            instance_path);
    const Instance instance = instance_builder.build();

    Solution solution(instance, solution_path);
    std::cout
            << "Solution" << std::endl
            << "--------" << std::endl ;
    solution.format(std::cout, 2);

    return 0;
}
