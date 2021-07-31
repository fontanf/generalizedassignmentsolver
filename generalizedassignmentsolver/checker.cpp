#include "generalizedassignmentsolver/solution.hpp"

#include <iomanip>

using namespace generalizedassignmentsolver;

int main(int, char *argv[])
{
    std::string instance_path = argv[1];
    std::string solution_path = argv[2];
    Instance instance(instance_path);
    Solution solution(instance, solution_path);
    std::cout << "Item number: " << solution.number_of_items() << "/" << instance.number_of_items() << std::endl;
    std::cout << "Overcapacity: " << solution.overcapacity() << std::endl;
    std::cout << "Feasible: " << solution.feasible() << std::endl;
    std::cout << "Cost: " << solution.cost() << std::endl;
    return 0;
}

