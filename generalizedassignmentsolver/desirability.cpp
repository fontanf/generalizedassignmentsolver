#include "generalizedassignmentsolver/desirability.hpp"

using namespace generalizedassignmentsolver;

std::unique_ptr<Desirability> generalizedassignmentsolver::desirability(
        std::string str,
        const Instance& instance)
{
    if (str == "cij") {
        return std::make_unique<DesirabilityCost>(instance);
    } else if (str == "wij") {
        return std::make_unique<DesirabilityWeight>(instance);
    } else if (str == "cij*wij") {
        return std::make_unique<DesirabilityCostWeight>(instance);
    } else if (str == "-pij/wij") {
        return std::make_unique<DesirabilityEfficiency>(instance);
    } else if (str == "wij/ti") {
        return std::make_unique<DesirabilityWeightCapacity>(instance);

    } else {
        std::cout << "unknown desirability function" << std::endl;
        return std::make_unique<DesirabilityCost>(instance);
    }
}
