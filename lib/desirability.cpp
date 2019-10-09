#include "gap/lib/desirability.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

std::unique_ptr<Desirability> gap::desirability(std::string str, const Instance& ins)
{
    if (str == "cij") {
        return std::make_unique<DesirabilityCost>(ins);
    } else if (str == "wij") {
        return std::make_unique<DesirabilityWeight>(ins);
    } else if (str == "cij*wij") {
        return std::make_unique<DesirabilityCostWeight>(ins);
    } else if (str == "-pij/wij") {
        return std::make_unique<DesirabilityEfficiency>(ins);
    } else if (str == "wij/ti") {
        return std::make_unique<DesirabilityWeightCapacity>(ins);
    } else {
        std::cout << "unknown desirability function" << std::endl;
        return std::make_unique<DesirabilityCost>(ins);
    }
}

