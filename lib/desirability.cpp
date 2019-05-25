#include "gap/lib/desirability.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

std::unique_ptr<Desirability> gap::desirability(std::string str, const Instance& ins)
{
    if (str == "cij") {
        return std::unique_ptr<Desirability>(new DesirabilityCost(ins));
    } else if (str == "wij") {
        return std::unique_ptr<Desirability>(new DesirabilityWeight(ins));
    } else if (str == "cij*wij") {
        return std::unique_ptr<Desirability>(new DesirabilityCostWeight(ins));
    } else if (str == "-pij/wij") {
        return std::unique_ptr<Desirability>(new DesirabilityEfficiency(ins));
    } else if (str == "wij/ti") {
        return std::unique_ptr<Desirability>(new DesirabilityWeightCapacity(ins));
    } else {
        std::cout << "unknown desirability function" << std::endl;
        return std::unique_ptr<Desirability>(new DesirabilityCost(ins));
    }
}

