#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

Output run(std::string algorithm, const Instance& instance, std::mt19937_64& generator, Info info);

}

