#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

Solution random_infeasible(const Instance& ins, std::mt19937_64& gen);
Output random(const Instance& ins, std::mt19937_64& gen, Info info = Info());

}

