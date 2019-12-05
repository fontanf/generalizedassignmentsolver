#pragma once

#include "generalizedassignment/lib/solution.hpp"

namespace generalizedassignment
{

Solution sol_random_infeasible(const Instance& ins, std::mt19937_64& gen);
Output sol_random(const Instance& ins, std::mt19937_64& gen, Info info = Info());

}

