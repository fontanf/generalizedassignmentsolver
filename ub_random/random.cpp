#include "gap/ub_random/random.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

Solution sol_random(const Instance& ins, Info info)
{
    Solution sol(ins);
    return algorithm_end(sol, info);
}

