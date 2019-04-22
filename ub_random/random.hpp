#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

Solution sol_random(const Instance& ins, Cpt seed = 0, Info info = Info());

}

