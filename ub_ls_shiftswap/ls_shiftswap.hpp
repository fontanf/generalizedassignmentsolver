#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

bool shift_swap_first(Solution& sol, std::vector<std::pair<ItemIdx, ItemIdx>>& alt,
        std::default_random_engine& gen, Info& info);
bool shift_swap_best(const Instance& ins, Solution& sol, Info& info);

Solution sol_ls_shiftswap_first(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info = Info());
Solution sol_ls_shiftswap_best(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info = Info());

}

