#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

Solution sol_lsfirst_shiftswap(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info = Info());
Solution sol_lsbest_shiftswap(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info = Info());
Solution sol_ts_shiftswap(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info = Info());
Solution sol_sa_shiftswap(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info = Info());
Solution sol_pr_shiftswap(const Instance& ins, Solution& sol, std::default_random_engine& gen, Info info = Info());

bool doubleshift_best(const Instance& ins, Solution& sol, Info& info);
bool tripleswap_best(const Instance& ins, Solution& sol, Info& info);

}

