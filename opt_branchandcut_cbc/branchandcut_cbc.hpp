#pragma once

#if COINOR_FOUND

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include <coin/CbcModel.hpp>
#include <coin/OsiCbcSolverInterface.hpp>
#include <coin/CglKnapsackCover.hpp>
#include <coin/CglClique.hpp>

namespace gap
{

struct MilpMatrix
{
    MilpMatrix(const Instance& ins);
    CoinPackedMatrix matrix;
    std::vector<double> col_lower;
    std::vector<double> col_upper;
    std::vector<double> objective;
    std::vector<double> row_lower;
    std::vector<double> row_upper;
};

struct BranchAndCutCbcData
{
    const Instance& ins;
    Solution& sol;
    Cost& lb;
    bool stop_at_first_improvment = false;
    Info info = Info();
};

Solution sopt_branchandcut_cbc(BranchAndCutCbcData d);

}

#endif

