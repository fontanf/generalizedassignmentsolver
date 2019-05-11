#pragma once

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
    std::vector<double> colLower;
    std::vector<double> colUpper;
    std::vector<double> objective;
    std::vector<double> rowLower;
    std::vector<double> rowUpper;
};

struct BranchAndCutCbcData
{
    const Instance& ins;
    Solution& sol;
    bool stop_at_first_improvment = false;
    Info info = Info();
};

Solution sopt_branchandcut_cbc(BranchAndCutCbcData d);

}

