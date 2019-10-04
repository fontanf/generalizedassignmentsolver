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

struct BranchAndCutCbcOptionalParameters
{
    Info info = Info();

    bool stop_at_first_improvment = false;
};

struct BranchAndCutCbcOutput: Output
{
    BranchAndCutCbcOutput(const Instance& ins, Info& info): Output(ins, info) { }
};

BranchAndCutCbcOutput sopt_branchandcut_cbc(const Instance& ins, BranchAndCutCbcOptionalParameters p = {});

}

#endif

