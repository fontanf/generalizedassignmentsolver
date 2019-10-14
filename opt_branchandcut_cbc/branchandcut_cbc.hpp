#pragma once

#if COINOR_FOUND

#include "gap/lib/solution.hpp"

#include <coin/CbcModel.hpp>
#include <coin/OsiCbcSolverInterface.hpp>
#include <coin/CglKnapsackCover.hpp>
#include <coin/CglClique.hpp>

namespace gap
{

struct CoinLP
{
    CoinLP(const Instance& ins);

    std::vector<double> col_lower;
    std::vector<double> col_upper;
    std::vector<double> objective;

    std::vector<double> row_lower;
    std::vector<double> row_upper;
    CoinPackedMatrix matrix;
};

struct BranchAndCutCbcOptionalParameters
{
    Info info = Info();

    const Solution* initial_solution = NULL;
    bool stop_at_first_improvment = false;
};

struct BranchAndCutCbcOutput: Output
{
    BranchAndCutCbcOutput(const Instance& ins, Info& info): Output(ins, info) { }
    BranchAndCutCbcOutput& algorithm_end(Info& info);
};

BranchAndCutCbcOutput sopt_branchandcut_cbc(const Instance& ins, BranchAndCutCbcOptionalParameters p = {});

}

#endif

