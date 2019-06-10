#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include "gap/lb_linrelax_clp/linrelax_clp.hpp"

namespace gap
{

struct LSFirstShiftSwapData
{
    const Instance& ins;
    std::mt19937_64& gen;
    Info info = Info();
    PCost alpha = 100;

    LSFirstShiftSwapData& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.find("alpha");
        if (it != args.end())
            alpha = std::stod(it->second);
        return *this;
    }
};
std::vector<std::pair<ItemIdx, AgentIdx>> moves_shift(const Instance& ins);
std::vector<std::pair<ItemIdx, ItemIdx>> moves_swap(const Instance& ins);
std::vector<std::pair<ItemIdx, ItemIdx>> moves_shiftswap(const Instance& ins);
bool shift_iter(Solution& sol_curr, std::vector<std::pair<ItemIdx, ItemIdx>>& moves, std::mt19937_64& gen, std::stringstream& ss);
bool swap_iter(Solution& sol_curr, std::vector<std::pair<ItemIdx, ItemIdx>>& moves, std::mt19937_64& gen, std::stringstream& ss);
bool shiftswap_iter(Solution& sol_curr, std::vector<std::pair<ItemIdx, ItemIdx>>& moves, std::mt19937_64& gen, std::stringstream& ss);
void sol_lsfirst_shift(LSFirstShiftSwapData d, Solution& sol);
void sol_lsfirst_shiftswap(LSFirstShiftSwapData d, Solution& sol);
void sol_lsfirst_shift_swap(LSFirstShiftSwapData d, Solution& sol);
Solution sol_lsfirst_shift(LSFirstShiftSwapData d);
Solution sol_lsfirst_shiftswap(LSFirstShiftSwapData d);
Solution sol_lsfirst_shift_swap(LSFirstShiftSwapData d);

struct LSBestShiftSwapData
{
    const Instance& ins;
    std::mt19937_64& gen;
    Info info = Info();
    double alpha = 100;

    LSBestShiftSwapData& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.find("alpha");
        if (it != args.end())
            alpha = std::stod(it->second);
        return *this;
    }
};
Solution sol_lsbest_shiftswap(LSBestShiftSwapData d);

struct TSShiftSwapData
{
    const Instance& ins;
    std::mt19937_64& gen;
    Info info = Info();
    PCost alpha = 100;
    Cpt tabusize = 16;

    TSShiftSwapData& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.find("alpha");
        if (it != args.end())
            alpha = std::stod(it->second);

        it = args.find("tabusize");
        if (it != args.end())
            tabusize = std::stol(it->second);

        return *this;
    }
};
Solution sol_ts_shiftswap(TSShiftSwapData d);

struct SAShiftSwapData
{
    const Instance& ins;
    std::mt19937_64& gen;
    Info info = Info();
    double alpha = 100;
    double beta = 0.99;
    Cpt l = 100000;

    SAShiftSwapData& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.find("alpha");
        if (it != args.end())
            alpha = std::stod(it->second);

        it = args.find("beta");
        if (it != args.end())
            beta = std::stol(it->second);

        it = args.find("l");
        if (it != args.end())
            l = std::stol(it->second);

        return *this;
    }
};
Solution sol_sa_shiftswap(SAShiftSwapData d);

struct PRShiftSwapData
{
    const Instance& ins;
    std::mt19937_64& gen;
    Info info = Info();
    PCost alpha = 100;
    Cpt rho = 20;
    Cpt gamma = 10;

    PRShiftSwapData& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.find("alpha");
        if (it != args.end())
            alpha = std::stod(it->second);

        it = args.find("rho");
        if (it != args.end())
            rho = std::stol(it->second);

        it = args.find("gamma");
        if (it != args.end())
            gamma = std::stol(it->second);

        return *this;
    }
};
Solution sol_pr_shiftswap(PRShiftSwapData d);

}

