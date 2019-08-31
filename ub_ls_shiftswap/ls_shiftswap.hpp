#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

#include "gap/lb_linrelax_clp/linrelax_clp.hpp"

namespace gap
{

struct LSShiftSwapData
{
    const Instance& ins;
    std::mt19937_64& gen;
    Info info = Info();

    LSShiftSwapData& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.end();
        (void)it;
        return *this;
    }
};
Solution sol_ls_shiftswap(LSShiftSwapData d);

struct TSShiftSwapData
{
    const Instance& ins;
    std::mt19937_64& gen;
    Info info = Info();
    Cpt l = 10000;

    TSShiftSwapData& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.end();
        if ((it = args.find("l")) != args.end()) l = std::stod(it->second);
        return *this;
    }
};
Solution sol_ts_shiftswap(TSShiftSwapData d);

struct SAShiftSwapData
{
    const Instance& ins;
    std::mt19937_64& gen;
    Info info = Info();
    double beta = 0.999;
    double l = 1000000;

    SAShiftSwapData& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.end();
        if ((it = args.find("beta")) != args.end()) beta = std::stod(it->second);
        if ((it = args.find("l")) != args.end()) l = std::stol(it->second);
        return *this;
    }
};
Solution sol_sa_shiftswap(SAShiftSwapData d);

}

