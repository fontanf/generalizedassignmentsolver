#pragma once

#include "generalizedassignment/lib/solution.hpp"

namespace generalizedassignment
{

/**************************** cgh_restrictedmaster ****************************/

struct CghRestrictedMasterOptionalParameters
{
    Info info = Info();

    std::string solver = "clp"; // "clp", "cplex"

    CghRestrictedMasterOptionalParameters& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.end();
        if ((it = args.find("solver")) != args.end()) solver = it->second;
        return *this;
    }
};

struct CghRestrictedMasterOutput: Output
{
    CghRestrictedMasterOutput(const Instance& ins, Info& info): Output(ins, info) { }
    CghRestrictedMasterOutput& algorithm_end(Info& info);
};

CghRestrictedMasterOutput sol_cgh_restrictedmaster(const Instance& ins, CghRestrictedMasterOptionalParameters p = {});

/******************************* cgh_purediving *******************************/

struct CghPureDivingOptionalParameters
{
    Info info = Info();

    std::string solver = "clp"; // "clp", "cplex"

    CghPureDivingOptionalParameters& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.end();
        if ((it = args.find("solver")) != args.end()) solver = it->second;
        return *this;
    }
};

struct CghPureDivingOutput: Output
{
    CghPureDivingOutput(const Instance& ins, Info& info): Output(ins, info) { }
    CghPureDivingOutput& algorithm_end(Info& info);
};

CghPureDivingOutput sol_cgh_purediving(const Instance& ins, CghPureDivingOptionalParameters p = {});

/***************************** cgh_divingwithlds ******************************/

struct CghDivingWithLdsOptionalParameters
{
    Info info = Info();

    std::string solver = "clp"; // "clp", "cplex"

    CghDivingWithLdsOptionalParameters& set_params(const std::map<std::string, std::string>& args)
    {
        auto it = args.end();
        if ((it = args.find("solver")) != args.end()) solver = it->second;
        return *this;
    }
};

struct CghDivingWithLdsOutput: Output
{
    CghDivingWithLdsOutput(const Instance& ins, Info& info): Output(ins, info) { }
    CghDivingWithLdsOutput& algorithm_end(Info& info);
};

CghDivingWithLdsOutput sol_cgh_divingwithlds(const Instance& ins, CghDivingWithLdsOptionalParameters p = {});

}

