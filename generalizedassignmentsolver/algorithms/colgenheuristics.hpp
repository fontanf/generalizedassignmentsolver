#pragma once

#include "generalizedassignmentsolver/solution.hpp"

namespace generalizedassignmentsolver
{

/**************************** cgh_restrictedmaster ****************************/

struct CghRestrictedMasterOptionalParameters
{
    Info info = Info();

    std::string lp_solver = "clp"; // "clp", "cplex"
};

struct CghRestrictedMasterOutput: Output
{
    CghRestrictedMasterOutput(const Instance& ins, Info& info): Output(ins, info) { }
    CghRestrictedMasterOutput& algorithm_end(Info& info);
};

CghRestrictedMasterOutput cgh_restrictedmaster(const Instance& ins, CghRestrictedMasterOptionalParameters p = {});

/********************************* cgh_greedy *********************************/

struct CghGreedyOptionalParameters
{
    Info info = Info();

    std::string lp_solver = "clp"; // "clp", "cplex"
};

struct CghGreedyOutput: Output
{
    CghGreedyOutput(const Instance& ins, Info& info): Output(ins, info) { }
    CghGreedyOutput& algorithm_end(Info& info);
};

CghGreedyOutput cgh_greedy(const Instance& ins, CghGreedyOptionalParameters p = {});

/************************ cgh_limiteddiscrepencysearch ************************/

struct CghLimitedDiscrepencySearchOptionalParameters
{
    Info info = Info();

    std::string lp_solver = "clp"; // "clp", "cplex"
};

struct CghLimitedDiscrepencySearchOutput: Output
{
    CghLimitedDiscrepencySearchOutput(const Instance& ins, Info& info): Output(ins, info) { }
    CghLimitedDiscrepencySearchOutput& algorithm_end(Info& info);
};

CghLimitedDiscrepencySearchOutput cgh_limiteddiscrepencysearch(const Instance& ins, CghLimitedDiscrepencySearchOptionalParameters p = {});

}

