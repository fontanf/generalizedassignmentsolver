#include "generalizedassignmentsolver/algorithms/colgenheuristics.hpp"

#include "generalizedassignmentsolver/algorithms/columngeneration.hpp"

using namespace generalizedassignmentsolver;

CghRestrictedMasterOutput& CghRestrictedMasterOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

CghRestrictedMasterOutput generalizedassignmentsolver::cgh_restrictedmaster(const Instance& ins, CghRestrictedMasterOptionalParameters p)
{
    VER(p.info, "*** cgh_restrictedmaster " << p.solver << " ***" << std::endl);
    CghRestrictedMasterOutput output(ins, p.info);

    return output.algorithm_end(p.info);
}

/******************************* cgh_purediving *******************************/

CghPureDivingOutput& CghPureDivingOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

CghPureDivingOutput generalizedassignmentsolver::cgh_purediving(const Instance& ins, CghPureDivingOptionalParameters p)
{
    VER(p.info, "*** cgh_purediving " << p.solver << " ***" << std::endl);
    CghPureDivingOutput output(ins, p.info);

    return output.algorithm_end(p.info);
}

/***************************** cgh_divingwithlds ******************************/

CghDivingWithLdsOutput& CghDivingWithLdsOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

CghDivingWithLdsOutput generalizedassignmentsolver::cgh_divingwithlds(const Instance& ins, CghDivingWithLdsOptionalParameters p)
{
    VER(p.info, "*** cgh_divingwithlds " << p.solver << " ***" << std::endl);
    CghDivingWithLdsOutput output(ins, p.info);

    return output.algorithm_end(p.info);
}

