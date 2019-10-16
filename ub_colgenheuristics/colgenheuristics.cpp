#include "gap/ub_colgenheuristics/colgenheuristics.hpp"

#include "gap/lb_columngeneration/columngeneration.hpp"

using namespace gap;

CghRestrictedMasterOutput& CghRestrictedMasterOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

CghRestrictedMasterOutput gap::sol_cgh_restrictedmaster(const Instance& ins, CghRestrictedMasterOptionalParameters p)
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

CghPureDivingOutput gap::sol_cgh_purediving(const Instance& ins, CghPureDivingOptionalParameters p)
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

CghDivingWithLdsOutput gap::sol_cgh_divingwithlds(const Instance& ins, CghDivingWithLdsOptionalParameters p)
{
    VER(p.info, "*** cgh_divingwithlds " << p.solver << " ***" << std::endl);
    CghDivingWithLdsOutput output(ins, p.info);

    return output.algorithm_end(p.info);
}

