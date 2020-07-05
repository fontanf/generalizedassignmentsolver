#include "generalizedassignmentsolver/algorithms/colgenheuristics.hpp"

#include "generalizedassignmentsolver/algorithms/columngeneration.hpp"

using namespace generalizedassignmentsolver;

/**************************** cgh_restrictedmaster ****************************/

CghRestrictedMasterOutput& CghRestrictedMasterOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

CghRestrictedMasterOutput generalizedassignmentsolver::cgh_restrictedmaster(
        const Instance& instance, CghRestrictedMasterOptionalParameters parameters)
{
    VER(parameters.info, "*** cgh_restrictedmaster " << parameters.lp_solver << " ***" << std::endl);
    CghRestrictedMasterOutput output(instance, parameters.info);

    return output.algorithm_end(parameters.info);
}

/********************************* cgh_greedy *********************************/

CghGreedyOutput& CghGreedyOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

CghGreedyOutput generalizedassignmentsolver::cgh_greedy(
        const Instance& instance, CghGreedyOptionalParameters parameters)
{
    VER(parameters.info, "*** cgh_greedy " << parameters.lp_solver << " ***" << std::endl);
    CghGreedyOutput output(instance, parameters.info);

    Solution solution(instance);
    std::vector<std::vector<std::vector<ItemIdx>>> columns(instance.agent_number());
    std::vector<int> fixed_alternatives(instance.alternative_number(), -1);
    std::vector<int> fixed_agents(instance.agent_number(), 0);
    ColGenOptionalParameters colgen_parameters;
    colgen_parameters.columns      = &columns;
    colgen_parameters.fixed_alt    = &fixed_alternatives;
    colgen_parameters.fixed_agents = &fixed_agents;
    colgen_parameters.lp_solver    = parameters.lp_solver;
    while (!solution.feasible()) {
        //std::cout << "n " << solution.item_number()
            //<< " c " << solution.cost()
            //<< std::endl;
        auto colgen_output = columngeneration(instance, colgen_parameters);
        ColIdx col_best = -1;
        for (ColIdx col = 0; col < (ColIdx)colgen_output.column_indices.size(); ++col)
            if (col_best == -1 || colgen_output.solution[col_best] < colgen_output.solution[col])
                col_best = col;
        if (col_best == 0)
            return output.algorithm_end(parameters.info);
        AgentIdx i_best = colgen_output.column_indices[col_best].first;
        //std::cout << "i_best " << i_best << std::endl;
        for (ItemIdx j: columns[i_best][colgen_output.column_indices[col_best].second]) {
            solution.set(j, i_best);
            for (AgentIdx i = 0; i < instance.agent_number(); ++i)
                fixed_alternatives[instance.alternative_index(j, i)] = 0;
            fixed_alternatives[instance.alternative_index(j, i_best)] = 1;
        }
        fixed_agents[i_best] = 1;
    }
    output.update_solution(solution, std::stringstream(""), parameters.info);

    return output.algorithm_end(parameters.info);
}

/************************ cgh_limiteddiscrepencysearch ************************/

CghLimitedDiscrepencySearchOutput& CghLimitedDiscrepencySearchOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

CghLimitedDiscrepencySearchOutput generalizedassignmentsolver::cgh_limiteddiscrepencysearch(
        const Instance& instance, CghLimitedDiscrepencySearchOptionalParameters parameters)
{
    VER(parameters.info, "*** cgh_limiteddiscrepencysearch " << parameters.lp_solver << " ***" << std::endl);
    CghLimitedDiscrepencySearchOutput output(instance, parameters.info);

    return output.algorithm_end(parameters.info);
}

