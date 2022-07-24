#if KNITRO_FOUND

// Useful links:
// https://www.artelys.com/docs/knitro/2_userGuide/gettingStarted/startCallableLibrary.html
// https://www.artelys.com/docs/knitro/3_referenceManual/userOptions.html

#include "generalizedassignmentsolver/algorithms/milp_knitro.hpp"

#include <knitro.h>

using namespace generalizedassignmentsolver;

MilpKnitroOutput& MilpKnitroOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

struct MilpKnitroCallbackUserParams
{
    const Instance& instance;
    MilpKnitroOptionalParameters& parameters;
    MilpKnitroOutput& output;
    const std::vector<std::vector<KNINT>>& x;
};

int milp_knitro_callback(
        KN_context* kc,
        const double* const,
        const double* const,
        void* const user_params_void)
{
    MilpKnitroCallbackUserParams* user_params = (MilpKnitroCallbackUserParams*)user_params_void;

    double bound;
    KN_get_mip_relaxation_bnd(kc, &bound);
    Cost lb = std::ceil(bound - FFOT_TOL);
    user_params->output.update_lower_bound(
            lb,
            std::stringstream(""),
            user_params->parameters.info);

    double obj;
    int return_code = KN_get_mip_incumbent_obj(kc, &obj);
    if (return_code == 0) {
        if (!user_params->output.solution.feasible()
                || user_params->output.solution.cost() > obj + 0.5) {
            Solution solution(user_params->instance);
            AgentIdx m = user_params->instance.number_of_agents();
            ItemIdx n = user_params->instance.number_of_items();
            std::vector<double> values(n * m);
            KN_get_mip_incumbent_x(kc, values.data());
            for (ItemIdx j = 0; j < n; ++j) {
                for (AgentIdx i = 0; i < m; ++i) {
                    if (values[user_params->x[j][i]] > 0.5)
                        solution.set(j, i);
                }
            }
            std::stringstream ss;
            user_params->output.update_solution(
                    solution,
                    std::stringstream(""),
                    user_params->parameters.info);
        }
    }

    return 0;
}

MilpKnitroOutput generalizedassignmentsolver::milp_knitro(
        const Instance& instance,
        MilpKnitroOptionalParameters parameters)
{
    init_display(instance, parameters.info);
    parameters.info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "MILP (Knitro)" << std::endl
            << std::endl;

    MilpKnitroOutput output(instance, parameters.info);

    ItemIdx n = instance.number_of_items();
    AgentIdx m = instance.number_of_agents();

    if (n == 0)
        return output.algorithm_end(parameters.info);

    KN_context* kc;
    int constraint_id;

    // Create a new Knitro solver instance.
    KN_new(&kc);
    if (kc == NULL)
        throw std::runtime_error("Failed to find a valid license.");

    // Variables.
    std::vector<std::vector<KNINT>> x(n, std::vector<KNINT>(m));
    for (ItemIdx j = 0; j < n; j++) {
        for (AgentIdx i = 0; i < m; i++) {
            KN_add_var(kc, &x[j][i]);
            KN_set_var_type(kc, x[j][i], KN_VARTYPE_BINARY);
        }
    }

    // Objective.
    for (ItemIdx j = 0; j < n; j++)
        for (AgentIdx i = 0; i < m; i++)
            KN_add_obj_linear_term(kc, x[j][i], instance.cost(j, i));
    KN_set_obj_goal(kc, KN_OBJGOAL_MINIMIZE);

    // Capacity constraints.
    for (AgentIdx i = 0; i < m; i++) {
        KN_add_con(kc, &constraint_id);
        for (ItemIdx j = 0; j < n; j++)
            KN_add_con_linear_term(kc, constraint_id, x[j][i], instance.weight(j, i));
        KN_set_con_upbnd(kc, constraint_id, instance.capacity(i));
    }

    // One alternative per item constraint.
    for (ItemIdx j = 0; j < n; j++) {
        KN_add_con(kc, &constraint_id);
        for (AgentIdx i = 0; i < m; i++)
            KN_add_con_linear_term(kc, constraint_id, x[j][i], 1);
        KN_set_con_lobnd(kc, constraint_id, 1);
        KN_set_con_upbnd(kc, constraint_id, 1);
    }

    // Redirect standard output to log file.
    KN_set_int_param(kc, KN_PARAM_OUTMODE, KN_OUTMODE_FILE);

    if (parameters.only_linear_relaxation) {
        KN_set_int_param(kc, KN_PARAM_MIP_INTVAR_STRATEGY, KN_MIP_INTVAR_STRATEGY_RELAX);
        KN_solve(kc);
        double obj;
        KN_get_obj_value(kc, &obj);
        Cost lb = std::ceil(obj - FFOT_TOL);
        output.update_lower_bound(lb, std::stringstream("linearrelaxation"), parameters.info);
        for (ItemIdx j = 0; j < n; j++) {
            output.x.push_back(std::vector<double>(m));
            for (AgentIdx i = 0; i < m; i++)
                KN_get_var_primal_value(kc, x[j][i], &(output.x[j][i]));
        }
        return output.algorithm_end(parameters.info);
    }

    // Initial solution.
    if (parameters.initial_solution != NULL && parameters.initial_solution->feasible()) {
        for (ItemIdx j = 0; j < n; ++j) {
            for (AgentIdx i = 0; i < m; ++i) {
                KN_set_mip_var_primal_init_value(
                        kc,
                        x[j][i],
                        (parameters.initial_solution->agent(j) == i)? 1: 0);
            }
        }
    }

    KN_set_double_param(kc, KN_PARAM_MIP_OPTGAPREL, 0); // Fix precision issue
    KN_set_double_param(kc, KN_PARAM_MIP_OPTGAPABS, 0); // Fix precision issue

    // Time limit.
    if (parameters.info.time_limit != std::numeric_limits<double>::infinity())
        KN_set_double_param(kc, KN_PARAM_MIP_MAXTIMECPU, parameters.info.time_limit);

    // Callback
    MilpKnitroCallbackUserParams user_params {instance, parameters, output, x};
    KN_set_mip_node_callback(kc, milp_knitro_callback, (void*)(&user_params));

    // Optimize.
    int status = KN_solve(kc);

    // https://www.artelys.com/docs/knitro/3_referenceManual/returnCodes.html#returncodes
    double obj;
    double bound;
    double value;
    int return_code = KN_get_mip_incumbent_obj(kc, &obj);
    if (status == KN_RC_MIP_EXH_INFEAS) {
        output.update_lower_bound(instance.bound(), std::stringstream(""), parameters.info);
    } else if (status == KN_RC_OPTIMAL_OR_SATISFACTORY) {
        if (!output.solution.feasible() || output.solution.cost() > obj + 0.5) {
            Solution solution(instance);
            for (ItemIdx j = 0; j < n; ++j) {
                for (AgentIdx i = 0; i < m; ++i) {
                    KN_get_var_primal_value(kc, x[j][i], &value);
                    if (value > 0.5)
                        solution.set(j, i);
                }
            }
            output.update_solution(solution, std::stringstream(""), parameters.info);
        }
        output.update_lower_bound(output.solution.cost(), std::stringstream(""), parameters.info);
    } else if (return_code == 0) {
        if (!output.solution.feasible() || output.solution.cost() > obj + 0.5) {
            Solution solution(instance);
            for (ItemIdx j = 0; j < n; ++j) {
                for (AgentIdx i = 0; i < m; ++i) {
                    KN_get_var_primal_value(kc, x[j][i], &value);
                    if (value > 0.5)
                        solution.set(j, i);
                }
            }
            output.update_solution(solution, std::stringstream(""), parameters.info);
        }
        KN_get_mip_relaxation_bnd(kc, &bound);
        Cost lb = std::ceil(bound - FFOT_TOL);
        output.update_lower_bound(lb, std::stringstream(""), parameters.info);
    } else {
        KN_get_mip_relaxation_bnd(kc, &bound);
        Cost lb = std::ceil(bound - FFOT_TOL);
        output.update_lower_bound(lb, std::stringstream(""), parameters.info);
    }

    return output.algorithm_end(parameters.info);
}

#endif

