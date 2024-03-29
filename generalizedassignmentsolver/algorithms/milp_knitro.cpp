#if KNITRO_FOUND

// Useful links:
// https://www.artelys.com/docs/knitro/2_userGuide/gettingStarted/startCallableLibrary.html
// https://www.artelys.com/docs/knitro/3_referenceManual/userOptions.html

#include "generalizedassignmentsolver/algorithms/milp_knitro.hpp"

#include "knitrocpp/knitro.hpp"

using namespace generalizedassignmentsolver;

const MilpKnitroOutput generalizedassignmentsolver::milp_knitro(
        const Instance& instance,
        const MilpKnitroParameters& parameters)
{
    MilpKnitroOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("MILP (Knitro)");
    algorithm_formatter.print_header();

    if (instance.number_of_items() == 0) {
        algorithm_formatter.end();
        return output;
    }

    // Create a new Knitro context.
    knitrocpp::Context knitro_context;

    // Variables.
    std::vector<std::vector<KNINT>> x(
            instance.number_of_items(),
            std::vector<KNINT>(instance.number_of_agents()));
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        for (AgentIdx agent_id = 0;
                agent_id < instance.number_of_agents();
                ++agent_id) {
            x[item_id][agent_id] = knitro_context.add_var();
            knitro_context.set_var_type(
                    x[item_id][agent_id],
                    KN_VARTYPE_BINARY);
        }
    }

    // Objective.
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        for (AgentIdx agent_id = 0;
                agent_id < instance.number_of_agents();
                ++agent_id) {
            knitro_context.add_obj_linear_term(
                    x[item_id][agent_id],
                    instance.cost(item_id, agent_id));
        }
    }
    knitro_context.set_obj_goal(KN_OBJGOAL_MINIMIZE);

    // Capacity constraints.
    for (AgentIdx agent_id = 0;
            agent_id < instance.number_of_agents();
            ++agent_id) {
        knitrocpp::ConstraintId constraint_id = knitro_context.add_con();
        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            knitro_context.add_con_linear_term(
                    constraint_id,
                    x[item_id][agent_id],
                    instance.weight(item_id, agent_id));
        }
        knitro_context.set_con_upbnd(
                constraint_id,
                instance.capacity(agent_id));
    }

    // One alternative per item constraint.
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        knitrocpp::ConstraintId constraint_id = knitro_context.add_con();
        for (AgentIdx agent_id = 0;
                agent_id < instance.number_of_agents();
                ++agent_id) {
            knitro_context.add_con_linear_term(
                    constraint_id,
                    x[item_id][agent_id],
                    1);
        }
        knitro_context.set_con_lobnd(constraint_id, 1);
        knitro_context.set_con_upbnd(constraint_id, 1);
    }

    // Redirect standard output to log file.
    knitro_context.set_int_param(KN_PARAM_OUTMODE, KN_OUTMODE_FILE);

    if (parameters.only_linear_relaxation) {
        knitro_context.set_int_param(
                KN_PARAM_MIP_INTVAR_STRATEGY,
                KN_MIP_INTVAR_STRATEGY_RELAX);
        knitro_context.solve();
        double obj = knitro_context.get_obj_value();
        Cost lb = std::ceil(obj - FFOT_TOL);
        algorithm_formatter.update_bound(lb, "linearrelaxation");
        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            output.x.push_back(std::vector<double>(instance.number_of_agents()));
            for (AgentIdx agent_id = 0;
                    agent_id < instance.number_of_agents();
                    ++agent_id) {
                output.x[item_id][agent_id] = knitro_context.get_var_primal_value(
                        x[item_id][agent_id]);
            }
        }

        algorithm_formatter.end();
        return output;
    }

    // Initial solution.
    if (parameters.initial_solution != NULL
            && parameters.initial_solution->feasible()) {
        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            for (AgentIdx agent_id = 0;
                    agent_id < instance.number_of_agents();
                    ++agent_id) {
                knitro_context.set_mip_var_primal_init_value(
                        x[item_id][agent_id],
                        (parameters.initial_solution->agent(item_id) == agent_id)? 1: 0);
            }
        }
    }

    knitro_context.set_double_param(KN_PARAM_MIP_OPTGAPREL, 0); // Fix precision issue
    knitro_context.set_double_param(KN_PARAM_MIP_OPTGAPABS, 0); // Fix precision issue

    // Time limit.
    if (parameters.timer.time_limit() != std::numeric_limits<double>::infinity()) {
        knitro_context.set_double_param(
                KN_PARAM_MIP_MAXTIMECPU,
                parameters.timer.time_limit());
    }

    // Callback
    knitro_context.set_mip_node_callback(
        [&instance, &parameters, &output, &x](
                const knitrocpp::Context& knitro_context,
                const double* const,
                const double* const)
        {
            double bound = knitro_context.get_mip_relaxation_bnd();
            Cost lb = std::ceil(bound - FFOT_TOL);
            algorithm_formatter.update_bound(lb, "");

            if (knitro_context.has_mip_incumbent()) {
                double obj = knitro_context.get_mip_incumbent_obj();
                if (!output.solution.feasible()
                        || output.solution.cost() > obj + 0.5) {
                    Solution solution(instance);
                    std::vector<double> values = knitro_context.get_mip_incumbent_x();
                    for (ItemIdx item_id = 0;
                            item_id < instance.number_of_items();
                            ++item_id) {
                        for (AgentIdx agent_id = 0;
                                agent_id < instance.number_of_agents();
                                ++agent_id) {
                            if (values[x[item_id][agent_id]] > 0.5)
                                solution.set(item_id, agent_id);
                        }
                    }
                    algorithm_formatter.update_solution(solution, "");
                }
            }
            return 0;
        });

    // Optimize.
    int status = knitro_context.solve();

    // https://www.artelys.com/docs/knitro/3_referenceManual/returnCodes.html#returncodes
    if (!knitro_context.has_mip_incumbent()) {
        algorithm_formatter.update_bound(instance.bound(), "");
    } else if (status == KN_RC_OPTIMAL_OR_SATISFACTORY) {
        double obj = knitro_context.get_mip_incumbent_obj();
        if (!output.solution.feasible()
                || output.solution.cost() > obj + 0.5) {
            Solution solution(instance);
            for (ItemIdx item_id = 0;
                    item_id < instance.number_of_items();
                    ++item_id) {
                for (AgentIdx agent_id = 0;
                        agent_id < instance.number_of_agents();
                        ++agent_id) {
                    double value = knitro_context.get_var_primal_value(
                            x[item_id][agent_id]);
                    if (value > 0.5)
                        solution.set(item_id, agent_id);
                }
            }
            algorithm_formatter.update_solution(solution, "");
        }
        algorithm_formatter.update_bound(output.solution.cost(), "");
    } else if (status == 0) {
        double obj = knitro_context.get_mip_incumbent_obj();
        if (!output.solution.feasible()
                || output.solution.cost() > obj + 0.5) {
            Solution solution(instance);
            for (ItemIdx item_id = 0;
                    item_id < instance.number_of_items();
                    ++item_id) {
                for (AgentIdx agent_id = 0;
                        agent_id < instance.number_of_agents();
                        ++agent_id) {
                    double value = knitro_context.get_var_primal_value(
                            x[item_id][agent_id]);
                    if (value > 0.5)
                        solution.set(item_id, agent_id);
                }
            }
            algorithm_formatter.update_solution(solution, "");
        }
        double bound = knitro_context.get_mip_relaxation_bnd();
        Cost lb = std::ceil(bound - FFOT_TOL);
        algorithm_formatter.update_bound(lb, "");
    } else {
        double bound = knitro_context.get_mip_relaxation_bnd();
        Cost lb = std::ceil(bound - FFOT_TOL);
        algorithm_formatter.update_bound(lb, "");
    }

    algorithm_formatter.end();
    return output;
}

#endif
