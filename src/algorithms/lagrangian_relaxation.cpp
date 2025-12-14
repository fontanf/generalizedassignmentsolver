#include "generalizedassignmentsolver/algorithms/lagrangian_relaxation.hpp"

#include "generalizedassignmentsolver/algorithm_formatter.hpp"

#include "mathoptsolverscmake/box_constrained_nlp.hpp"

#include "knapsacksolver/instance_builder.hpp"
#include "knapsacksolver/algorithms/dynamic_programming_primal_dual.hpp"
//#include "knapsacksolver/algorithms/dynamic_programming_bellman.hpp"

using namespace generalizedassignmentsolver;

////////////////////////////////////////////////////////////////////////////////
///////////////////////// lagrangian_relaxation_assignment /////////////////////
////////////////////////////////////////////////////////////////////////////////

const LagrangianRelaxationAssignmentOutput generalizedassignmentsolver::lagrangian_relaxation_assignment(
        const Instance& instance,
        std::vector<double>* initial_multipliers,
        std::vector<std::vector<int>>* fixed_alt,
        const LagrangianRelaxationAssignmentParameters& parameters)
{
    LagrangianRelaxationAssignmentOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Lagrangian relaxation - assignment constraints");
    algorithm_formatter.print_header();

    mathoptsolverscmake::BoxConstrainedNlpModel model;

    // Compute knapsack capacities
    std::vector<Weight> kp_capacities(instance.number_of_agents());
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
        kp_capacities[agent_id] = instance.capacity(agent_id);
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            if (fixed_alt != NULL
                    && (*fixed_alt)[item_id][agent_id] == 1) {
                kp_capacities[agent_id] -= instance.weight(item_id, agent_id);
            }
        }
        if (kp_capacities[agent_id] < 0) {
            std::cout << "ERROR agent_id " << agent_id << " c " << kp_capacities[agent_id] << std::endl;
        }
    }

    model.objective_direction = mathoptsolverscmake::ObjectiveDirection::Maximize;
    model.objective_function = [&instance, &fixed_alt, &kp_capacities](
            const std::vector<double>& multipliers)
    {
        mathoptsolverscmake::BoxConstrainedNlpFunctionOutput output;

        // Initialize bound and gradient;
        output.objective_value = 0;
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            output.objective_value += multipliers[item_id];
        output.gradient = std::vector<double>(instance.number_of_items(), 1);

        for (AgentIdx agent_id = 0;
                agent_id < instance.number_of_agents();
                ++agent_id) {
            // Create knapsack instance
            knapsacksolver::InstanceFromFloatProfitsBuilder kp_instancebuilder;
            kp_instancebuilder.set_capacity(kp_capacities[agent_id]);
            std::vector<ItemIdx> kp_to_gap;
            for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
                if ((fixed_alt != NULL
                            && (*fixed_alt)[item_id][agent_id] >= 0)) {
                    continue;
                }
                if (instance.weight(item_id, agent_id) > kp_capacities[agent_id])
                    continue;
                double profit = multipliers[item_id] - instance.cost(item_id, agent_id);
                if (profit <= 0)
                    continue;
                kp_instancebuilder.add_item(profit, instance.weight(item_id, agent_id));
                kp_to_gap.push_back(item_id);
            }
            knapsacksolver::Instance kp_instance = kp_instancebuilder.build();

            // Solve knapsack instance
            knapsacksolver::DynamicProgrammingPrimalDualParameters kp_parameters;
            kp_parameters.verbosity_level = 0;
            //auto kp_output = knapsacksolver::dynamic_programming_bellman_array_all(kp_instance, kp_parameters);
            auto kp_output = knapsacksolver::dynamic_programming_primal_dual(
                    kp_instance,
                    kp_parameters);
            //std::cout << "i " << i << " opt " << kp_output.solution.profit() << std::endl;

            // Update bound and gradient
            for (knapsacksolver::ItemId kp_item_id = 0;
                    kp_item_id < kp_instance.number_of_items();
                    ++kp_item_id) {
                if (!kp_output.solution.contains(kp_item_id))
                    continue;
                ItemIdx item_id = kp_to_gap[kp_item_id];
                output.gradient[item_id]--;
                output.objective_value += instance.cost(item_id, agent_id) - multipliers[item_id];
            }
        }

        return output;
    };

    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        model.variables_lower_bounds.push_back(-std::numeric_limits<double>::infinity());
        model.variables_upper_bounds.push_back(+std::numeric_limits<double>::infinity());
    }

    // Initialize multipliers
    if (initial_multipliers != NULL) {
        model.variables_initial_values = *initial_multipliers;
    } else {
        model.variables_initial_values = std::vector<double>(model.number_of_variables(), 0);
    }

    // Solve.
    double bcnlp_bound = 0;
#if DLIB_FOUND
    if (parameters.solver == mathoptsolverscmake::SolverName::Dlib) {
        mathoptsolverscmake::BoxConstrainedNlpDlibOutput dlib_output = mathoptsolverscmake::solve_dlib(model);
        bcnlp_bound = dlib_output.objective_value;
        output.multipliers = dlib_output.solution;
    }
#endif
#if CONICBUNDLE_FOUND
    if (parameters.solver == mathoptsolverscmake::SolverName::ConicBundle) {
        mathoptsolverscmake::BoxConstrainedNlpConicBundleOutput conicbundle_output = mathoptsolverscmake::solve_conicbundle(model);
        bcnlp_bound = conicbundle_output.objective_value;
        output.multipliers = conicbundle_output.solution;
    }
#endif

    // Fill output.
    algorithm_formatter.update_bound(std::ceil(bcnlp_bound - FFOT_TOL), "");

    algorithm_formatter.end();
    return output;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////// lagrangian_relaxation_knapsack //////////////////////
////////////////////////////////////////////////////////////////////////////////

const LagrangianRelaxationKnapsackOutput generalizedassignmentsolver::lagrangian_relaxation_knapsack(
        const Instance& instance,
        std::vector<double>* initial_multipliers,
        std::vector<std::vector<int>>* fixed_alt,
        const LagrangianRelaxationKnapsackParameters& parameters)
{
    LagrangianRelaxationKnapsackOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Lagrangian relaxation - knapsack constraints");
    algorithm_formatter.print_header();

    mathoptsolverscmake::BoxConstrainedNlpModel model;

    model.objective_direction = mathoptsolverscmake::ObjectiveDirection::Maximize;
    model.objective_function = [&instance, &fixed_alt](
            const std::vector<double>& multipliers)
    {
        mathoptsolverscmake::BoxConstrainedNlpFunctionOutput output;

        // Initialize bound and gradient
        output.objective_value = 0;
        output.gradient = std::vector<double>(instance.number_of_agents(), 0);
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
            output.objective_value -= multipliers[agent_id] * instance.capacity(agent_id);
            output.gradient[agent_id] = -instance.capacity(agent_id);
        }

        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            // Solve the trivial Generalized Upper Bound Problem
            AgentIdx agent_id_best = -1;
            double rc_best = -1;
            for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
                double rc = instance.cost(item_id, agent_id) + multipliers[agent_id] * instance.weight(item_id, agent_id);
                if (agent_id_best == -1
                        || rc_best > rc
                        // If the minimum reduced cost of a job is reached for
                        // several agents, schedule the job on the agent with the
                        // most available remaining capacity.
                        // Without this condition, the relaxation fails to get the
                        // optimal bound (the one from the linear relaxation) for
                        // some instances.
                        || (rc_best == rc && output.gradient[agent_id] > output.gradient[agent_id_best])) {
                    agent_id_best = agent_id;
                    rc_best = rc;
                }
            }

            // Update bound and gradient
            output.gradient[agent_id_best] += instance.weight(item_id, agent_id_best);
            //x_(item_id) = agent_id_best;
            output.objective_value += rc_best;
        }
        return output;
    };

    for (AgentIdx agent_id = 0;
            agent_id < instance.number_of_agents();
            ++agent_id) {
        model.variables_lower_bounds.push_back(0);
        model.variables_upper_bounds.push_back(+std::numeric_limits<double>::infinity());
    }

    // Initialize multipliers
    if (initial_multipliers != NULL) {
        model.variables_initial_values = *initial_multipliers;
    } else {
        model.variables_initial_values = std::vector<double>(model.number_of_variables(), 0);
    }

    // Solve.
    double bcnlp_bound = 0;
#if DLIB_FOUND
    if (parameters.solver == mathoptsolverscmake::SolverName::Dlib) {
        mathoptsolverscmake::BoxConstrainedNlpDlibOutput dlib_output = mathoptsolverscmake::solve_dlib(model);
        bcnlp_bound = dlib_output.objective_value;
        output.multipliers = dlib_output.solution;
    }
#endif
#if CONICBUNDLE_FOUND
    if (parameters.solver == mathoptsolverscmake::SolverName::ConicBundle) {
        mathoptsolverscmake::BoxConstrainedNlpConicBundleOutput conicbundle_output = mathoptsolverscmake::solve_conicbundle(model);
        bcnlp_bound = conicbundle_output.objective_value;
        output.multipliers = conicbundle_output.solution;
    }
#endif

    // Fill output.
    algorithm_formatter.update_bound(std::ceil(bcnlp_bound - FFOT_TOL), "");

    algorithm_formatter.end();
    return output;
}
