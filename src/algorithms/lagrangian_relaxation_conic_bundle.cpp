#include "generalizedassignmentsolver/algorithms/lagrangian_relaxation_conic_bundle.hpp"

#include "generalizedassignmentsolver/algorithm_formatter.hpp"

#include "knapsacksolver/instance_builder.hpp"
#include "knapsacksolver/algorithms/dynamic_programming_primal_dual.hpp"

#include "CBSolver.hxx"

using namespace generalizedassignmentsolver;

////////////////////////////////////////////////////////////////////////////////
////////////////// lagrangian_relaxation_assignment_conic_bundle ///////////////
////////////////////////////////////////////////////////////////////////////////

class LagrangianRelaxationAssignmentConicBundleFunction: public ConicBundle::FunctionOracle
{

public:

    LagrangianRelaxationAssignmentConicBundleFunction(
            const Instance& instance):
        instance_(instance) { }

    int evaluate(
            const double* lagrangian_multipliers,
            double /* relprec */,
            double& objective_value,
            std::vector<ConicBundle::Minorant*>& minorants,
            ConicBundle::PrimalExtender*&)
    {
        objective_value = 0;
        for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id)
            objective_value += lagrangian_multipliers[item_id];
        ConicBundle::DVector subgradient(instance_.number_of_items(),1.);
        for (AgentIdx agent_id = 0;
                agent_id < instance_.number_of_agents();
                ++agent_id) {
            // Create knapsack instance
            knapsacksolver::InstanceFromFloatProfitsBuilder kp_instance_builder;
            kp_instance_builder.set_capacity(instance_.capacity(agent_id));
            std::vector<ItemIdx> kp_to_orig;
            for (ItemIdx item_id = 0;
                    item_id < instance_.number_of_items();
                    ++item_id) {
                Weight weight = instance_.weight(item_id, agent_id);
                if (weight > instance_.capacity(agent_id))
                    continue;
                //std::cout << "agent_id " << agent_id
                //    << " item_id " << item_id
                //    << " multiplier " << lagrangian_multipliers[item_id]
                //    << " cost " << instance_.cost(item_id, agent_id)
                //    << std::endl;
                double profit = lagrangian_multipliers[item_id] - instance_.cost(item_id, agent_id);
                if (profit <= 0)
                    continue;
                kp_instance_builder.add_item(profit, weight);
                kp_to_orig.push_back(item_id);
            }
            knapsacksolver::Instance kp_instance = kp_instance_builder.build();

            // Solve knapsack instance
            knapsacksolver::DynamicProgrammingPrimalDualParameters kp_parameters;
            kp_parameters.verbosity_level = 0;
            auto kp_output = knapsacksolver::dynamic_programming_primal_dual(
                    kp_instance,
                    kp_parameters);

            // Update bound and gradient
            for (knapsacksolver::ItemId kp_item_id = 0;
                    kp_item_id < kp_instance.number_of_items();
                    ++kp_item_id) {
                if (!kp_output.solution.contains(kp_item_id))
                    continue;
                ItemIdx item_id = kp_to_orig[kp_item_id];
                objective_value += instance_.cost(item_id, agent_id) - lagrangian_multipliers[item_id];
                subgradient[item_id]--;
            }
        }

        // ConicBundle minimizes.
        // Here we maximizes, so let's multiply the objective value and
        // subgradientradient by -1.
        objective_value *= -1;
        for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id)
            subgradient[item_id] *= -1;

        minorants.push_back(new ConicBundle::Minorant(objective_value, subgradient));
        return 0;
    }

private:

    const Instance& instance_;

};

const LagrangianRelaxationAssignmentConicBundleOutput generalizedassignmentsolver::lagrangian_relaxation_assignment_conic_bundle(
        const Instance& instance,
        const LagrangianRelaxationAssignmentConicBundleParameters& parameters)
{
    LagrangianRelaxationAssignmentConicBundleOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Lagrangian relaxation - assignment constraints (ConicBundle)");
    algorithm_formatter.print_header();

    LagrangianRelaxationAssignmentConicBundleFunction function(instance);

    // initilialize solver with basic output
    ConicBundle::CBSolver solver(&std::cout, 1);
    solver.init_problem(instance.number_of_items());
    solver.add_function(function);
    // Set relative precision
    solver.set_term_relprec(1e-8);
    // Minimize the function.
    solver.solve();
    solver.print_termination_code(std::cout);
    ConicBundle::DVector lagrangian_multipliers;
    // Retrieve the computed solution
    solver.get_center(lagrangian_multipliers);
    output.multipliers = std::vector<double>(instance.number_of_items(), 0);
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        output.multipliers[item_id] = -lagrangian_multipliers[item_id];

    // Update bound.
    double objective_value = -solver.get_objval();
    algorithm_formatter.update_bound(std::ceil(objective_value - 1e-7), "");

    algorithm_formatter.end();
    return output;
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////// lagrangian_relaxation_knapsack_conic_bundle ////////////////////
////////////////////////////////////////////////////////////////////////////////

/*
class LagrangianRelaxationKnapsackConicBundleFunction
{

public:

    LagrangianRelaxationKnapsackConicBundleFunction(const Instance& instance):
        instance_(instance),
        x_(instance.number_of_items()),
        grad_(instance.number_of_agents())
    {  }
    virtual ~LagrangianRelaxationKnapsackConicBundleFunction() { };

    double f(const column_vector& x);

    const column_vector der(const column_vector& x) const { (void)x; return grad_; }

    AgentIdx agent(ItemIdx item_id) const { return x_(item_id); }

private:

    const Instance& instance_;
    column_vector x_;
    column_vector grad_;

};

double LagrangianRelaxationKnapsackConicBundleFunction::f(const column_vector& mu)
{
    // Initialize bound and gradient
    double l = 0;
    for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id) {
        l += mu(agent_id) * instance_.capacity(agent_id);
        grad_(agent_id) = instance_.capacity(agent_id);
    }

    for (ItemIdx item_id = 0; item_id < instance_.number_of_items(); ++item_id) {
        // Solve the trivial Generalized Upper Bound Problem
        AgentIdx agent_id_best = -1;
        double rc_best = -1;
        for (AgentIdx agent_id = 0; agent_id < instance_.number_of_agents(); ++agent_id) {
            double rc = instance_.cost(item_id, agent_id) - mu(agent_id) * instance_.weight(item_id, agent_id);
            if (agent_id_best == -1
                    || rc_best > rc
                    // If the minimum reduced cost of a job is reached for
                    // several agents, schedule the job on the agent with the
                    // most available remaining capacity.
                    // Without this condition, the relaxation fails to get the
                    // optimal bound (the one from the linear relaxation) for
                    // some instances.
                    || (rc_best == rc && grad_(agent_id) > grad_(agent_id_best))) {
                agent_id_best = agent_id;
                rc_best = rc;
            }
        }

        // Update bound and gradient
        grad_(agent_id_best) -= instance_.weight(item_id, agent_id_best);
        x_(item_id) = agent_id_best;
        l += rc_best;
    }

    return l;
}

const LagrangianRelaxationKnapsackConicBundleOutput generalizedassignmentsolver::lagrangian_relaxation_knapsack_conic_bundle(
        const Instance& instance,
        const Parameters& parameters)
{
    LagrangianRelaxationKnapsackConicBundleOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Lagrangian relaxation - knapsack constraints (conic_bundle)");
    algorithm_formatter.print_header();

    // Initialize multipliers
    column_vector mu(instance.number_of_agents());
    column_vector mu_lower(instance.number_of_agents());
    column_vector mu_upper(instance.number_of_agents());
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
        //mu_lower(agent_id) = 0;
        //mu_upper(agent_id) = std::numeric_limits<double>::max();
        mu(agent_id) = 0;
        mu_lower(agent_id) = -std::numeric_limits<double>::max();
        mu_upper(agent_id) = 0;
    }

    // Solve
    LagrangianRelaxationKnapsackConicBundleFunction func(instance);
    auto f   = [&func](const column_vector& x) { return func.f(x); };
    auto def = [&func](const column_vector& x) { return func.der(x); };
    auto stop_strategy = objective_delta_stop_strategy();
    //auto stop_strategy = gradient_norm_stop_strategy();
    double res = find_max_box_constrained(
            conic_bundle_search_strategy(256),
            stop_strategy,
            f,
            def,
            mu,
            mu_lower,
            mu_upper);

    // Compute output parameters
    Cost lb = std::ceil(res - FFOT_TOL);
    algorithm_formatter.update_bound(lb, "");
    output.multipliers.resize(instance.number_of_agents());
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
        output.multipliers[agent_id] = mu(agent_id);
    func.f(mu);
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        output.x.push_back(std::vector<double>(instance.number_of_agents(), 0));
        output.x[item_id][func.agent(item_id)] = 1;
    }

    algorithm_formatter.end();
    return output;
}
*/
