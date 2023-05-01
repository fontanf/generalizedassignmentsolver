#if LOCALSOLVER_FOUND

#include "generalizedassignmentsolver/algorithms/localsolver.hpp"

#include <localsolver.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <numeric>
#include <cmath>

using namespace generalizedassignmentsolver;
using namespace localsolver;

class MyCallback: public LSCallback
{

public:

    MyCallback(
            Output& output,
            LocalSolverOptionalParameters& parameters,
            std::vector<LSExpression>& agents):
        output_(output), parameters_(parameters), agents_(agents) { }

    void callback(LocalSolver& ls, LSCallbackType type) {
        (void)type;
        LSSolutionStatus s = ls.getSolution().getStatus();
        if (s == SS_Infeasible)
            return;

        const Instance& instance = output_.solution.instance();
        LSExpression obj = ls.getModel().getObjective(0);
        if(!output_.solution.feasible() || output_.solution.cost() > obj.getValue() + 0.5) {
            Solution sol_curr(output_.solution.instance());
            for (AgentIdx agent_id = 0;
                    agent_id < instance.number_of_agents();
                    ++agent_id) {
                LSCollection bin_collection = agents_[agent_id].getCollectionValue();
                for (ItemIdx item_id_pos = 0;
                        item_id_pos < bin_collection.count();
                        ++item_id_pos) {
                    sol_curr.set(bin_collection[item_id_pos], agent_id);
                }
            }
            std::stringstream ss;
            output_.update_solution(
                    sol_curr,
                    std::stringstream(""),
                    parameters_.info);
        }
    }

private:

    Output& output_;
    LocalSolverOptionalParameters& parameters_;
    std::vector<LSExpression>& agents_;

};

Output generalizedassignmentsolver::localsolver(
        const Instance& instance,
        LocalSolverOptionalParameters parameters)
{
    init_display(instance, parameters.info);
    parameters.info.os()
            << "Algorithm" << std::endl
            << "---------" << std::endl
            << "Local Solver" << std::endl
            << std::endl;

    Output output(instance, parameters.info);

    if (instance.number_of_items() == 0)
        return output.algorithm_end(parameters.info);

    LocalSolver localsolver;

    // Remove display
    localsolver.getParam().setVerbosity(0);

    // Declares the optimization model.
    LSModel model = localsolver.getModel();

    // Weight of each item
    std::vector<std::vector<lsint>> item_weights(n, std::vector<lsint>(n));
    std::vector<std::vector<lsint>> item_costs(n, std::vector<lsint>(n));
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        for (ItemIdx item_id = 0; item_id < number_of_items; ++item_id) {
            item_weights[agent_id][item_id] = instance.alternative(item_id, agent_id).weight;
            item_costs[agent_id][item_id] = instance.alternative(item_id, agent_id).cost;
        }
    }

    // Decision variables
    std::vector<LSExpression> agents(m);

    // Weight of each bin in the solution
    std::vector<LSExpression> agents_weight(m);

    // Profit of each bin in the solution
    std::vector<LSExpression> agents_cost(m);

    LSExpression total_cost; // Objective

    // Set decisions: bins[k] represents the items in bin k
    for (AgentIdx agent_id = 0;
            agent_id < instance.number_of_agents();
            ++agent_id) {
        agents[agent_id] = model.setVar(instance.number_of_items());
    }

    // Each item must be in one bin and one bin only
    model.constraint(model.partition(agents.begin(), agents.end()));

    // Create an array and a function to retrieve the item's weight
    std::vector<LSExpression> weight_array(instance.number_of_agents());
    std::vector<LSExpression> cost_array(instance.number_of_agents());
    std::vector<LSExpression> weight_selector(instance.number_of_agents());
    std::vector<LSExpression> cost_selector(instance.number_of_agents());
    for (AgentIdx agent_id = 0;
            agent_id < instance.number_of_agents();
            ++agent_id) {
        weight_array[agent_id] = model.array(
                item_weights[agent_id].begin(),
                item_weights[agent_id].end());
        weight_selector[agent_id] = model.createFunction([&](LSExpression item_id) {
                return weight_array[agent_id][item_id]; });

        cost_array[agent_id] = model.array(
                item_costs[agent_id].begin(),
                item_costs[agent_id].end());
        cost_selector[agent_id] = model.createFunction([&](LSExpression item_id) {
                return cost_array[agent_id][item_id]; });
    }

    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        // Weight constraint for each bin
        agents_weight[agent_id] = model.sum(
                agents[agent_id],
                weight_selector[agent_id]);
        model.constraint(agents_weight[agent_id] <= (lsint)instance.capacity(i));

        agents_cost[agent_id] = model.sum(
                agents[agent_id],
                cost_selector[agent_id]);
    }

    // Count the used bins
    total_cost = model.sum(agents_cost.begin(), agents_cost.end());

    // Minimize the number of used bins
    model.minimize(total_cost);

    model.close();

    // Time limit
    if (parameters.info.time_limit != std::numeric_limits<double>::infinity())
        localsolver.getParam().setTimeLimit(parameters.info.time_limit);

    // Custom callback
    MyCallback cb(output, parameters, agents);
    localsolver.addCallback(CT_TimeTicked, &cb);

    // Solve
    localsolver.solve();

    // Retrieve solution
    Solution sol_curr(instance);
    for (AgentIdx agent_id = 0; agent_id < number_of_agents; ++agent_id) {
        LSCollection bin_collection = agents[agent_id].getCollectionValue();
        for (ItemIdx item_id_pos = 0;
                item_id_pos < bin_collection.count();
                ++item_id_pos) {
            sol_curr.set(bin_collection[item_id_pos], agent_id);
        }
    }
    output.update_solution(sol_curr, std::stringstream(""), parameters.info);

    return output.algorithm_end(parameters.info);
}

#endif

