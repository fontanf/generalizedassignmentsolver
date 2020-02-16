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

LocalSolverOutput& LocalSolverOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    //PUT(info, "Algorithm", "Iterations", it);
    //VER(info, "Iterations: " << it << std::endl);
    return *this;
}

class MyCallback: public LSCallback
{

public:

    MyCallback(LocalSolverOutput& output, LocalSolverOptionalParameters& p, std::vector<LSExpression>& agents):
        output_(output), p_(p), agents_(agents) { }

    void callback(LocalSolver& ls, LSCallbackType type) {
        (void)type;
        LSSolutionStatus s = ls.getSolution().getStatus();
        if (s == SS_Infeasible)
            return;

        LSExpression obj = ls.getModel().getObjective(0);
        if(!output_.solution.feasible() || output_.solution.cost() > obj.getValue() + 0.5) {
            Solution sol_curr(output_.solution.instance());
            AgentIdx m = output_.solution.instance().agent_number();
            for (AgentIdx i=0; i<m; ++i) {
                LSCollection bin_collection = agents_[i].getCollectionValue();
                for (ItemIdx j_pos=0; j_pos<bin_collection.count(); ++j_pos) {
                    sol_curr.set(bin_collection[j_pos], i);
                }
            }
            std::stringstream ss;
            output_.update_solution(sol_curr, std::stringstream(""), p_.info);
        }
    }

private:

    LocalSolverOutput& output_;
    LocalSolverOptionalParameters& p_;
    std::vector<LSExpression>& agents_;

};

LocalSolverOutput generalizedassignmentsolver::localsolver(const Instance& ins, LocalSolverOptionalParameters p)
{
    VER(p.info, "*** localsolver ***" << std::endl);
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();

    LocalSolverOutput output(ins, p.info);

    if (n == 0)
        return output.algorithm_end(p.info);

    LocalSolver localsolver;

    // Remove display
    localsolver.getParam().setVerbosity(0);

    // Declares the optimization model.
    LSModel model = localsolver.getModel();

    // Weight of each item
    std::vector<std::vector<lsint>> item_weights(n, std::vector<lsint>(n));
    std::vector<std::vector<lsint>> item_costs(n, std::vector<lsint>(n));
    for (AgentIdx i=0; i<m; ++i) {
        for (ItemIdx j=0; j<n; ++j) {
            item_weights[i][j] = ins.alternative(j, i).w;
            item_costs[i][j] = ins.alternative(j, i).c;
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
    for (AgentIdx i=0; i<ins.agent_number(); ++i)
        agents[i] = model.setVar(ins.item_number());

    // Each item must be in one bin and one bin only
    model.constraint(model.partition(agents.begin(), agents.end()));

    // Create an array and a function to retrieve the item's weight
    std::vector<LSExpression> weight_array(m);
    std::vector<LSExpression> cost_array(m);
    std::vector<LSExpression> weight_selector(m);
    std::vector<LSExpression> cost_selector(m);
    for (AgentIdx i=0; i<m; ++i) {
        weight_array[i] = model.array(item_weights[i].begin(), item_weights[i].end());
        weight_selector[i] = model.createFunction([&](LSExpression j) {
                return weight_array[i][j]; });
        cost_array[i] = model.array(item_costs[i].begin(), item_costs[i].end());
        cost_selector[i] = model.createFunction([&](LSExpression j) {
                return cost_array[i][j]; });
    }

    for (AgentIdx i=0; i<m; ++i) {
        // Weight constraint for each bin
        agents_weight[i] = model.sum(agents[i], weight_selector[i]);
        model.constraint(agents_weight[i] <= (lsint)ins.capacity(i));
        agents_cost[i] = model.sum(agents[i], cost_selector[i]);
    }

    // Count the used bins
    total_cost = model.sum(agents_cost.begin(), agents_cost.end());

    // Minimize the number of used bins
    model.minimize(total_cost);

    model.close();

    // Time limit
    if (p.info.timelimit != std::numeric_limits<double>::infinity())
        localsolver.getParam().setTimeLimit(p.info.timelimit);

    // Custom callback
    MyCallback cb(output, p, agents);
    localsolver.addCallback(CT_TimeTicked, &cb);

    // Solve
    localsolver.solve();

    // Retrieve solution
    Solution sol_curr(ins);
    for (AgentIdx i=0; i<m; ++i) {
        LSCollection bin_collection = agents[i].getCollectionValue();
        for (ItemIdx j_pos=0; j_pos<bin_collection.count(); ++j_pos)
            sol_curr.set(bin_collection[j_pos], i);
    }
    output.update_solution(sol_curr, std::stringstream(""), p.info);

    return output.algorithm_end(p.info);
}

#endif

