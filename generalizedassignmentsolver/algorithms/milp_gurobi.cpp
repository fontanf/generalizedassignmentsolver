#if GUROBI_FOUND

#include "generalizedassignmentsolver/algorithms/milp_gurobi.hpp"

#include "gurobi_c++.h"

/*
 * https://stackoverflow.com/questions/46779850/cannot-compile-gurobi-examples-in-version-7-5-1
 */

using namespace generalizedassignmentsolver;

class MilpGurobiCallback: public GRBCallback
{

public:

    MilpGurobiCallback(
            const Instance& instance,
            MilpGurobiParameters& parameters,
            MilpGurobiOutput& output,
            std::vector<GRBVar*>& x):
        instance_(instance), parameters_(parameters), output_(output), x_(x) { }

protected:

    void callback()
    {
        if (where != GRB_CB_MIPSOL)
            return;

        Cost lb = std::ceil(getDoubleInfo(GRB_CB_MIPSOL_OBJBND) - FFOT_TOL);
        algorithm_formatter_.update_bound(lb, "");

        if (!output_.solution.feasible()
                || output_.solution.cost() > getDoubleInfo(GRB_CB_MIPSOL_OBJ) + 0.5) {
            Solution solution(instance_);
            for (ItemIdx item_id = 0;
                    item_id < instance_.number_of_items();
                    ++item_id) {
                for (AgentIdx agent_id = 0;
                        agent_id < instance_.number_of_agents();
                        ++agent_id) {
                    if (getSolution(x_[item_id][agent_id]) > 0.5)
                        solution.set(item_id, agent_id);
                }
            }
            std::stringstream ss;
            algorithm_formatter_.update_solution(solution, "");
        }
    }

private:

    const Instance& instance_;
    MilpGurobiParameters& parameters_;
    MilpGurobiOutput& output_;
    std::vector<GRBVar*>& x_;

};

const MilpGurobiOutput generalizedassignmentsolver::milp_gurobi(
        const Instance& instance,
        const MilpGurobiParameters& parameters)
{
    MilpGurobiOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("MILP (Gurobi)");
    algorithm_formatter.print_header();

    GRBEnv env;

    if (instance.number_of_items() == 0) {
        algorithm_formatter.end();
        return output;
    }

    GRBModel model(env);

    // Variables
    std::vector<GRBVar*> x;
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        x.push_back(model.addVars(instance.number_of_agents(), GRB_BINARY));
    }

    // Objective
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        for (AgentIdx agent_id = 0;
                agent_id < instance.number_of_agents();
                ++agent_id) {
            x[item_id][agent_id].set(
                    GRB_DoubleAttr_Obj,
                    instance.cost(item_id, agent_id));
        }
    }
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    // Capacity constraints
    for (AgentIdx agent_id = 0;
            agent_id < instance.number_of_agents();
            ++agent_id) {
        GRBLinExpr expr;
        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                ++item_id)
            expr += instance.weight(item_id, agent_id) * x[item_id][agent_id];
        model.addConstr(expr <= instance.capacity(agent_id));
    }

    // One alternative per item constraint
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        GRBLinExpr expr;
        for (AgentIdx agent_id = 0;
                agent_id < instance.number_of_agents();
                ++agent_id)
            expr += x[item_id][agent_id];
        model.addConstr(expr == 1);
    }

    // Redirect standard output to log file
    model.set(GRB_StringParam_LogFile, "gurobi.log");
    model.set(GRB_IntParam_LogToConsole, 0);

    if (parameters.only_linear_relaxation) {
        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            for (AgentIdx agent_id = 0;
                    agent_id < instance.number_of_agents();
                    ++agent_id) {
                x[item_id][agent_id].set(
                        GRB_CharAttr_VType,
                        GRB_CONTINUOUS);
            }
        }
        model.optimize();
        Cost lb = std::ceil(model.get(GRB_DoubleAttr_ObjVal) - FFOT_TOL);
        algorithm_formatter.update_bound(lb, "linearrelaxation");

        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            output.x.push_back(std::vector<double>(instance.number_of_agents()));
            for (AgentIdx agent_id = 0;
                    agent_id < instance.number_of_agents();
                    ++agent_id)
                output.x[item_id][agent_id] = x[item_id][agent_id].get(GRB_DoubleAttr_X);
        }

        algorithm_formatter.end();
        return output;
    }

    // Initial solution
    if (parameters.initial_solution != NULL
            && parameters.initial_solution->feasible()) {
        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            for (AgentIdx agent_id = 0;
                    agent_id < instance.number_of_agents();
                    ++agent_id) {
                x[item_id][agent_id].set(
                        GRB_DoubleAttr_Start,
                        (parameters.initial_solution->agent(item_id) == agent_id)? 1: 0);
            }
        }
    }

    model.set(GRB_DoubleParam_MIPGap, 0); // Fix precision issue
    model.set(GRB_DoubleParam_NodefileStart, 0.5); // Avoid running out of memory

    // Time limit
    if (parameters.timer.time_limit() != std::numeric_limits<double>::infinity())
        model.set(GRB_DoubleParam_TimeLimit, parameters.timer.time_limit());

    // Callback
    MilpGurobiCallback cb = MilpGurobiCallback(instance, parameters, output, x);
    model.setCallback(&cb);

    // Optimize
    model.optimize();

    int optimstatus = model.get(GRB_IntAttr_Status);
    if (optimstatus == GRB_INFEASIBLE) {
        algorithm_formatter.update_bound(instance.bound(), "");
    } else if (optimstatus == GRB_OPTIMAL) {
        if (!output.solution.feasible()
                || output.solution.cost() > model.get(GRB_DoubleAttr_ObjVal) + 0.5) {
            Solution solution(instance);
            for (ItemIdx item_id = 0;
                    item_id < instance.number_of_items();
                    ++item_id) {
                for (AgentIdx agent_id = 0;
                        agent_id < instance.number_of_agents();
                        ++agent_id) {
                    if (x[item_id][agent_id].get(GRB_DoubleAttr_X) > 0.5)
                        solution.set(item_id, agent_id);
                }
            }
            algorithm_formatter.update_solution(solution, "");
        }
        algorithm_formatter.update_bound(output.solution.cost(), "");
    } else if (model.get(GRB_IntAttr_SolCount) > 0) {
        if (!output.solution.feasible()
                || output.solution.cost() > model.get(GRB_DoubleAttr_ObjVal) + 0.5) {
            Solution solution(instance);
            for (ItemIdx item_id = 0;
                    item_id < instance.number_of_items();
                    ++item_id)
                for (AgentIdx agent_id = 0;
                        agent_id < instance.number_of_agents();
                        ++agent_id)
                    if (x[item_id][agent_id].get(GRB_DoubleAttr_X) > 0.5)
                        solution.set(item_id, agent_id);
            algorithm_formatter.update_solution(solution, "");
        }
        Cost lb = std::ceil(model.get(GRB_DoubleAttr_ObjBound) - FFOT_TOL);
        algorithm_formatter.update_bound(lb, "");
    } else {
        Cost lb = std::ceil(model.get(GRB_DoubleAttr_ObjBound) - FFOT_TOL);
        algorithm_formatter.update_bound(lb, "");
    }

    algorithm_formatter.end();
    return output;
}

#endif

