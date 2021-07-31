#if GUROBI_FOUND

#include "generalizedassignmentsolver/algorithms/milp_gurobi.hpp"

#include "gurobi_c++.h"

/*
 * https://stackoverflow.com/questions/46779850/cannot-compile-gurobi-examples-in-version-7-5-1
 */

using namespace generalizedassignmentsolver;

MilpGurobiOutput& MilpGurobiOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

class MilpGurobiCallback: public GRBCallback
{

public:

    MilpGurobiCallback(
            const Instance& instance,
            MilpGurobiOptionalParameters& parameters,
            MilpGurobiOutput& output,
            std::vector<GRBVar*>& x):
        instance_(instance), parameters_(parameters), output_(output), x_(x) { }

protected:

    void callback()
    {
        if (where != GRB_CB_MIPSOL)
            return;

        Cost lb = std::ceil(getDoubleInfo(GRB_CB_MIPSOL_OBJBND) - TOL);
        output_.update_lower_bound(lb, std::stringstream(""), parameters_.info);

        if (!output_.solution.feasible() || output_.solution.cost() > getDoubleInfo(GRB_CB_MIPSOL_OBJ) + 0.5) {
            Solution solution(instance_);
            AgentIdx m = instance_.number_of_agents();
            ItemIdx  n = instance_.number_of_items();
            for (ItemIdx j = 0; j < n; ++j)
                for (AgentIdx i = 0; i < m; ++i)
                    if (getSolution(x_[j][i]) > 0.5)
                        solution.set(j, i);
            std::stringstream ss;
            output_.update_solution(solution, std::stringstream(""), parameters_.info);
        }
    }

private:

    const Instance& instance_;
    MilpGurobiOptionalParameters& parameters_;
    MilpGurobiOutput& output_;
    std::vector<GRBVar*>& x_;

};

MilpGurobiOutput generalizedassignmentsolver::milp_gurobi(
        const Instance& instance, MilpGurobiOptionalParameters parameters)
{
    GRBEnv env;
    VER(parameters.info, "*** milp_gurobi ***" << std::endl);

    MilpGurobiOutput output(instance, parameters.info);

    ItemIdx  n = instance.number_of_items();
    AgentIdx m = instance.number_of_agents();

    if (n == 0)
        return output.algorithm_end(parameters.info);

    GRBModel model(env);

    // Variables
    std::vector<GRBVar*> x;
    for (ItemIdx j = 0; j < n; j++)
        x.push_back(model.addVars(m, GRB_BINARY));

    // Objective
    for (ItemIdx j = 0; j < n; j++)
        for (AgentIdx i = 0; i < m; i++)
            x[j][i].set(GRB_DoubleAttr_Obj, instance.cost(j, i));
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    // Capacity constraints
    for (AgentIdx i = 0; i < m; i++) {
        GRBLinExpr expr;
        for (ItemIdx j = 0; j < n; j++)
            expr += instance.weight(j, i) * x[j][i];
        model.addConstr(expr <= instance.capacity(i));
    }

    // One alternative per item constraint
    for (ItemIdx j = 0; j < n; j++) {
        GRBLinExpr expr;
        for (AgentIdx i = 0; i < m; i++)
            expr += x[j][i];
        model.addConstr(expr == 1);
    }

    // Redirect standard output to log file
    model.set(GRB_StringParam_LogFile, "gurobi.log");
    model.set(GRB_IntParam_LogToConsole, 0);

    if (parameters.only_linear_relaxation) {
        for (ItemIdx j = 0; j < n; j++)
            for (AgentIdx i = 0; i < m; i++)
                x[j][i].set(GRB_CharAttr_VType, GRB_CONTINUOUS);
        model.optimize();
        Cost lb = std::ceil(model.get(GRB_DoubleAttr_ObjVal) - TOL);
        output.update_lower_bound(lb, std::stringstream("linearrelaxation"), parameters.info);
        for (ItemIdx j = 0; j < n; j++) {
            output.x.push_back(std::vector<double>(m));
            for (AgentIdx i = 0; i < m; i++)
                output.x[j][i] = x[j][i].get(GRB_DoubleAttr_X);
        }
        return output.algorithm_end(parameters.info);
    }

    // Initial solution
    if (parameters.initial_solution != NULL && parameters.initial_solution->feasible())
        for (ItemIdx j = 0; j < n; ++j)
            for (AgentIdx i = 0; i < m; ++i)
                x[j][i].set(GRB_DoubleAttr_Start, (parameters.initial_solution->agent(j) == i)? 1: 0);

    model.set(GRB_DoubleParam_MIPGap, 0); // Fix precision issue
    model.set(GRB_DoubleParam_NodefileStart, 0.5); // Avoid running out of memory

    // Time limit
    if (parameters.info.time_limit != std::numeric_limits<double>::infinity())
        model.set(GRB_DoubleParam_TimeLimit, parameters.info.time_limit);

    // Callback
    MilpGurobiCallback cb = MilpGurobiCallback(instance, parameters, output, x);
    model.setCallback(&cb);

    // Optimize
    model.optimize();

    int optimstatus = model.get(GRB_IntAttr_Status);
    if (optimstatus == GRB_INFEASIBLE) {
        output.update_lower_bound(instance.bound(), std::stringstream(""), parameters.info);
    } else if (optimstatus == GRB_OPTIMAL) {
        if (!output.solution.feasible() || output.solution.cost() > model.get(GRB_DoubleAttr_ObjVal) + 0.5) {
            Solution solution(instance);
            for (ItemIdx j = 0; j < n; ++j)
                for (AgentIdx i = 0; i < m; ++i)
                    if (x[j][i].get(GRB_DoubleAttr_X) > 0.5)
                        solution.set(j, i);
            output.update_solution(solution, std::stringstream(""), parameters.info);
        }
        output.update_lower_bound(output.solution.cost(), std::stringstream(""), parameters.info);
    } else if (model.get(GRB_IntAttr_SolCount) > 0) {
        if (!output.solution.feasible() || output.solution.cost() > model.get(GRB_DoubleAttr_ObjVal) + 0.5) {
            Solution solution(instance);
            for (ItemIdx j = 0; j < n; ++j)
                for (AgentIdx i = 0; i < m; ++i)
                    if (x[j][i].get(GRB_DoubleAttr_X) > 0.5)
                        solution.set(j, i);
            output.update_solution(solution, std::stringstream(""), parameters.info);
        }
        Cost lb = std::ceil(model.get(GRB_DoubleAttr_ObjBound) - TOL);
        output.update_lower_bound(lb, std::stringstream(""), parameters.info);
    } else {
        Cost lb = std::ceil(model.get(GRB_DoubleAttr_ObjBound) - TOL);
        output.update_lower_bound(lb, std::stringstream(""), parameters.info);
    }

    return output.algorithm_end(parameters.info);
}

#endif

