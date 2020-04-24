#if GUROBI_FOUND

#include "generalizedassignmentsolver/algorithms/branchandcut_gurobi.hpp"

#include "gurobi_c++.h"

/*
 * https://stackoverflow.com/questions/46779850/cannot-compile-gurobi-examples-in-version-7-5-1
 */

using namespace generalizedassignmentsolver;

BranchAndCutGurobiOutput& BranchAndCutGurobiOutput::algorithm_end(Info& info)
{
    Output::algorithm_end(info);
    return *this;
}

class BranchAndCutGurobiCallback: public GRBCallback
{

public:

    BranchAndCutGurobiCallback(
            const Instance& instance,
            BranchAndCutGurobiOptionalParameters& parameters,
            BranchAndCutGurobiOutput& output,
            GRBVar* x):
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
            AltIdx o = instance_.alternative_number();
            for (AltIdx k = 0; k < o; ++k)
                if (getSolution(x_[k]) > 0.5)
                    solution.set(k);
            std::stringstream ss;
            output_.update_solution(solution, std::stringstream(""), parameters_.info);
        }
    }

private:

    const Instance& instance_;
    BranchAndCutGurobiOptionalParameters& parameters_;
    BranchAndCutGurobiOutput& output_;
    GRBVar* x_;

};

BranchAndCutGurobiOutput generalizedassignmentsolver::branchandcut_gurobi(
        const Instance& instance, BranchAndCutGurobiOptionalParameters parameters)
{
    GRBEnv env;
    VER(parameters.info, "*** branchandcut_gurobi ***" << std::endl);

    BranchAndCutGurobiOutput output(instance, parameters.info);

    ItemIdx n = instance.item_number();
    AgentIdx m = instance.agent_number();
    AltIdx o = instance.alternative_number();

    if (n == 0)
        return output.algorithm_end(parameters.info);

    GRBModel model(env);

    // Variables
    GRBVar* x = model.addVars(o, GRB_BINARY);

    // Objective
    for (AltIdx k = 0; k < o; ++k)
        x[k].set(GRB_DoubleAttr_Obj, instance.alternative(k).c);
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    // Capacity constraints
    for (AgentIdx i = 0; i < m; i++) {
        GRBLinExpr expr;
        for (ItemIdx j = 0; j < n; j++)
            expr += x[instance.alternative_index(j, i)] * instance.alternative(j, i).w;
        model.addConstr(expr <= instance.capacity(i));
    }

    // One alternative per item constraint
    for (ItemIdx j = 0; j < n; j++) {
        GRBLinExpr expr;
        for (AgentIdx i = 0; i < m; i++)
            expr += x[instance.alternative_index(j, i)];
        model.addConstr(expr == 1);
    }

    // Initial solution
    if (parameters.initial_solution != NULL && parameters.initial_solution->feasible())
        for (ItemIdx j = 0; j < n; ++j)
            for (AgentIdx i = 0; i < m; ++i)
                x[instance.alternative_index(j, i)].set(GRB_DoubleAttr_Start, (parameters.initial_solution->agent(j) == i)? 1: 0);

    // Redirect standard output to log file
    model.set(GRB_StringParam_LogFile, "gurobi.log");
    model.set(GRB_IntParam_LogToConsole, 0);

    model.set(GRB_DoubleParam_MIPGap, 0); // Fix precision issue
    model.set(GRB_DoubleParam_NodefileStart, 0.5); // Avoid running out of memory

    // Time limit
    if (parameters.info.timelimit != std::numeric_limits<double>::infinity())
        model.set(GRB_DoubleParam_TimeLimit, parameters.info.timelimit);

    // Callback
    BranchAndCutGurobiCallback cb = BranchAndCutGurobiCallback(instance, parameters, output, x);
    model.setCallback(&cb);

    // Optimize
    model.optimize();

    int optimstatus = model.get(GRB_IntAttr_Status);
    if (optimstatus == GRB_INFEASIBLE) {
        output.update_lower_bound(instance.bound(), std::stringstream(""), parameters.info);
    } else if (optimstatus == GRB_OPTIMAL) {
        if (!output.solution.feasible() || output.solution.cost() > model.get(GRB_DoubleAttr_ObjVal) + 0.5) {
            Solution solution(instance);
            for (AltIdx k = 0; k < o; ++k)
                if (x[k].get(GRB_DoubleAttr_X) > 0.5)
                    solution.set(k);
            output.update_solution(solution, std::stringstream(""), parameters.info);
        }
        output.update_lower_bound(output.solution.cost(), std::stringstream(""), parameters.info);
    } else if (model.get(GRB_IntAttr_SolCount) > 0) {
        if (!output.solution.feasible() || output.solution.cost() > model.get(GRB_DoubleAttr_ObjVal) + 0.5) {
            Solution solution(instance);
            for (AltIdx k = 0; k < o; ++k)
                if (x[k].get(GRB_DoubleAttr_X) > 0.5)
                    solution.set(k);
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

