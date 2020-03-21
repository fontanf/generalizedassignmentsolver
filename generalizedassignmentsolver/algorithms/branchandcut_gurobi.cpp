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
            BranchAndCutGurobiOptionalParameters& p,
            BranchAndCutGurobiOutput& output,
            GRBVar* x):
        instance_(instance), p_(p), output_(output), x_(x) { }

protected:

    void callback()
    {
        if (where != GRB_CB_MIPSOL)
            return;

        Cost lb = std::ceil(getDoubleInfo(GRB_CB_MIPSOL_OBJBND) - TOL);
        output_.update_lower_bound(lb, std::stringstream(""), p_.info);

        if (!output_.solution.feasible() || output_.solution.cost() > getDoubleInfo(GRB_CB_MIPSOL_OBJ) + 0.5) {
            Solution sol_curr(instance_);
            AltIdx o = instance_.alternative_number();
            double* x = getSolution(x_, o);
            for (AltIdx k = 0; k < o; ++k)
                if (x[k] > 0.5)
                    sol_curr.set(k);
            std::stringstream ss;
            output_.update_solution(sol_curr, std::stringstream(""), p_.info);
        }
    }

private:

    const Instance& instance_;
    BranchAndCutGurobiOptionalParameters& p_;
    BranchAndCutGurobiOutput& output_;
    GRBVar* x_;

};

BranchAndCutGurobiOutput generalizedassignmentsolver::branchandcut_gurobi(
        const Instance& instance, BranchAndCutGurobiOptionalParameters p)
{
    GRBEnv env;
    VER(p.info, "*** branchandcut_gurobi ***" << std::endl);

    BranchAndCutGurobiOutput output(instance, p.info);

    ItemIdx n = instance.item_number();
    AgentIdx m = instance.agent_number();
    AltIdx o = instance.alternative_number();

    if (n == 0)
        return output.algorithm_end(p.info);

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
    if (p.initial_solution != NULL && p.initial_solution->feasible())
        for (ItemIdx j = 0; j < n; ++j)
            for (AgentIdx i = 0; i < m; ++i)
                x[instance.alternative_index(j, i)].set(GRB_DoubleAttr_Start, (p.initial_solution->agent(j) == i)? 1: 0);

    // Redirect standard output to log file
    model.set(GRB_StringParam_LogFile, "gurobi.log");
    model.set(GRB_IntParam_LogToConsole, 0);

    model.set(GRB_DoubleParam_MIPGap, 0); // Fix precision issue
    model.set(GRB_DoubleParam_NodefileStart, 0.5); // Avoid running out of memory

    // Time limit
    if (p.info.timelimit != std::numeric_limits<double>::infinity())
        model.set(GRB_DoubleParam_TimeLimit, p.info.timelimit);

    // Callback
    BranchAndCutGurobiCallback cb = BranchAndCutGurobiCallback(instance, p, output, x);
    model.setCallback(&cb);

    // Optimize
    model.optimize();

    int optimstatus = model.get(GRB_IntAttr_Status);
    if (optimstatus == GRB_INFEASIBLE) {
        output.update_lower_bound(instance.bound(), std::stringstream(""), p.info);
    } else if (optimstatus == GRB_OPTIMAL) {
        if (!output.solution.feasible() || output.solution.cost() > model.get(GRB_DoubleAttr_ObjVal) + 0.5) {
            Solution sol_curr(instance);
            for (AltIdx k = 0; k < o; ++k)
                if (x[k].get(GRB_DoubleAttr_X) > 0.5)
                    sol_curr.set(k);
            output.update_solution(sol_curr, std::stringstream(""), p.info);
        }
        output.update_lower_bound(output.solution.cost(), std::stringstream(""), p.info);
    } else if (model.get(GRB_IntAttr_SolCount) > 0) {
        if (!output.solution.feasible() || output.solution.cost() > model.get(GRB_DoubleAttr_ObjVal) + 0.5) {
            Solution sol_curr(instance);
            for (AltIdx k = 0; k < o; ++k)
                if (x[k].get(GRB_DoubleAttr_X) > 0.5)
                    sol_curr.set(k);
            output.update_solution(sol_curr, std::stringstream(""), p.info);
        }
        Cost lb = std::ceil(model.get(GRB_DoubleAttr_ObjBound) - TOL);
        output.update_lower_bound(lb, std::stringstream(""), p.info);
    } else {
        Cost lb = std::ceil(model.get(GRB_DoubleAttr_ObjBound) - TOL);
        output.update_lower_bound(lb, std::stringstream(""), p.info);
    }

    return output.algorithm_end(p.info);
}

#endif

