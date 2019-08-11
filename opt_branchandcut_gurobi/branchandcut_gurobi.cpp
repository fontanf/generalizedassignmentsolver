#if GUROBI_FOUND

#include "gap/opt_branchandcut_gurobi/branchandcut_gurobi.hpp"

#include "gurobi_c++.h"

/*
 * https://stackoverflow.com/questions/46779850/cannot-compile-gurobi-examples-in-version-7-5-1
 */

using namespace gap;

class mycallback: public GRBCallback
{

public:

    mycallback(BranchAndCutGurobiData& d, GRBVar* x):
        d_(d), x_(x) { }

protected:

    void callback()
    {
        if (where != GRB_CB_MIPSOL)
            return;

        if (d_.lb < getDoubleInfo(GRB_CB_MIPSOL_OBJBND) - 0.5)
            update_lb(d_.lb, getDoubleInfo(GRB_CB_MIPSOL_OBJBND), d_.sol, std::stringstream(""), d_.info);

        if(!d_.sol.feasible() || d_.sol.cost() > getDoubleInfo(GRB_CB_MIPSOL_OBJ) + 0.5) {
            Solution sol_curr(d_.ins);
            AltIdx o = d_.ins.alternative_number();
            double* x = getSolution(x_, o);
            for (AltIdx k=0; k<o; ++k)
                if (x[k] > 0.5)
                    sol_curr.set(k);
            std::stringstream ss;
            d_.sol.update(sol_curr, d_.lb, ss, d_.info);
        }
    }

private:

    BranchAndCutGurobiData& d_;
    GRBVar* x_;

};

Solution gap::sopt_branchandcut_gurobi(BranchAndCutGurobiData d)
{
    GRBEnv env;
    VER(d.info, "*** branchandcut_gurobi ***" << std::endl);

    init_display(d.info);

    ItemIdx n = d.ins.item_number();
    AgentIdx m = d.ins.agent_number();
    AltIdx o = d.ins.alternative_number();

    if (n == 0)
        return algorithm_end(d.sol, d.lb, d.info);

    GRBModel model(env);

    // Variables
    GRBVar* x = model.addVars(o, GRB_BINARY);

    // Objective
    for (AltIdx k=0; k<o; ++k)
        x[k].set(GRB_DoubleAttr_Obj, d.ins.alternative(k).c);
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    // Capacity constraints
    for (AgentIdx i=0; i<m; i++) {
        GRBLinExpr expr;
        for (ItemIdx j=0; j<n; j++)
            expr += x[d.ins.alternative_index(j, i)] * d.ins.alternative(j, i).w;
        model.addConstr(expr <= d.ins.capacity(i));
    }

    // One alternative per item constraint
    for (ItemIdx j=0; j<n; j++) {
        GRBLinExpr expr;
        for (AgentIdx i=0; i<m; i++)
            expr += x[d.ins.alternative_index(j, i)];
        model.addConstr(expr == 1);
    }

    // Initial solution
    if (d.sol.feasible())
        for (ItemIdx j=0; j<n; ++j)
            for (AgentIdx i=0; i<m; ++i)
                x[d.ins.alternative_index(j, i)].set(GRB_DoubleAttr_Start, (d.sol.agent(j) == i)? 1: 0);

    // Display
    model.getEnv().set(GRB_IntParam_OutputFlag, 0);

    // Time limit
    if (d.info.timelimit != std::numeric_limits<double>::infinity())
        model.set(GRB_DoubleParam_TimeLimit, d.info.timelimit);

    // Callback
    mycallback cb = mycallback(d, x);
    model.setCallback(&cb);

    // Optimize
    model.optimize();

    int optimstatus = model.get(GRB_IntAttr_Status);
    if (optimstatus == GRB_INFEASIBLE) {
        if (d.lb < d.ins.bound())
            update_lb(d.lb, d.ins.bound(), d.sol, std::stringstream(""), d.info);
    } else if (optimstatus == GRB_OPTIMAL) {
        if (!d.sol.feasible() || d.sol.cost() > model.get(GRB_DoubleAttr_ObjVal) + 0.5) {
            Solution sol_curr(d.ins);
            for (AltIdx k=0; k<o; ++k)
                if (x[k].get(GRB_DoubleAttr_X) > 0.5)
                    sol_curr.set(k);
            d.sol.update(sol_curr, d.lb, std::stringstream(""), d.info);
        }
        if (d.lb < d.sol.cost())
            update_lb(d.lb, d.sol.cost(), d.sol, std::stringstream(""), d.info);
    } else if (model.get(GRB_IntAttr_SolCount) > 0) {
        if (!d.sol.feasible() || d.sol.cost() > model.get(GRB_DoubleAttr_ObjVal) + 0.5) {
            Solution sol_curr(d.ins);
            for (AltIdx k=0; k<o; ++k)
                if (x[k].get(GRB_DoubleAttr_X) > 0.5)
                    sol_curr.set(k);
            d.sol.update(sol_curr, d.lb, std::stringstream(""), d.info);
        }
        if (d.lb < model.get(GRB_DoubleAttr_ObjBound) + 0.5)
            update_lb(d.lb, model.get(GRB_DoubleAttr_ObjBound), d.sol, std::stringstream(""), d.info);
    } else {
        if (d.lb < model.get(GRB_DoubleAttr_ObjBound) + 0.5)
            update_lb(d.lb, model.get(GRB_DoubleAttr_ObjBound), d.sol, std::stringstream(""), d.info);
    }

    return algorithm_end(d.sol, d.lb, d.info);
}

#endif

