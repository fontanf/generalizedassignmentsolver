#if GUROBI_FOUND

#include "gap/lb_linrelax_gurobi/linrelax_gurobi.hpp"

#include "gurobi_c++.h"

using namespace gap;

LinRelaxGurobiOutput gap::lb_linrelax_gurobi(const Instance& ins, Info info)
{
    VER(info, "*** linrelax_gurobi ***" << std::endl);

    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    AltIdx o = ins.alternative_number();

    GRBEnv env;
    GRBModel model(env);

    // Variables and objective
    GRBVar* x = model.addVars(o);
    for (AltIdx k=0; k<o; ++k) {
        x[k].set(GRB_CharAttr_VType, GRB_CONTINUOUS);
        x[k].set(GRB_DoubleAttr_LB, 0);
        x[k].set(GRB_DoubleAttr_UB, 1);
        x[k].set(GRB_DoubleAttr_Obj, ins.alternative(k).c);
    }
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

    // Capacity constraints
    for (AgentIdx i=0; i<m; i++) {
        GRBLinExpr expr;
        for (ItemIdx j=0; j<n; j++)
            expr += x[ins.alternative_index(j, i)] * ins.alternative(j, i).w;
        model.addConstr(expr <= ins.capacity(i));
    }

    // One alternative per item constraint
    for (ItemIdx j=0; j<n; j++) {
        GRBLinExpr expr;
        for (AgentIdx i=0; i<m; i++)
            expr += x[ins.alternative_index(j, i)];
        model.addConstr(expr == 1);
    }

    // Display
    model.getEnv().set(GRB_IntParam_OutputFlag, 0);

    // Optimize
    model.optimize();

    // Get solution
    LinRelaxGurobiOutput out;
    out.lb = std::ceil(model.get(GRB_DoubleAttr_ObjVal));
    out.x = std::vector<double>(o, 0);
    for (AltIdx k=0; k<ins.alternative_number(); ++k)
        out.x[k] = x[k].get(GRB_DoubleAttr_X);

    algorithm_end(out.lb, info);
    return out;
}

#endif

