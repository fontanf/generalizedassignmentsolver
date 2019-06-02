#include "gap/ub_vnsbranching_cbc/vnsbranching_cbc.hpp"

#include "gap/ub_random/random.hpp"
#include "gap/ub_repair/repair.hpp"
#include "gap/opt_branchandcut_cbc/branchandcut_cbc.hpp"

#include <set>
#include <random>
#include <algorithm>
#include <vector>

using namespace gap;

Solution gap::sol_vnsbranching_cbc(const Instance& ins, Info info)
{
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    AltIdx o = ins.alternative_number();

    init_display(info);
    Solution sol_best(ins);

    //LinRelaxClpOutput linrelax_output = lb_linrelax_clp(ins);
    //Cost lb = linrelax_output.lb;
    //Solution sol_curr = sol_repairlinrelax(ins, linrelax_output);
    Cost lb = 0;
    std::mt19937_64 gen;
    Solution sol_curr = sol_random(ins, gen);

    std::stringstream ss;
    sol_best.update(sol_curr, lb, ss, info);

    for (AltIdx k_max=2; k_max<o; k_max+=2) {
        std::vector<int> vec_ind(o, 0);
        std::vector<double> vec_elem(o, 0);
        AltIdx k_ind = 0;
        for (ItemIdx j=0; j<n; ++j) {
            for (AgentIdx i=0; i<m; ++i, k_ind++) {
                AltIdx k = ins.alternative_index(j, i);
                vec_ind[k_ind] = k;
                if (i == sol_best.agent(j)) {
                    vec_elem[k_ind] = -1;
                } else {
                    vec_elem[k_ind] = 1;
                }
            }
        }
        MilpMatrix mat(ins);
        mat.matrix.appendRow(o, vec_ind.data(), vec_elem.data());
        mat.rowLower.push_back(- n);
        mat.rowUpper.push_back(k_max - n);

        OsiCbcSolverInterface solver1;

        // Reduce printout
        solver1.getModelPtr()->setLogLevel(0);
        solver1.messageHandler()->setLogLevel(0);

        // Load problem
        solver1.loadProblem(mat.matrix, mat.colLower.data(), mat.colUpper.data(),
                mat.objective.data(), mat.rowLower.data(), mat.rowUpper.data());

        // Mark integer
        for (AltIdx k=0; k<o; ++k)
            solver1.setInteger(k);

        // Solve
        solver1.initialSolve();

        // Pass data and solver to CbcModel
        CbcModel model(solver1);

        model.setMaximumSolutions(1);

        // Reduce printout
        model.setLogLevel(0);
        model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);

        // Set time limit
        model.setMaximumSeconds(info.timelimit - info.elapsed_time());

        // Add initial solution
        std::vector<double> sol_init(o, 0);
        for (AltIdx k=0; k<o; ++k)
            if (sol_best.agent(ins.alternative(k).j) == ins.alternative(k).i)
                sol_init[k] = 1;
        model.setBestSolution(sol_init.data(), o, sol_best.cost());

        // Do complete search
        model.branchAndBound();

        if (sol_best.cost() <= model.getObjValue() + 0.5)
            continue;

        // Get solution
        const double *solution = model.solver()->getColSolution();
        Solution sol_tmp(ins);
        for (AltIdx k=0; k<o; ++k)
            if (solution[k] > 0.5)
                sol_tmp.set(k);
        std::stringstream ss;
        ss << "k_max " << k_max;
        sol_best.update(sol_tmp, lb, ss, info);
        k_max = 0;
    }

    return algorithm_end(sol_best, info);
}

