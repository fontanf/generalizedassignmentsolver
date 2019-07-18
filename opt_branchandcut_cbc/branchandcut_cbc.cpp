#include "gap/opt_branchandcut_cbc/branchandcut_cbc.hpp"

#include "coin/CbcHeuristicDiveCoefficient.hpp"
#include "coin/CbcHeuristicDiveFractional.hpp"
#include "coin/CbcHeuristicDiveGuided.hpp"
#include "coin/CbcHeuristicDiveVectorLength.hpp"
#include "coin/CbcLinked.hpp"
#include "coin/CbcHeuristicGreedy.hpp"
#include "coin/CbcHeuristicFPump.hpp"
#include "coin/CbcHeuristicLocal.hpp"
#include "coin/CbcHeuristic.hpp"
#include "coin/CbcHeuristicRINS.hpp"
#include "coin/CbcHeuristicRENS.hpp"

#include "coin/CglAllDifferent.hpp"
#include "coin/CglClique.hpp"
#include "coin/CglDuplicateRow.hpp"
#include "coin/CglFlowCover.hpp"
#include "coin/CglGomory.hpp"
#include "coin/CglKnapsackCover.hpp"
#include "coin/CglLandP.hpp"
#include "coin/CglLiftAndProject.hpp"
#include "coin/CglMixedIntegerRounding.hpp"
#include "coin/CglMixedIntegerRounding2.hpp"
#include "coin/CglOddHole.hpp"
#include "coin/CglProbing.hpp"
#include "coin/CglRedSplit.hpp"
#include "coin/CglResidualCapacity.hpp"
#include "coin/CglSimpleRounding.hpp"
#include "coin/CglStored.hpp"
#include "coin/CglTwomir.hpp"

/**
 * Useful links:
 * https://github.com/coin-or/Cgl/wiki
 * https://github.com/coin-or/Cbc/blob/master/Cbc/examples/sample2.cpp
 * https://github.com/coin-or/Cbc/blob/master/Cbc/examples/sample3.cpp
 */

using namespace gap;

MilpMatrix::MilpMatrix(const Instance& ins)
{
    int numberRows = ins.agent_number() + ins.item_number();
    int numberColumns = ins.alternative_number();
    int numberElements = 2 * ins.alternative_number();

    // Matrix data - column ordered
    std::vector<CoinBigIndex> start(numberColumns + 1);
    for (AltIdx k=0; k<=numberColumns; ++k)
        start[k] = 2 * k;
    std::vector<int> length(numberColumns, 2);

    std::vector<int> rows(numberElements);
    std::vector<double> elements(numberElements);
    for (AltIdx k=0; k<ins.alternative_number(); ++k) {
            rows[2 * k] = ins.alternative(k).i;
        elements[2 * k] = ins.alternative(k).w;
            rows[2 * k + 1] = ins.agent_number() + ins.alternative(k).j;
        elements[2 * k + 1] = 1;
    }

    matrix = CoinPackedMatrix(true, numberRows, numberColumns, numberElements,
            elements.data(), rows.data(), start.data(), length.data());

    // Rim data
    objective = std::vector<double>(numberColumns);
    for (AltIdx k=0; k<numberColumns; ++k)
        objective[k] = ins.alternative(k).c;

    rowLower = std::vector<double>(numberRows);
    rowUpper = std::vector<double>(numberRows);
    for (AgentIdx i=0; i<ins.agent_number(); ++i) {
        rowLower[i] = 0;
        rowUpper[i] = ins.capacity(i);
    }
    for (ItemIdx j=0; j<ins.item_number(); ++j) {
        rowLower[ins.agent_number() + j] = 1;
        rowUpper[ins.agent_number() + j] = 1;
    }
    colLower = std::vector<double>(numberColumns, 0);
    colUpper = std::vector<double>(numberColumns, 1);
}

Solution gap::sopt_branchandcut_cbc(BranchAndCutCbcData d)
{
    VER(d.info, "*** branchandcut_cbc ***" << std::endl);

    int loglevel = (d.info.output->verbose)? 1: 0;

    MilpMatrix mat(d.ins);

    OsiCbcSolverInterface solver1;

    // Reduce printout
    solver1.getModelPtr()->setLogLevel(loglevel);
    solver1.messageHandler()->setLogLevel(loglevel);

    // Load problem
    solver1.loadProblem(mat.matrix, mat.colLower.data(), mat.colUpper.data(),
              mat.objective.data(), mat.rowLower.data(), mat.rowUpper.data());

    // Mark integer
    for (AltIdx k=0; k<d.ins.alternative_number(); ++k)
        solver1.setInteger(k);

    // Solve
    solver1.initialSolve();

    // Pass data and solver to CbcModel
    CbcModel model(solver1);

    // Reduce printout
    model.setLogLevel(loglevel);
    model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);

    // Heuristics
    //CbcHeuristicDiveCoefficient heuristic_divecoefficient(model);
    //model.addHeuristic(&heuristic_divecoefficient);
    //CbcHeuristicDiveFractional heuristic_divefractional(model);
    //model.addHeuristic(&heuristic_divefractional);
    //CbcHeuristicDiveGuided heuristic_diveguided(model);
    //model.addHeuristic(&heuristic_diveguided);
    //CbcHeuristicDiveVectorLength heuristic_divevetorlength(model);
    //model.addHeuristic(&heuristic_divevetorlength);
    //CbcHeuristicDynamic3 heuristic_dynamic3(model); // crash
    //model.addHeuristic(&heuristic_dynamic3);
    //CbcHeuristicFPump heuristic_fpump(model);
    //model.addHeuristic(&heuristic_fpump);
    //CbcHeuristicGreedyCover heuristic_greedycover(model);
    //model.addHeuristic(&heuristic_greedycover);
    //CbcHeuristicGreedyEquality heuristic_greedyequality(model);
    //model.addHeuristic(&heuristic_greedyequality);
    //CbcHeuristicLocal heuristic_local(model);
    //model.addHeuristic(&heuristic_local);
    //CbcHeuristicPartial heuristic_partial(model);
    //model.addHeuristic(&heuristic_partial);
    //CbcHeuristicRENS heuristic_rens(model);
    //model.addHeuristic(&heuristic_rens);
    //CbcHeuristicRINS heuristic_rins(model);
    //model.addHeuristic(&heuristic_rins);
    //CbcRounding heuristic_rounding(model);
    //model.addHeuristic(&heuristic_rounding);
    //CbcSerendipity heuristic_serendipity(model);
    //model.addHeuristic(&heuristic_serendipity);

    // Cuts
    //CglClique cutgen_clique;
    //model.addCutGenerator(&cutgen_clique);
    //CglAllDifferent cutgen_alldifferent;
    //model.addCutGenerator(&cutgen_alldifferent);
    //CglDuplicateRow cutgen_duplicaterow;
    //model.addCutGenerator(&cutgen_duplicaterow);
    //CglFlowCover cutgen_flowcover;
    //model.addCutGenerator(&cutgen_flowcover);
    //CglGomory cutgen_gomory;
    //model.addCutGenerator(&cutgen_gomory);
    //CglKnapsackCover cutgen_knapsackcover;
    //model.addCutGenerator(&cutgen_knapsackcover);
    //CglLandP cutgen_landp;
    //model.addCutGenerator(&cutgen_landp);
    //CglLiftAndProject cutgen_liftandproject;
    //model.addCutGenerator(&cutgen_liftandproject);
    //CglMixedIntegerRounding cutgen_mixedintegerrounding;
    //model.addCutGenerator(&cutgen_mixedintegerrounding);
    //CglMixedIntegerRounding2 cutgen_mixedintegerrounding2;
    //model.addCutGenerator(&cutgen_mixedintegerrounding2);
    //CglOddHole cutgen_oddhole;
    //model.addCutGenerator(&cutgen_oddhole);
    //CglProbing cutgen_probing;
    //model.addCutGenerator(&cutgen_probing);
    //CglRedSplit cutgen_redsplit;
    //model.addCutGenerator(&cutgen_redsplit);
    //CglResidualCapacity cutgen_residualcapacity;
    //model.addCutGenerator(&cutgen_residualcapacity);
    //CglSimpleRounding cutgen_simplerounding;
    //model.addCutGenerator(&cutgen_simplerounding);
    //CglStored cutgen_stored;
    //model.addCutGenerator(&cutgen_stored);
    //CglTwomir cutgen_twomir;
    //model.addCutGenerator(&cutgen_twomir);

    // Set time limit
    model.setMaximumSeconds(d.info.timelimit);

    // Add initial solution
    std::vector<double> sol_init(d.ins.alternative_number(), 0);
    if (d.sol.feasible()) {
        for (AltIdx k=0; k<d.ins.alternative_number(); ++k)
            if (d.sol.agent(d.ins.alternative(k).j) == d.ins.alternative(k).i)
                sol_init[k] = 1;
        model.setBestSolution(sol_init.data(), d.ins.alternative_number(), d.sol.cost());
    }
    if (d.stop_at_first_improvment)
        model.setMaximumSolutions(1);

    // Do complete search
    model.branchAndBound();

    if (d.sol.feasible() && d.sol.cost() <= model.getObjValue() + 0.5)
        return algorithm_end(d.sol, d.info);

    // Get solution
    const double *solution = model.solver()->getColSolution();
    for (AltIdx k=0; k<d.ins.alternative_number(); ++k)
        if (solution[k] > 0.5)
            d.sol.set(k);
    if (!d.sol.feasible())
        d.sol = Solution(d.ins);
    return algorithm_end(d.sol, d.info);
}

