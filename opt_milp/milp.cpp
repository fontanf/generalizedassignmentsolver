#include "gap/opt_milp/milp.hpp"

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

Solution gap::sopt_milp(MilpData d)
{
    VER(d.info, "*** milp ***" << std::endl);

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

