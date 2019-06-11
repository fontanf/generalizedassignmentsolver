#include "gap/opt_dip/dip.hpp"

#include "knapsack/opt_minknap/minknap.hpp"

#include <coin/DecompApp.h>
#include <coin/DecompVar.h>
#include <coin/AlpsDecompModel.h>
#include <coin/DecompAlgoC.h>
#include <coin/DecompAlgoPC.h>
#include <coin/UtilTimer.h>

using namespace gap;

/**
 * Useful links
 * https://github.com/coin-or/Dip/wiki
 * https://github.com/coin-or/Dip/blob/master/Dip/examples/GAP/GAPDecompApp.cpp
 * http://coral.ie.lehigh.edu/~ted/files/talks/DecompCSIRO11.pdf
 * https://www.coin-or.org/Doxygen/Dip/class_decomp_app.html
 */

class GAPDecompApp: public DecompApp
{

public:

    GAPDecompApp(UtilParameters& util_param, const Instance& ins);

    virtual ~GAPDecompApp() { UtilDeleteMapPtr(models_); };

    DecompSolverStatus solveRelaxed(
            const int whichBlock, const double* redCostX,
            const double target, std::list<DecompVar*>& vars);

    int createModelPartAP(DecompConstraintSet* model);

    inline const Instance& instance() const { return ins_; }
    inline DecompConstraintSet* getModel(std::string modelName) const
    {
        std::map<std::string, DecompConstraintSet*>::const_iterator it;
        it = models_.find(modelName);

        if (it == models_.end()) {
            std::cout << "Error: model with name " << modelName << " not defined." << std::endl;
            assert(it != models_.end());
            return NULL;
        }

        return it->second;
    }

private:

    const Instance& ins_;

    /**
     * The various model constraint systems used for different algorithms, keyed
     * by a unique string (model name).
     */
    std::map<std::string, DecompConstraintSet*> models_;

};

GAPDecompApp::GAPDecompApp(UtilParameters& util_param, const Instance& ins):
    DecompApp(util_param), ins_(ins)
{
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    AltIdx o = ins.alternative_number();

    std::vector<double> objective(o);
    m_objective = new double[o];
    for (ItemIdx j=0; j<n; ++j) {
        for (AgentIdx i=0; i<m; ++i) {
            AltIdx k = ins.alternative_index(j, i);
            objective[k] = ins.alternative(k).c;
        }
    }
    setModelObjective(objective.data(), o);

    DecompConstraintSet* modelCore = new DecompConstraintSet();
    int status = createModelPartAP(modelCore);
    if (status)
        return;

    setModelCore(modelCore, "AP");
    models_.insert(std::make_pair("AP", modelCore));

    for (AgentIdx i=0; i<m; i++) {
        std::string modelName = "KP" + UtilIntToStr(i);
        setModelRelax(NULL, modelName, i);
    }
}


int GAPDecompApp::createModelPartAP(DecompConstraintSet* model)
{
    const Instance& ins = ins_;
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    AltIdx o = m * n;
    model->M = new CoinPackedMatrix(false, 0.0, 0.0);
    CoinAssertHint(model->M, "Error: Out of Memory");
    model->M->setDimensions(0, o);
    model->reserve(n, o);

    for (ItemIdx j=0; j<n; j++) {
        CoinPackedVector row;
        std::string rowName = "a(j_" + UtilIntToStr(j) + ")";
        for (AgentIdx i=0; i<m; i++) {
            AltIdx k = ins.alternative_index(j, i);
            row.insert(k, 1.0);
        }
        model->appendRow(row, 1.0, 1.0, rowName);
    }

    // set the col upper and lower bounds
    UtilFillN(model->colLB, o, 0.0);
    UtilFillN(model->colUB, o, 1.0);

    // set column names for debugging
    AltIdx colIndex = 0;
    for (AgentIdx i=0; i<m; i++) {
        for (ItemIdx j=0; j<n; j++) {
            std::string colName = "x("
                + UtilIntToStr(colIndex) + "_"
                + UtilIntToStr(i) + "," + UtilIntToStr(j) + ")";
            model->colNames.push_back(colName);
            colIndex++;
        }
    }

    // set the indices of the integer variables of model
    UtilIotaN(model->integerVars, o, 0);
    return 0;
}

DecompSolverStatus GAPDecompApp::solveRelaxed(
        const int i, const double* rc, const double target, std::list<DecompVar*>& vars)
{
    (void)target;
    const Instance& ins = ins_;
    ItemIdx n = ins.item_number();
    //AgentIdx m = ins.agent_number();
    double varRedCost  = 0.0;
    double varOrigCost = 0.0;
    std::vector<int>    sol_ind(n, -1);
    std::vector<double> sol_els(n, 0);

    for (AltIdx k=0; k<ins.alternative_number(); ++k)
        std::cout << " " << rc[k];
    std::cout << std::endl;

    // Solve independent knapsack problems
    Weight mult = 10000;
    std::vector<ItemIdx> indices(n);
    knapsack::Instance ins_kp(n, ins.capacity(i));
    for (ItemIdx j=0; j<n; ++j) {
        AltIdx k = ins.alternative_index(j, i);
        sol_ind[j] = k;
        const Alternative& a = ins.alternative(k);
        knapsack::Profit p = mult * rc[j];
        if (p > 0) {
            knapsack::ItemIdx j_kp = ins_kp.add_item(a.w, p);
            indices[j_kp] = j;
        }
    }
    knapsack::Solution sol = knapsack::Minknap(ins_kp, knapsack::MinknapParams::combo()).run();
    for (knapsack::ItemIdx j_kp=0; j_kp<ins_kp.total_item_number(); ++j_kp) {
        if (sol.contains_idx(j_kp)) {
            ItemIdx j = indices[j_kp];
            sol_els[j] = 1;
            AltIdx k = ins.alternative_index(j, i);
            varOrigCost += ins.alternative(k).c;
            varRedCost += rc[k];
        }
    }

    DecompVar* var = new DecompVar(sol_ind, sol_els, varRedCost, varOrigCost);
    var->setBlockId(i);
    vars.push_back(var);
    return DecompSolStatOptimal;
}

Cost gap::dip(const Instance& ins)
{
    try {
        UtilParameters utilParam;
        utilParam.Add("DECOMP", "BranchEnforceInMaster", "1");
        utilParam.Add("DECOMP", "BranchEnforceInSubProb", "0");

        bool doCut          = utilParam.GetSetting("doCut",          true);
        bool doPriceCut     = utilParam.GetSetting("doPriceCut",     false);
        bool doDirect       = utilParam.GetSetting("doDirect",       false);

        UtilTimer timer;
        double    timeSetupReal = 0.0;
        double    timeSetupCpu  = 0.0;
        double    timeSolveReal = 0.0;
        double    timeSolveCpu  = 0.0;
        // start overall timer
        timer.start();
        // create the user application (a DecompApp)
        GAPDecompApp gap(utilParam, ins);
        // create the algorithm (a DecompAlgo)
        DecompAlgo* algo = NULL;
        assert(doCut + doPriceCut == 1);

        // create the CPM algorithm object
        if (doCut) {
            algo = new DecompAlgoC(&gap, utilParam);
        }

        // create the PC algorithm object
        if (doPriceCut) {
            algo = new DecompAlgoPC(&gap, utilParam);
        }

        if (doCut && doDirect) {
            timer.stop();
            timeSetupCpu  = timer.getCpuTime();
            timeSetupReal = timer.getRealTime();
            // solve
            timer.start();
            algo->solveDirect();
            timer.stop();
            timeSolveCpu  = timer.getCpuTime();
            timeSolveReal = timer.getRealTime();
        } else {
            timer.stop();
            timeSetupCpu  = timer.getCpuTime();
            timeSetupReal = timer.getRealTime();
            // create the driver AlpsDecomp model
            int status = 0;
            AlpsDecompModel alpsModel(utilParam, algo);
            // solve
            timer.start();
            status = alpsModel.solve();
            timer.stop();
            timeSolveCpu  = timer.getCpuTime();
            timeSolveReal = timer.getRealTime();
            // sanity check
            std::cout << setiosflags(std::ios::fixed | std::ios::showpoint);
            std::cout << "Status= " << status
                << " BestLB= " << std::setw(10)
                << UtilDblToStr(alpsModel.getGlobalLB(), 5)
                << " BestUB= " << std::setw(10)
                << UtilDblToStr(alpsModel.getGlobalUB(), 5)
                << " Nodes= " << std::setw(6)
                << alpsModel.getNumNodesProcessed()
                << " SetupCPU= "  << timeSetupCpu
                << " SolveCPU= "  << timeSolveCpu
                << " TotalCPU= "  << timeSetupCpu + timeSolveCpu
                << " SetupReal= " << timeSetupReal
                << " SetupReal= " << timeSolveReal
                << " TotalReal= " << timeSetupReal + timeSetupReal
                << std::endl;

            if (status == AlpsExitStatusOptimal && gap.getBestKnownUB() < 1.0e50) {
                // the assumption here is that the BestKnownLB/UB is optimal
                double diff = fabs(gap.getBestKnownUB() - alpsModel.getGlobalUB());

                if (diff > 1.0e-4) {
                    std::cerr << "ERROR. BestKnownUB= " << gap.getBestKnownUB()
                        << " but DECOMP claims GlobalUB= "
                        << alpsModel.getGlobalUB() << std::endl;
                    throw UtilException("Invalid claim of optimal.", "main", "DECOMP");
                }
            }
        }

        // free local memory
        delete algo;
    } catch (CoinError& ex) {
        std::cerr << "COIN Exception [ " << ex.message() << " ]"
            << " at " << ex.fileName()  << ":L" << ex.lineNumber()
            << " in " << ex.className() << "::" << ex.methodName() << std::endl;
        return 1;
    }

    return 0;
}

