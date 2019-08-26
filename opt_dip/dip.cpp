#if COINOR_FOUND

#include "gap/opt_dip/dip.hpp"

#include "knapsack/opt_minknap/minknap.hpp"

#include <coin/DecompApp.h>
#include <coin/DecompVar.h>
#include <coin/AlpsDecompModel.h>
#include <coin/DecompAlgoC.h>
#include <coin/DecompAlgoPC.h>
#include <coin/DecompAlgoRC.h>
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

    DecompSolverStatus solveRelaxed(const int i, const double* rc,
            const double target, std::list<DecompVar*>& vars);

    void createModelPartAP(DecompConstraintSet* model);

    DecompConstraintSet* getModel(std::string modelName) const;
    inline const Instance& instance() const { return ins_; }

private:

    const Instance& ins_;
    std::map<std::string, DecompConstraintSet*> models_;

};

DecompConstraintSet* GAPDecompApp::getModel(std::string modelName) const
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

GAPDecompApp::GAPDecompApp(UtilParameters& util_param, const Instance& ins):
    DecompApp(util_param), ins_(ins)
{
    AgentIdx m = ins.agent_number();
    AltIdx o = ins.alternative_number();

    // Objective
    std::vector<double> objective(o);
    m_objective = new double[o];
    for (AltIdx k=0; k<o; ++k)
        objective[k] = ins.alternative(k).c;
    setModelObjective(objective.data(), o);

    // Assignment constraints
    DecompConstraintSet* modelCore = new DecompConstraintSet();
    createModelPartAP(modelCore);
    setModelCore(modelCore, "AP");
    models_.insert(std::make_pair("AP", modelCore));

    // Knapsack constraints
    for (AgentIdx i=0; i<m; i++)
        setModelRelax(NULL, "KP" + std::to_string(i), i);
}

void GAPDecompApp::createModelPartAP(DecompConstraintSet* model)
{
    const Instance& ins = ins_;
    ItemIdx n = ins.item_number();
    AgentIdx m = ins.agent_number();
    AltIdx o = ins.alternative_number();

    model->M = new CoinPackedMatrix(false, 0.0, 0.0);
    CoinAssertHint(model->M, "Error: Out of Memory");
    model->M->setDimensions(0, o);
    model->reserve(n, o);

    for (ItemIdx j=0; j<n; j++) {
        CoinPackedVector row;
        for (AgentIdx i=0; i<m; i++)
            row.insert(ins.alternative_index(j, i), 1.0);
        model->appendRow(row, 1.0, 1.0, "AP" + std::to_string(j));
    }

    // Column bounds
    UtilFillN(model->colLB, o, 0.0);
    UtilFillN(model->colUB, o, 1.0);

    // Column names
    for (AltIdx k=0; k<o; ++k) {
        const Alternative& a = ins.alternative(k);
        model->colNames.push_back("x_" + std::to_string(a.j) + "," + std::to_string(a.i));
    }

    // Set variable integer
    UtilIotaN(model->integerVars, o, 0);
}

DecompSolverStatus GAPDecompApp::solveRelaxed(
        const int i, const double* rc, const double target, std::list<DecompVar*>& vars)
{
    (void)target;
    const Instance& ins = ins_;
    ItemIdx n = ins.item_number();

    double varRedCost  = 0.0;
    double varOrigCost = 0.0;
    std::vector<int>    sol_ind;
    std::vector<double> sol_els;

    // Build Knapsck Instance
    Weight mult = 1000;
    std::vector<ItemIdx> indices(n);
    knapsack::Instance ins_kp;
    ins_kp.set_capacity(ins.capacity(i));
    for (ItemIdx j=0; j<n; ++j) {
        AltIdx k = ins.alternative_index(j, i);
        if (rc[k] < 0) {
            ins_kp.add_item(ins.alternative(k).w, std::ceil(- mult * rc[k]));
            knapsack::ItemIdx j_kp = ins_kp.item_number() - 1;
            indices[j_kp] = j;
        }
    }

    // Solve Knapsack Instance
    knapsack::Solution sol = knapsack::sopt_minknap(ins_kp, knapsack::MinknapParams::combo());

    // Retrieve Knapsack Solution
    for (knapsack::ItemIdx j_kp=0; j_kp<ins_kp.total_item_number(); ++j_kp) {
        if (sol.contains_idx(j_kp)) {
            ItemIdx j = indices[j_kp];
            AltIdx k = ins.alternative_index(j, i);
            sol_ind.push_back(k);
            sol_els.push_back(1.0);
            varOrigCost += ins.alternative(k).c;
            varRedCost += rc[k];
        }
    }
    //std::cout << "solve kp " << i << " opt " << varOrigCost << " " << varRedCost << std::endl;

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

        bool doCut      = utilParam.GetSetting("doCut",      true);
        bool doPriceCut = utilParam.GetSetting("doPriceCut", false);
        bool doDirect   = utilParam.GetSetting("doDirect",   true);

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
            // solve
            algo->solveDirect();
        } else {
            // create the driver AlpsDecomp model
            AlpsDecompModel alpsModel(utilParam, algo);
            // solve
            int status = alpsModel.solve();
            // sanity check
            std::cout << setiosflags(std::ios::fixed | std::ios::showpoint);
            std::cout << "Status= " << status
                << " BestLB= " << std::setw(10)
                << UtilDblToStr(alpsModel.getGlobalLB(), 5)
                << " BestUB= " << std::setw(10)
                << UtilDblToStr(alpsModel.getGlobalUB(), 5)
                << " Nodes= " << std::setw(6)
                << alpsModel.getNumNodesProcessed()
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

Solution gap::sopt_branchandcut_dip(const Instance& ins, Info info)
{
    UtilParameters utilParam;
    utilParam.Add("DECOMP", "BranchEnforceInMaster", "1");
    utilParam.Add("DECOMP", "BranchEnforceInSubProb", "0");
    GAPDecompApp gap(utilParam, ins);
    DecompAlgoC algo(&gap, utilParam);
    algo.solveDirect();
    Solution sol(ins);
    return algorithm_end(sol, info);
}

Solution gap::sopt_branchandpriceandcut_dip(const Instance& ins, Info info)
{
    UtilParameters utilParam;
    utilParam.Add("DECOMP", "BranchEnforceInMaster", "1");
    utilParam.Add("DECOMP", "BranchEnforceInSubProb", "0");
    GAPDecompApp gap(utilParam, ins);
    DecompAlgoPC algo(&gap, utilParam);

    Solution sol(ins);
    return algorithm_end(sol, info);
}

Solution gap::sopt_relaxandcut_dip(const Instance& ins, Info info)
{
    UtilParameters utilParam;
    utilParam.Add("DECOMP", "BranchEnforceInMaster", "1");
    utilParam.Add("DECOMP", "BranchEnforceInSubProb", "0");
    GAPDecompApp gap(utilParam, ins);
    DecompAlgoRC algo(&gap, utilParam);

    Solution sol(ins);
    return algorithm_end(sol, info);
}

#endif

