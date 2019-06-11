#include "gap/opt_constraintprogramming_gecode/constraintprogramming_gecode.hpp"

#include <gecode/int.hh>
#include <gecode/search.hh>
#include <gecode/minimodel.hh>
#include <gecode/gist.hh>

using namespace gap;
using namespace Gecode;

/**
 * Useful links
 * https://www.gecode.org/doc-latest/MPG.pdf
 */

class GapGecode: public IntMinimizeSpace
{

public:

    GapGecode(const Instance& ins):
        ins_(ins),
        xij0_(*this, ins.item_number() * ins.agent_number(), 0, 1),
        xj_(*this, ins.item_number(), 0, ins.agent_number() - 1),
        cj_(*this, ins.item_number()),
        load_(*this, ins.agent_number())
    {
        ItemIdx n = ins.item_number();
        AgentIdx m = ins.agent_number();

        // Channel xij == 1 <=> xj == i
        Matrix<BoolVarArgs> xij(xij0_, n, m);
        for (ItemIdx j=0; j<n; j++)
            channel(*this, xij.col(j), xj_[j]);

        // Weights
        weights_ = std::vector<IntArgs>(m, IntArgs(n));
        for (AgentIdx i=0; i<m; ++i)
            for (ItemIdx j=0; j<n; ++j)
                weights_[i][j] = ins.alternative(j, i).w;

        // Costs
        costs_ = std::vector<IntArgs>(n, IntArgs(m));
        for (ItemIdx j=0; j<n; ++j)
            for (AgentIdx i=0; i<m; ++i)
                costs_[j][i] = ins.alternative(j, i).c;

        // Cost variables
        for (ItemIdx j=0; j<n; j++) {
            cj_[j] = IntVar(*this, ins.item(j).c_min, ins.item(j).c_max);
            element(*this, costs_[j], xj_[j], cj_[j]);
        }
        c_ = expr(*this, sum(cj_));

        // Load variables (and capacity constraint)
        for (AgentIdx i=0; i<m; ++i) {
            load_[i] = IntVar(*this, 0, ins.capacity(i));
            linear(*this, weights_[i], xij.row(i), IRT_EQ, load_[i]);
        }
    }

    ~GapGecode() {  }

    GapGecode(GapGecode& s):
        IntMinimizeSpace(s), ins_(s.ins_), weights_(s.weights_), costs_(s.costs_)
    {
        xij0_.update(*this, s.xij0_);
        xj_.update(*this, s.xj_);
        cj_.update(*this, s.cj_);
        c_.update(*this, s.c_);
        load_.update(*this, s.load_);
    }

    virtual IntMinimizeSpace* copy(void) { return new GapGecode(*this); }

    virtual IntVar cost(void) const { return c_; }

    void print(void) const
    {
        std::cout << "Cost " << c_ << std::endl;
    }

    AgentIdx agent(ItemIdx j) const { return xj_[j].val(); }

private:

    const Instance& ins_;
    std::vector<IntArgs> weights_;
    std::vector<IntArgs> costs_;
    BoolVarArray xij0_;
    IntVarArray xj_;
    IntVarArray cj_;
    IntVar c_;
    IntVarArray load_;

};

Solution gap::sopt_constraintprogramming_gecode(ConstraintProgrammingGecodeData d)
{
    VER(d.info, "*** constraintprogramming_gecode ***" << std::endl);

    GapGecode model(d.ins);
    //Gist::bab(&model);
    BAB<GapGecode> engine(&model);
    GapGecode* sol = NULL;
    while ((sol = engine.next())) {
        std::cout << "New solution" << std::endl;
        sol->print();
        for (ItemIdx j=0; j<d.ins.item_number(); j++)
            d.sol.set(j, sol->agent(j));
        delete sol;
    }

    return algorithm_end(d.sol, d.info);
}

