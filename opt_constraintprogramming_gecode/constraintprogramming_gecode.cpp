#include "gap/opt_constraintprogramming_gecode/constraintprogramming_gecode.hpp"

#include <gecode/int.hh>
#include <gecode/search.hh>
#include <gecode/minimodel.hh>

using namespace gap;

class GAPGeocode: public Gecode::Space
{

public:

    GAPGeocode(const Instance& ins):
        ins_(ins),
        xij0_(*this, ins.item_number() * ins.agent_number(), 0, 1),
        xj_(*this, ins.item_number(), 0, ins.agent_number() - 1),
        load_(*this, ins.item_number())
    {
        for (AgentIdx i=0; i<ins.agent_number(); ++i)
            load_[i] = Gecode::IntVar(*this, 0, ins.capacity(i));

        sizes_ = std::vector<Gecode::IntArgs>(ins.agent_number(), Gecode::IntArgs(ins.item_number()));
        for (ItemIdx j=0; j<ins.item_number(); j++)
            for (AgentIdx i=0; i<ins.agent_number(); ++i)
                sizes_[i][j] = ins.alternative(j, i).w;

        Gecode::Matrix<Gecode::BoolVarArgs> xij(xij0_, ins.item_number(), ins.agent_number());
        for (ItemIdx j=0; j<ins.item_number(); j++)
            channel(*this, xij.col(j), xj_[j]);

        for (AgentIdx i=0; i<ins.agent_number(); ++i)
            linear(*this, sizes_[i], xij.row(i), Gecode::IRT_EQ, load_[i]);
    }

    ~GAPGeocode() {  }

    GAPGeocode(GAPGeocode& s):
        Space(s),
        ins_(s.ins_),
        sizes_(s.sizes_)
    {
        xij0_.update(*this, s.xij0_);
        xj_.update(*this, s.xj_);
        load_.update(*this, s.load_);
    }

    virtual Space* copy(void) { return new GAPGeocode(*this); }

    void print(void) const { }

private:

    const Instance& ins_;
    std::vector<Gecode::IntArgs> sizes_;
    Gecode::BoolVarArray xij0_;
    Gecode::IntVarArray xj_;
    Gecode::IntVarArray load_;

};

Solution gap::sopt_constraintprogramming_gecode(ConstraintProgrammingGecodeData d)
{
    VER(d.info, "*** constraintprogramming_gecode ***" << std::endl);

    GAPGeocode model(d.ins);
    //Gecode::BAB<GAPGeocode> e(model);

    return algorithm_end(d.sol, d.info);
}

