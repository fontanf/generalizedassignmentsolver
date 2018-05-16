#include "gap/lib/subinstance.hpp"

using namespace gap;

SubInstance::SubInstance(const Instance& ins): instance_(ins)
{
    reduced_solution_ = new Solution(ins);
    alternatives_ = std::vector<int>(ins.alternative_number(), 0);
    alternative_number_ = std::vector<ItemIdx>(ins.agent_number(), 0);
}

SubInstance& SubInstance::operator=(const SubInstance sub)
{
    if (this != &sub) {
        *reduced_solution_  = *sub.reduced_solution_;
        alternatives_       = sub.alternatives_;
        alternative_number_ = sub.alternative_number_;
    }
    return *this;
}

SubInstance::~SubInstance()
{
    delete reduced_solution();
}

void SubInstance::set(ItemIdx j, AgentIdx i)
{
    reduced_solution_->set(j, i);
    const Item& it = instance().item(j);
    for (AltIdx k: it.alt)
        alternatives_[k] = -1;
}

void SubInstance::remove(AltIdx k)
{
    alternatives_[k] = -1;
}

////////////////////////////////////////////////////////////////////////////////

std::ostream& gap::operator<<(std::ostream& os, const SubInstance& sub)
{
    const Instance& ins = sub.instance();
    os <<  "N " << ins.item_number();
    os << " C";
    for (AgentIdx i=0; i<ins.agent_number(); ++i)
        os << " " << sub.capacity(i);
    os << std::endl;

    for (ItemPos j=0; j<ins.item_number(); ++j) {
        os << j << ":" << std::flush;
        for (AltIdx k: ins.item(j).alt)
            os << "; " << ins.alternative(k);
        if (ins.optimal_solution() != NULL)
            os << " O " << ins.optimal_solution()->agent(j);
        os << std::endl;
    }
    return os;
}

