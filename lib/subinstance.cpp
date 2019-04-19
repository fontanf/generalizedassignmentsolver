#include "gap/lib/subinstance.hpp"

using namespace gap;

SubInstance::SubInstance(const Instance& ins): instance_(ins)
{
    reduced_solution_ = new Solution(ins);
    alternatives_ = std::vector<int>(ins.alternative_number(), 0);
    agent_alternative_number_ = std::vector<ItemIdx>(ins.agent_number(), ins.item_number());
    item_alternative_number_ = std::vector<ItemIdx>(ins.item_number(), ins.agent_number());
    remove_big_alt();
}

SubInstance::SubInstance(const SubInstance& sub): instance_(sub.instance_)
{
    reduced_solution_ = new Solution(sub.instance());
    *reduced_solution_ = *sub.reduced_solution_;
    alternatives_ = sub.alternatives_;
    agent_alternative_number_ = sub.agent_alternative_number_;
    item_alternative_number_ = sub.item_alternative_number_;
}

SubInstance& SubInstance::operator=(const SubInstance sub)
{
    if (this != &sub) {
        *reduced_solution_ = *sub.reduced_solution_;
        alternatives_ = sub.alternatives_;
        agent_alternative_number_ = sub.agent_alternative_number_;
        item_alternative_number_ = sub.item_alternative_number_;
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
    for (AgentIdx i=0; i<instance().agent_number(); ++i)
        remove(j, i);
    remove_big_alt(i);
}

void SubInstance::remove(ItemIdx j, AgentIdx i)
{
    AltIdx k = instance().alternative_index(j, i);
    if (alternatives_[k] == -1)
        return;
    alternatives_[k] = -1;
    agent_alternative_number_[i]--;
    item_alternative_number_[j]--;
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
            os << " " << ins.alternative(k);
        if (ins.optimal_solution() != NULL)
            os << " O " << ins.optimal_solution()->agent(j);
        os << std::endl;
    }
    return os;
}

void SubInstance::remove_big_alt()
{
    for (AgentIdx i=0; i<instance().agent_number(); ++i)
        remove_big_alt(i);
}

void SubInstance::remove_big_alt(AgentIdx i)
{
    for (ItemIdx j=0; j<instance().item_number(); ++j) {
        AltIdx k = instance().alternative_index(j, i);
        if (reduced(k))
            continue;
        if (instance().alternative(k).w > capacity(i))
            remove(j, i);
    }
}

