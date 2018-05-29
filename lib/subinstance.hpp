#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

class SubInstance
{

public:

    SubInstance(const Instance& ins);
    SubInstance(const SubInstance& ins);
    SubInstance& operator=(const SubInstance sub);

    ~SubInstance();

    inline const Instance& instance() const { return instance_; }

    inline ItemIdx item_number() const { return instance().item_number()-reduced_solution()->item_number(); }
    inline ItemIdx agent_alternative_number(AgentIdx i) const { return agent_alternative_number_[i]; }
    inline AgentIdx item_alternative_number(ItemIdx j) const { return item_alternative_number_[j]; }
    inline Weight capacity(AgentIdx i) const { return reduced_solution()->remaining_capacity(i); }
    inline bool feasible() const { return feasible_; }

    /**
     * Apply variable reduction.
     */
    Solution* reduced_solution() const { return reduced_solution_; };
    inline int reduced(AltIdx k) const { return alternatives_[k]; }
    inline int reduced(ItemIdx j, AgentIdx i) const { return alternatives_[instance().alternative_index(j, i)]; }
    void set(ItemIdx j, AgentIdx i);
    void remove(ItemIdx j, AgentIdx i);

private:

    void remove_big_alt();
    void remove_big_alt(AgentIdx i);

    const Instance& instance_;

    Solution* reduced_solution_;
    std::vector<int> alternatives_;
    std::vector<ItemIdx> agent_alternative_number_;
    std::vector<AgentIdx> item_alternative_number_;
    bool feasible_ = true;
};

std::ostream& operator<<(std::ostream& os, const SubInstance& sub);

}
