#pragma once

#include "gap/lib/instance.hpp"
#include "gap/lib/solution.hpp"

namespace gap
{

class SubInstance
{

public:

    SubInstance(const Instance& ins);
    SubInstance& operator=(const SubInstance sub);

    ~SubInstance();

    inline const Instance& instance() const { return instance_; }

    inline ItemIdx item_number() const { return instance().item_number()-reduced_solution()->item_number(); }
    inline ItemIdx alternative_number(AgentIdx i) const { return alternative_number_[i]; }
    inline Weight capacity(AgentIdx i) const { return reduced_solution()->remaining_capacity(i); }
    inline bool feasible() const { return feasible_; }

    /**
     * Apply variable reduction.
     */
    Solution* reduced_solution() const { return reduced_solution_; };
    inline int reduced(AltIdx k) const { return alternatives_[k]; }
    void set(ItemIdx j, AgentIdx i);
    void remove(AltIdx k);

private:

    const Instance& instance_;

    Solution* reduced_solution_;
    std::vector<int> alternatives_;
    std::vector<ItemIdx> alternative_number_;
    bool feasible_ = true;
};

std::ostream& operator<<(std::ostream& os, const SubInstance& sub);

}
