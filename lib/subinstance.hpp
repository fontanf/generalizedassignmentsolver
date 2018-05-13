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

    inline ItemIdx job_number() const { return instance().item_number()-reduced_solution()->item_number(); }
    inline Weight capacity(AgentIdx i) const { return reduced_solution()->remaining_capacity(i); }
    inline bool feasible() const { return feasible_; }

    /**
     * Apply variable reduction.
     */
    void reduce(const Solution& sol, bool verbose = false);
    Solution* reduced_solution() const { return reduced_solution_; };
    inline int reduced(ItemIdx j) const { return reduced_solution()->agent(j); }
    void set(ItemIdx j, int b);

    bool check_opt(Profit p) const;
    bool check_sopt(const Solution& sol) const;
    bool check_ub(Profit p) const;
    bool check_lb(Profit p) const;
    bool check_sol(const Solution& sol) const;

private:

    bool update_deadlines(const std::vector<ItemIdx>& scheduled_jobs);

    const Instance& instance_;

    Solution* reduced_solution_;

    bool sol_red_opt_ = false;
    bool feasible_ = true;
};

std::ostream& operator<<(std::ostream& os, const SubInstance& sub);

}
