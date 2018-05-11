#include "gap/lib/subinstance.hpp"

SubInstance::SubInstance(const Instance& ins): instance_(ins)
{
    reduced_solution_ = new Solution(ins);
}

SubInstance& SubInstance::operator=(const SubInstance sub)
{
    if (this != &sub) {
        *reduced_solution_ = *sub.reduced_solution_;
    }
    return *this;
}

SubInstance::~SubInstance()
{
    delete reduced_solution();
}

////////////////////////////////////////////////////////////////////////////////

bool SubInstance::check_opt(Profit v) const
{
    if (instance().optimal_solution() != NULL
            && !sol_red_opt_
            && v != instance().optimum()) {
        std::cout << "V " << v << " != OPT " << instance().optimum() << std::endl;
        return false;
    }
    return true;
}

bool SubInstance::check_sopt(const Solution& sol) const
{
    if (!sol.feasible()) {
        std::cout << "NOT FEASIBLE" << std::endl;
        return false;
    }
    if (instance().optimal_solution() != NULL
                && !sol_red_opt_
                && sol.profit() != instance().optimum()) {
        std::cout << "V " << sol.profit() << " != OPT " << instance().optimum() << std::endl;
        return false;
    }
    return true;
}

bool SubInstance::check_ub(Profit p) const
{
    if (instance().optimal_solution() != NULL
            && !sol_red_opt_
            && p < instance().optimum()) {
        std::cout << "U " << p << " < OPT " << instance().optimum() << std::endl;
        return false;
    }
    return true;
}

bool SubInstance::check_lb(Profit v) const
{
    if (instance().optimal_solution() != NULL
            && !sol_red_opt_
            && v > instance().optimum()) {
        std::cout << "V " << v << " > OPT " << instance().optimum() << std::endl;
        return false;
    }
    return true;
}

bool SubInstance::check_sol(const Solution& sol) const
{
    if (!sol.feasible()) {
        std::cout << "NOT FEASIBLE" << std::endl;
        return false;
    }
    if (instance().optimal_solution() != NULL
            && !sol_red_opt_
            && sol.profit() > instance().optimum()) {
        std::cout << "V " << sol.profit() << " > OPT " << instance().optimum() << std::endl;
        return false;
    }
    return true;
}

std::ostream& operator<<(std::ostream& os, const SubInstance& sub)
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

