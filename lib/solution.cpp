#include "gap/lib/solution.hpp"

#include <iomanip>

using namespace gap;

Solution::Solution(const Instance& instance): instance_(instance),
    x_(std::vector<AgentIdx>(instance.item_number(), -1)),
    w_(std::vector<Weight>(instance.agent_number(), 0))
{ }

Solution::Solution(const Solution& solution):
    instance_(solution.instance_),
    n_(solution.n_),
    v_(solution.v_),
    w_tot_(solution.w_tot_),
    x_(solution.x_),
    w_(solution.w_)
{ }

Solution& Solution::operator=(const Solution& solution)
{
    if (this != &solution) {
        if (&solution.instance() == &instance()) {
            n_ = solution.n_;
            v_ = solution.v_;
            w_ = solution.w_;
            w_tot_ = solution.w_tot_;
            x_ = solution.x_;
        } else {
            assert(solution.instance().item_number() == instance().item_number());
            assert(solution.instance().agent_number() == instance().agent_number());
            clear();
            for (ItemIdx j=0; j<instance().item_number(); ++j)
                set(j, solution.agent(j));
        }
    }
    return *this;
}

AgentIdx Solution::agent(ItemIdx j) const
{
    assert(j >= 0 && j < instance().item_number());
    return x_[j];
}

void Solution::set(ItemIdx j, AgentIdx i)
{
    const Instance ins = instance();
    assert(i >= -1 || i < ins.agent_number());
    assert(j >= 0 && j < ins.item_number());

    AgentIdx i_old = agent(j);
    if (i_old == i)
        return;

    if (i_old != -1) {
        const Alternative& a_old = ins.alternative(j, i_old);
        v_        -= a_old.v;
        w_[i_old] -= a_old.w;
        w_tot_    -= a_old.w;
        n_--;
    }

    if (i != -1) {
        const Alternative& a = ins.alternative(j, i);
        v_     += a.v;
        w_[i]  += a.w;
        w_tot_ += a.w;
        n_++;
    }

    x_[j] = i;
}

void Solution::set(AltIdx k)
{
    set(instance().alternative(k).j, instance().alternative(k).i);
}

bool Solution::feasible() const
{
    if (n_ != instance().item_number())
        return false;
    for (AgentIdx i=0; i<instance().agent_number(); ++i)
        if (remaining_capacity(i) < 0)
            return false;
    return true;
}

void Solution::clear()
{
    n_ = 0;
    v_ = 0;
    w_tot_ = 0;
    std::fill(x_.begin(), x_.end(), -1);
    std::fill(w_.begin(), w_.end(), 0);
}

void Solution::write_cert(std::string file)
{
    if (file != "") {
        std::ofstream cert;
        cert.open(file);
        cert << *this;
        cert.close();
    }
}

std::ostream& gap::operator<<(std::ostream& os, const Solution& sol)
{
    os << "v " << sol.value() << std::endl;
    for (AgentIdx i=0; i<sol.instance().agent_number(); ++i) {
        os << "agent " << i << " (" << sol.remaining_capacity(i) << "/" << sol.instance().capacity(i) <<  "):";
        for (ItemPos j=0; j<sol.instance().item_number(); ++j)
            if (sol.agent(j) == i)
                os << " " << j;
        if (i != sol.instance().agent_number() - 1)
            os << std::endl;
    }
    return os;
}

bool Solution::check_capacity() const
{
    for (AgentIdx i=0; i<instance().agent_number(); ++i)
        if (remaining_capacity(i) < 0)
            return false;
    return true;
}

void Solution::update(const Solution& sol, Info& info, const std::stringstream& algorithm)
{
    info.output->mutex_sol.lock();

    if (!is_complete() || sol.value() < value()) {
        info.output->sol_number++;
        *this = sol;
        double t = info.elapsed_time();
        std::string sol_str = "Solution" + std::to_string(info.output->sol_number);
        PUT(info, sol_str + ".Value", sol.value());
        PUT(info, sol_str + ".Time", t);
        PUT(info, sol_str + ".Algorithm", algorithm.str());

        VER(info, std::left << std::setw(6) << info.output->sol_number);
        VER(info, std::left << std::setw(22) << algorithm.str());
        VER(info, std::left << std::setw(12) << sol.value());
        VER(info, t << std::endl);

        if (!info.output->onlywriteattheend) {
            info.write_ini();
            write_cert(info.output->certfile);
        }
    }

    info.output->mutex_sol.unlock();
}

/*********************************** Compare **********************************/

double SolutionCompare::value(const Solution& s)
{
    switch(id) {
    case 0:
        return s.value();
    case 1:
        return s.value() * s.weight();
    case 2:
        if (s.item_number() == 0)
            return 0;
        return (double)s.value() / s.item_number();
    case 3:
        if (s.item_number() == 0)
            return 0;
        return (double)s.value() / s.item_number() * (double)s.weight() / s.item_number();
    case 4:
        if (s.item_number() == 0)
            return 0;
        return (double)s.value() / s.item_number() * (double)s.weight();
    }
    return 0;
}

bool SolutionCompare::operator()(const Solution& s1, const Solution& s2)
{
    return value(s1) < value(s2);
}

