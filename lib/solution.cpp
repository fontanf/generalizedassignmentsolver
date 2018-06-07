#include "gap/lib/solution.hpp"

using namespace gap;

Solution::Solution(const Instance& instance): instance_(instance),
    x_(std::vector<AgentIdx>(instance.item_number(), -1)),
    w_(std::vector<Weight>(instance.agent_number(), 0))
{ }

Solution::Solution(const Solution& solution):
    instance_(solution.instance()),
    k_(solution.item_number()),
    p_(solution.value()),
    w_tot_(solution.weight()),
    x_(solution.data()),
    w_(solution.weights())
{ }

Solution& Solution::operator=(const Solution& solution)
{
    if (this != &solution) {
        if (&solution.instance() == &instance()) {
            k_ = solution.item_number();
            p_ = solution.value();
            w_ = solution.weights();
            w_tot_ = solution.weight();
            x_ = solution.data();
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
        k_--;
    }

    if (i != -1) {
        const Alternative& a = ins.alternative(j, i);
        v_     += a.v;
        w_[i]  += a.w;
        w_tot_ += a.w;
        k_++;
    }

    x_[j] = i;
}

bool Solution::feasible() const
{
    if (k_ != instance().item_number())
        return false;
    for (AgentIdx i=0; i<instance().agent_number(); ++i)
        if (remaining_capacity(i) < 0)
            return false;
    return true;
}

void Solution::clear()
{
    k_ = 0;
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

std::ostream& gap::operator<<(std::ostream& os, const Solution& solution)
{
    const Instance& instance = solution.instance();
    for (ItemPos i=0; i<instance.item_number(); ++i)
        os << solution.data()[i] << std::endl;
    return os;
}

bool Solution::update(const Solution& sol)
{
    if (sol.item_number() != instance().item_number())
        return false;
    if (
               (instance().objective() ==  1 && sol.value() <= value())
            || (instance().objective() == -1 && sol.value() >= value()))
        return false;
    *this = sol;
    return true;
}

bool Solution::check_capacity() const
{
    for (AgentIdx i=0; i<instance().agent_number(); ++i)
        if (remaining_capacity(i) < 0)
            return false;
    return true;
}

std::string Solution::print_bin() const
{
    std::string s = "";
    for (ItemPos i=0; i<instance().item_number(); ++i)
        s += std::to_string(agent(i));
    return s;
}
