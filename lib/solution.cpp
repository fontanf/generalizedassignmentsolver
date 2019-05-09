#include "gap/lib/solution.hpp"

#include <iomanip>

using namespace gap;

Solution::Solution(const Instance& ins): instance_(ins),
    x_(std::vector<AgentIdx>(ins.item_number(), -1)),
    agents_(std::vector<SolutionAgent>(ins.agent_number())),
    n_(0),
    total_cost_(0),
    total_weight_(0),
    total_overcapacity_(0),
    total_pcost_(0)
{ }

Solution::Solution(const Solution& sol):
    instance_(sol.instance_),
    x_(sol.x_), agents_(sol.agents_), n_(sol.n_),
    total_cost_(sol.total_cost_),
    total_weight_(sol.total_weight_),
    total_overcapacity_(sol.total_overcapacity_),
    total_pcost_(sol.total_pcost_)
{ }

Solution& Solution::operator=(const Solution& sol)
{
    if (this != &sol) {
        if (&sol.instance() == &instance()) {
            x_                  = sol.x_;
            agents_             = sol.agents_;
            n_                  = sol.n_;
            total_cost_         = sol.total_cost_;
            total_weight_       = sol.total_weight_;
            total_overcapacity_ = sol.total_overcapacity_;
            total_pcost_        = sol.total_pcost_;
        } else {
            clear();
            for (ItemIdx j=0; j<instance().item_number(); ++j)
                set(j, sol.agent(j));
        }
    }
    return *this;
}

bool Solution::operator==(const Solution& sol)
{
    if (total_weight_ != sol.total_weight_ || total_cost_ != sol.total_cost_)
        return false;
    for (ItemIdx j=0; j<instance().item_number(); ++j)
        if (x_[j] != sol.x_[j])
            return false;
    return true;
}

void Solution::set(ItemIdx j, AgentIdx i)
{
    const Instance& ins = instance();
    assert(i >= -1 || i < ins.agent_number());
    assert(j >= 0 && j < ins.item_number());

    AgentIdx i_old = agent(j);
    if (i_old == i)
        return;

    if (i_old != -1) {
        const Alternative& a_old = ins.alternative(j, i_old);
        if (agents_[i_old].weight <= ins.capacity(i_old)) {
        } else if (agents_[i_old].weight - a_old.w >= ins.capacity(i_old)) {
            agents_[i_old].overcapacity -= a_old.w;
            total_overcapacity_         -= a_old.w;
            agents_[i_old].pcost        -= agents_[i_old].penalty * a_old.w;
            total_pcost_                -= agents_[i_old].penalty * a_old.w;
        } else {
            agents_[i_old].overcapacity -= agents_[i_old].weight - ins.capacity(i_old);
            total_overcapacity_         -= agents_[i_old].weight - ins.capacity(i_old);
            agents_[i_old].pcost        -= agents_[i_old].penalty * (agents_[i_old].weight - ins.capacity(i_old));
            total_pcost_                -= agents_[i_old].penalty * (agents_[i_old].weight - ins.capacity(i_old));
        }
        agents_[i_old].cost   -= a_old.c;
        total_cost_           -= a_old.c;
        agents_[i_old].pcost  -= a_old.c;
        total_pcost_          -= a_old.c;
        agents_[i_old].weight -= a_old.w;
        total_weight_         -= a_old.w;
        n_--;
    }

    if (i != -1) {
        const Alternative& a = ins.alternative(j, i);
        if (agents_[i].weight >= ins.capacity(i)) {
            agents_[i].overcapacity    += a.w;
            total_overcapacity_        += a.w;
            agents_[i].pcost           += agents_[i].penalty * a.w;
            total_pcost_               += agents_[i].penalty * a.w;
        } else if (agents_[i].weight + a.w <= ins.capacity(i)) {
        } else {
            Weight w = agents_[i].weight + a.w - ins.capacity(i);
            agents_[i].overcapacity += w;
            total_overcapacity_     += w;
            agents_[i].pcost        += agents_[i].penalty * w;
            total_pcost_            += agents_[i].penalty * w;
        }
        agents_[i].cost   += a.c;
        total_cost_       += a.c;
        agents_[i].pcost  += a.c;
        total_pcost_      += a.c;
        agents_[i].weight += a.w;
        total_weight_     += a.w;
        n_++;
    }

    x_[j] = i;
}

void Solution::set(AltIdx k)
{
    set(instance().alternative(k).j, instance().alternative(k).i);
}

void Solution::update_penalties(bool inc, PCost delta_inc, PCost delta_dec)
{
    ItemIdx m = instance().agent_number();
    if (inc) {
        double ratio_max = 0.0;
        for (AgentIdx i=0; i<m; ++i) {
            double r = (double)overcapacity(i) / instance().capacity(i);
            if (ratio_max < r)
                ratio_max = r;
        }
        double big_delta = delta_inc / ratio_max;
        for (AgentIdx i=0; i<m; ++i)
            agents_[i].penalty *= (1 + big_delta * overcapacity(i) / instance().capacity(i));
    } else {
        for (AgentIdx i=0; i<m; ++i)
            agents_[i].penalty *= (1 - delta_dec);
    }

    total_pcost_ = 0;
    for (AgentIdx i=0; i<m; ++i) {
        agents_[i].pcost = agents_[i].cost + agents_[i].penalty * agents_[i].overcapacity;
        total_pcost_    += agents_[i].pcost;
    }
}

void Solution::update_penalties(PCost delta_inc)
{
    ItemIdx m = instance().agent_number();
    total_pcost_ = 0;
    for (AgentIdx i=0; i<m; ++i) {
        agents_[i].penalty *= (1 + delta_inc);
        agents_[i].pcost = agents_[i].cost + agents_[i].penalty * agents_[i].overcapacity;
        total_pcost_    += agents_[i].pcost;
    }
}

void Solution::update_penalties(const std::vector<PCost>& penalty)
{
    ItemIdx m = instance().agent_number();
    total_pcost_ = 0;
    for (AgentIdx i=0; i<m; ++i) {
        agents_[i].penalty = penalty[i];
        agents_[i].pcost = agents_[i].cost + agents_[i].penalty * agents_[i].overcapacity;
        total_pcost_    += agents_[i].pcost;
    }
}

void Solution::clear()
{
    std::fill(x_.begin(), x_.end(), -1);
    std::fill(agents_.begin(), agents_.end(), SolutionAgent{0, 0, 0, 0, 0});
    n_ = 0;
    total_cost_ = 0;
    total_weight_ = 0;
    total_overcapacity_ = 0;
    total_pcost_ = 0;
}

void Solution::write_cert(std::string file)
{
    if (file != "") {
        std::ofstream cert;
        cert.open(file);
        std::copy(x_.begin(), x_.end(), std::ostream_iterator<AgentIdx>(cert, " "));
        cert.close();
    }
}

std::ostream& gap::operator<<(std::ostream& os, const Solution& sol)
{
    os <<  "n " << sol.instance().item_number()
        << " cost " << sol.cost()
        << " overcapacity " << sol.overcapacity()
        << std::endl;
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

void Solution::update(const Solution& sol, Cost lb, const std::stringstream& s, Info& info)
{
    info.output->mutex_sol.lock();

    if (!feasible() || cost() > sol.cost()) {
        info.output->sol_number++;
        *this = sol;
        double t = std::round(info.elapsed_time());
        std::string sol_str = "Solution" + std::to_string(info.output->sol_number);
        PUT(info, sol_str + ".Cost", sol.cost());
        PUT(info, sol_str + ".Time", t);

        VER(info, std::left << std::setw(10) << t);
        VER(info, std::left << std::setw(12) << sol.cost());
        VER(info, std::left << std::setw(12) << lb);
        VER(info, std::left << std::setw(10) << sol.cost() - lb);
        VER(info, s.str() << std::endl);

        if (!info.output->onlywriteattheend) {
            info.write_ini();
            write_cert(info.output->certfile);
        }
    }

    info.output->mutex_sol.unlock();
}

void gap::init_display(Info& info)
{
    VER(info, std::left << std::setw(10) << "T (s)");
    VER(info, std::left << std::setw(12) << "UB");
    VER(info, std::left << std::setw(12) << "LB");
    VER(info, std::left << std::setw(10) << "GAP");
    VER(info, "");
    VER(info, std::endl);
}

std::string Solution::to_string(AgentIdx i)
{
    std::string s = "agent " + std::to_string(i) + ":";
    for (ItemIdx j=0; j<instance().item_number(); ++j)
        if (agent(j) == i)
            s += " " + std::to_string(j);
    return s;
}

/*********************************** Compare **********************************/

double SolutionCompare::value(const Solution& s)
{
    switch(id) {
    case 0:
        return s.cost();
    case 1:
        return s.cost() * s.weight();
    case 2:
        if (s.item_number() == 0)
            return 0;
        return (double)s.cost() / s.item_number();
    case 3:
        if (s.item_number() == 0)
            return 0;
        return (double)s.cost() / s.item_number() * (double)s.weight() / s.item_number();
    case 4:
        if (s.item_number() == 0)
            return 0;
        return (double)s.cost() / s.item_number() * (double)s.weight();
    }
    return 0;
}

bool SolutionCompare::operator()(const Solution& s1, const Solution& s2)
{
    return value(s1) < value(s2);
}

