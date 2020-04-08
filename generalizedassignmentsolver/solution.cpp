#include "generalizedassignmentsolver/solution.hpp"

#include <iomanip>
#include <iostream>

using namespace generalizedassignmentsolver;

Solution::Solution(const Instance& instance):
    instance_(instance),
    x_(instance.item_number(), -1),
    agents_(instance.agent_number())
{ }

Solution::Solution(const Instance& instance, std::string filepath):
    instance_(instance),
    x_(instance.item_number(), -1),
    agents_(instance.agent_number())
{
    if (filepath.empty())
        return;
    std::ifstream file(filepath);
    if (!file.good()) {
        std::cerr << "\033[31m" << "ERROR, unable to open file \"" << filepath << "\"" << "\033[0m" << std::endl;
        return;
    }

    AgentIdx i = -1;
    for (ItemPos j = 0; j < instance.item_number(); ++j) {
        file >> i;
        set(j, i);
    }
}

Solution::Solution(const Instance& instance, const std::vector<std::vector<ItemIdx>>& agents):
    instance_(instance),
    x_(instance.item_number(), -1),
    agents_(instance.agent_number())
{
    for (AgentIdx i = 0; i < (AgentIdx)agents.size(); i++)
        for (ItemIdx j: agents[i])
            set(j, i);
}

Solution::Solution(const Solution& sol):
    instance_(sol.instance_),
    x_(sol.x_),
    agents_(sol.agents_),
    n_(sol.n_),
    total_cost_(sol.total_cost_),
    total_weight_(sol.total_weight_),
    total_overcapacity_(sol.total_overcapacity_),
    total_pcost_(sol.total_pcost_),
    comp_(sol.comp_)
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
            comp_               = sol.comp_;
        } else {
            clear();
            for (ItemIdx j = 0; j < instance().item_number(); ++j)
                set(j, sol.agent(j));
        }
    }
    return *this;
}

bool Solution::operator==(const Solution& sol)
{
    if (total_weight_ != sol.total_weight_ || total_cost_ != sol.total_cost_)
        return false;
    for (ItemIdx j = 0; j < instance().item_number(); ++j)
        if (x_[j] != sol.x_[j])
            return false;
    return true;
}

void Solution::set(ItemIdx j, AgentIdx i)
{
    assert(i >= -1 || i < instance().agent_number());
    assert(j >= 0 && j < instance().item_number());

    AgentIdx i_old = agent(j);
    if (i_old == i)
        return;

    if (i_old != -1) {
        const Alternative& a_old = instance().alternative(j, i_old);
        if (agents_[i_old].weight <= instance().capacity(i_old)) {
        } else if (agents_[i_old].weight - a_old.w >= instance().capacity(i_old)) {
            agents_[i_old].overcapacity -= a_old.w;
            total_overcapacity_         -= a_old.w;
            agents_[i_old].pcost        -= agents_[i_old].penalty * a_old.w;
            total_pcost_                -= agents_[i_old].penalty * a_old.w;
        } else {
            agents_[i_old].overcapacity -= agents_[i_old].weight - instance().capacity(i_old);
            total_overcapacity_         -= agents_[i_old].weight - instance().capacity(i_old);
            agents_[i_old].pcost        -= agents_[i_old].penalty * (agents_[i_old].weight - instance().capacity(i_old));
            total_pcost_                -= agents_[i_old].penalty * (agents_[i_old].weight - instance().capacity(i_old));
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
        const Alternative& a = instance().alternative(j, i);
        if (agents_[i].weight >= instance().capacity(i)) {
            agents_[i].overcapacity    += a.w;
            total_overcapacity_        += a.w;
            agents_[i].pcost           += agents_[i].penalty * a.w;
            total_pcost_               += agents_[i].penalty * a.w;
        } else if (agents_[i].weight + a.w <= instance().capacity(i)) {
        } else {
            Weight w = agents_[i].weight + a.w - instance().capacity(i);
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
        for (AgentIdx i = 0; i < m; ++i) {
            double r = (double)overcapacity(i) / instance().capacity(i);
            if (ratio_max < r)
                ratio_max = r;
        }
        double big_delta = (ratio_max > 0)? delta_inc / ratio_max: 0;
        for (AgentIdx i = 0; i < m; ++i)
            agents_[i].penalty *= (1 + big_delta * overcapacity(i) / instance().capacity(i));
    } else {
        for (AgentIdx i = 0; i < m; ++i)
            agents_[i].penalty *= (1 - delta_dec);
    }

    total_pcost_ = 0;
    //std::cout << ((inc)? "inc": "dec ");
    //std::cout << " alpha";
    for (AgentIdx i = 0; i < m; ++i) {
        //std::cout << " " << agents_[i].penalty;
        agents_[i].pcost = agents_[i].cost + agents_[i].penalty * agents_[i].overcapacity;
        total_pcost_    += agents_[i].pcost;
    }
    //std::cout << std::endl;
}

void Solution::update_penalties(PCost delta_inc)
{
    ItemIdx m = instance().agent_number();
    total_pcost_ = 0;
    for (AgentIdx i = 0; i < m; ++i) {
        agents_[i].penalty *= (1 + delta_inc);
        agents_[i].pcost = agents_[i].cost + agents_[i].penalty * agents_[i].overcapacity;
        total_pcost_    += agents_[i].pcost;
    }
}

void Solution::update_penalties(const std::vector<PCost>& penalty)
{
    ItemIdx m = instance().agent_number();
    total_pcost_ = 0;
    for (AgentIdx i = 0; i < m; ++i) {
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

std::string Solution::to_string(AgentIdx i)
{
    std::string s = "agent " + std::to_string(i) + ":";
    for (ItemIdx j = 0; j < instance().item_number(); ++j)
        if (agent(j) == i)
            s += " " + std::to_string(j);
    return s;
}

ItemIdx generalizedassignmentsolver::distance(const Solution& sol1, const Solution& sol2)
{
    ItemIdx dist = 0;
    for (ItemIdx j = 0; j < sol1.instance().item_number(); ++j)
        if (sol1.agent(j) != sol2.agent(j))
            dist++;
    return dist;
}

void Solution::write_cert(std::string filepath)
{
    if (filepath.empty())
        return;
    std::ofstream cert(filepath);
    if (!cert.good()) {
        std::cerr << "\033[31m" << "ERROR, unable to open file \"" << filepath << "\"" << "\033[0m" << std::endl;
        return;
    }

    std::copy(x_.begin(), x_.end(), std::ostream_iterator<AgentIdx>(cert, " "));
    cert << std::endl;
    cert.close();
}

std::ostream& generalizedassignmentsolver::operator<<(std::ostream& os, const Solution& solution)
{
    os <<  "n " << solution.item_number() << "/" << solution.instance().item_number()
        << " cost " << solution.cost()
        << " overcapacity " << solution.overcapacity()
        << std::endl;
    for (AgentIdx i = 0; i < solution.instance().agent_number(); ++i) {
        os << "agent " << i << " (" << solution.remaining_capacity(i) << "/" << solution.instance().capacity(i) <<  "):";
        for (ItemPos j = 0; j < solution.instance().item_number(); ++j)
            if (solution.agent(j) == i)
                os << " " << j;
        if (i != solution.instance().agent_number() - 1)
            os << std::endl;
    }
    return os;
}

bool generalizedassignmentsolver::compare(const Solution& sol_best, const Solution& sol_curr)
{
    if (!sol_curr.feasible())
        return false;
    if (!sol_best.feasible())
        return true;
    return sol_best.cost() > sol_curr.cost();
}

/*********************************** Output ***********************************/

Output::Output(const Instance& instance, Info& info): solution(instance)
{
    VER(info, std::left << std::setw(10) << "T (s)");
    VER(info, std::left << std::setw(12) << "UB");
    VER(info, std::left << std::setw(12) << "LB");
    VER(info, std::left << std::setw(10) << "GAP");
    VER(info, std::left << std::setw(10) << "GAP (%)");
    VER(info, "");
    VER(info, std::endl);
    print(info, std::stringstream(""));
}

std::string Output::ub_str() const
{
    return (!solution.feasible())? "inf": std::to_string(solution.cost());
}

std::string Output::lb_str() const
{
    return (lower_bound >= solution.instance().bound())? "inf": std::to_string(lower_bound);
}

std::string Output::gap_str() const
{
    if (lower_bound >= solution.instance().bound())
        return "0";
    if (lower_bound == 0 || !solution.feasible())
        return "inf";
    return std::to_string(solution.cost() - lower_bound);
}

double Output::gap() const
{
    if (lower_bound >= solution.instance().bound())
        return 0;
    if (lower_bound == 0 || !solution.feasible())
        return std::numeric_limits<double>::infinity();
    return (double)(10000 * (solution.cost() - lower_bound) / lower_bound) / 100;
}

void Output::print(Info& info, const std::stringstream& s) const
{
    double t = (double)std::round(info.elapsed_time() * 10000) / 10000;

    VER(info, std::left << std::setw(10) << t);
    VER(info, std::left << std::setw(12) << ub_str());
    VER(info, std::left << std::setw(12) << lb_str());
    VER(info, std::left << std::setw(10) << gap_str());
    VER(info, std::left << std::setw(10) << gap());
    VER(info, s.str() << std::endl);

    if (!info.output->onlywriteattheend)
        info.write_ini();
}

void Output::update_solution(const Solution& solution_new, const std::stringstream& s, Info& info)
{
    if (!compare(solution, solution_new))
        return;

    info.output->mutex_sol.lock();

    if (compare(solution, solution_new)) {
        solution = solution_new;
        print(info, s);

        info.output->sol_number++;
        double t = (double)std::round(info.elapsed_time() * 10000) / 10000;
        std::string sol_str = "Solution" + std::to_string(info.output->sol_number);
        PUT(info, sol_str, "Cost", solution.cost());
        PUT(info, sol_str, "Time", t);
        if (!info.output->onlywriteattheend)
            solution.write_cert(info.output->certfile);
    }

    info.output->mutex_sol.unlock();
}

void Output::update_lower_bound(Cost lower_bound_new, const std::stringstream& s, Info& info)
{
    if (lower_bound != -1 && lower_bound >= lower_bound_new)
        return;

    info.output->mutex_sol.lock();

    if (lower_bound == -1 || lower_bound < lower_bound_new) {
        lower_bound = lower_bound_new;
        print(info, s);

        info.output->bnd_number++;
        double t = (double)std::round(info.elapsed_time() * 10000) / 10000;
        std::string sol_str = "Bound" + std::to_string(info.output->bnd_number);
        PUT(info, sol_str, "Cost", lower_bound);
        PUT(info, sol_str, "Time", t);
        if (!info.output->onlywriteattheend)
            solution.write_cert(info.output->certfile);
    }

    info.output->mutex_sol.unlock();
}

Output& Output::algorithm_end(Info& info)
{
    time = (double)std::round(info.elapsed_time() * 10000) / 10000;

    PUT(info, "Solution", "Value", ub_str());
    PUT(info, "Bound", "Value", lb_str());
    PUT(info, "Solution", "Time", time);
    PUT(info, "Bound", "Time", time);
    VER(info, "---" << std::endl
            << "Solution: " << ub_str() << std::endl
            << "Bound: " << lb_str() << std::endl
            << "Gap: " << gap_str() << std::endl
            << "Gap (%): " << gap() << std::endl
            << "Time (s): " << time << std::endl);

    info.write_ini();
    solution.write_cert(info.output->certfile);
    return *this;
}

Cost generalizedassignmentsolver::algorithm_end(Cost lower_bound, Info& info)
{
    double t = (double)std::round(info.elapsed_time() * 10000) / 10000;
    PUT(info, "Bound", "Value", lower_bound);
    PUT(info, "Bound", "Time", t);
    VER(info, "---" << std::endl
            << "Bound: " << lower_bound << std::endl
            << "Time (s): " << t << std::endl);

    info.write_ini();
    return lower_bound;
}

