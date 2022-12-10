#include "generalizedassignmentsolver/solution.hpp"

#include <iomanip>
#include <iostream>

using namespace generalizedassignmentsolver;

Solution::Solution(const Instance& instance):
    instance_(&instance),
    x_(instance.number_of_items(), -1),
    agents_(instance.number_of_agents())
{ }

Solution::Solution(const Instance& instance, std::string certificate_path):
    Solution(instance)
{
    if (certificate_path.empty())
        return;
    std::ifstream file(certificate_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + certificate_path + "\".");
    }

    AgentIdx i = -1;
    for (ItemPos j = 0; j < instance.number_of_items(); ++j) {
        file >> i;
        set(j, i);
    }
}

Solution::Solution(const Instance& instance, const std::vector<std::vector<ItemIdx>>& agents):
    Solution(instance)
{
    for (AgentIdx i = 0; i < (AgentIdx)agents.size(); i++)
        for (ItemIdx j: agents[i])
            set(j, i);
}

void Solution::set(ItemIdx j, AgentIdx i)
{
    assert(i >= -1 || i < instance().number_of_agents());
    assert(j >= 0 && j < instance().number_of_items());

    AgentIdx i_old = agent(j);
    if (i_old == i)
        return;

    if (i_old != -1) {
        Weight w_old = instance().weight(j, i_old);
        Cost   c_old = instance().cost(j, i_old);
        if (agents_[i_old].weight <= instance().capacity(i_old)) {
        } else if (agents_[i_old].weight - w_old >= instance().capacity(i_old)) {
            agents_[i_old].overcapacity -= w_old;
            total_overcapacity_         -= w_old;
            agents_[i_old].pcost        -= agents_[i_old].penalty * w_old;
            total_pcost_                -= agents_[i_old].penalty * w_old;
        } else {
            agents_[i_old].overcapacity -= agents_[i_old].weight - instance().capacity(i_old);
            total_overcapacity_         -= agents_[i_old].weight - instance().capacity(i_old);
            agents_[i_old].pcost        -= agents_[i_old].penalty * (agents_[i_old].weight - instance().capacity(i_old));
            total_pcost_                -= agents_[i_old].penalty * (agents_[i_old].weight - instance().capacity(i_old));
        }
        agents_[i_old].cost   -= c_old;
        total_cost_           -= c_old;
        agents_[i_old].pcost  -= c_old;
        total_pcost_          -= c_old;
        agents_[i_old].weight -= w_old;
        total_weight_         -= w_old;
        n_--;
    }

    if (i != -1) {
        Weight w = instance().weight(j, i);
        Cost   c = instance().cost(j, i);
        if (agents_[i].weight >= instance().capacity(i)) {
            agents_[i].overcapacity    += w;
            total_overcapacity_        += w;
            agents_[i].pcost           += agents_[i].penalty * w;
            total_pcost_               += agents_[i].penalty * w;
        } else if (agents_[i].weight + w <= instance().capacity(i)) {
        } else {
            Weight w_tmp = agents_[i].weight + w - instance().capacity(i);
            agents_[i].overcapacity += w_tmp;
            total_overcapacity_     += w_tmp;
            agents_[i].pcost        += agents_[i].penalty * w_tmp;
            total_pcost_            += agents_[i].penalty * w_tmp;
        }
        agents_[i].cost   += c;
        total_cost_       += c;
        agents_[i].pcost  += c;
        total_pcost_      += c;
        agents_[i].weight += w;
        total_weight_     += w;
        n_++;
    }

    x_[j] = i;
}

void Solution::update_penalties(bool inc, PCost delta_inc, PCost delta_dec)
{
    ItemIdx m = instance().number_of_agents();
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
    ItemIdx m = instance().number_of_agents();
    total_pcost_ = 0;
    for (AgentIdx i = 0; i < m; ++i) {
        agents_[i].penalty *= (1 + delta_inc);
        agents_[i].pcost = agents_[i].cost + agents_[i].penalty * agents_[i].overcapacity;
        total_pcost_    += agents_[i].pcost;
    }
}

void Solution::update_penalties(const std::vector<PCost>& penalty)
{
    ItemIdx m = instance().number_of_agents();
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
    for (ItemIdx j = 0; j < instance().number_of_items(); ++j)
        if (agent(j) == i)
            s += " " + std::to_string(j);
    return s;
}

ItemIdx generalizedassignmentsolver::distance(
        const Solution& solution_1,
        const Solution& solution_2)
{
    ItemIdx dist = 0;
    for (ItemIdx j = 0; j < solution_1.instance().number_of_items(); ++j)
        if (solution_1.agent(j) != solution_2.agent(j))
            dist++;
    return dist;
}

void Solution::write(std::string certificate_path)
{
    if (certificate_path.empty())
        return;
    std::ofstream file(certificate_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + certificate_path + "\".");
    }

    std::copy(x_.begin(), x_.end(), std::ostream_iterator<AgentIdx>(file, " "));
    file << std::endl;
    file.close();
}

std::ostream& generalizedassignmentsolver::operator<<(
        std::ostream& os,
        const Solution& solution)
{
    os <<  "n " << solution.number_of_items() << "/" << solution.instance().number_of_items()
        << " cost " << solution.cost()
        << " overcapacity " << solution.overcapacity()
        << std::endl;
    for (AgentIdx i = 0; i < solution.instance().number_of_agents(); ++i) {
        os << "agent " << i << " (" << solution.remaining_capacity(i) << "/" << solution.instance().capacity(i) <<  "):";
        for (ItemPos j = 0; j < solution.instance().number_of_items(); ++j)
            if (solution.agent(j) == i)
                os << " " << j;
        if (i != solution.instance().number_of_agents() - 1)
            os << std::endl;
    }
    return os;
}

bool generalizedassignmentsolver::compare(
        const Solution& best_solution,
        const Solution& current_solution)
{
    if (!current_solution.feasible())
        return false;
    if (!best_solution.feasible())
        return true;
    return best_solution.cost() > current_solution.cost();
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Output ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Output::Output(
        const Instance& instance,
        optimizationtools::Info& info):
    solution(instance),
    lower_bound(instance.combinatorial_relaxation())
{
    info.os()
            << std::setw(10) << "T (s)"
            << std::setw(14) << "UB"
            << std::setw(14) << "LB"
            << std::setw(14) << "GAP"
            << std::setw(10) << "GAP (%)"
            << std::setw(24) << "Comment"
            << std::endl
            << std::setw(10) << "-----"
            << std::setw(14) << "--"
            << std::setw(14) << "--"
            << std::setw(14) << "---"
            << std::setw(10) << "-------"
            << std::setw(24) << "-------"
            << std::endl;
    print(info, std::stringstream(""));
    info.reset_time();
}

bool Output::optimal() const
{
    return ((solution.feasible() && solution.cost() == lower_bound)
        || (lower_bound >= solution.instance().bound()));
}

std::string Output::upper_bound_string() const
{
    return (!solution.feasible())? "inf": std::to_string(solution.cost());
}

std::string Output::lower_bound_string() const
{
    return (lower_bound >= solution.instance().bound())? "inf": std::to_string(lower_bound);
}

std::string Output::gap_string() const
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
    return (double)(solution.cost() - lower_bound) / lower_bound * 100;
}

void Output::print(
        optimizationtools::Info& info,
        const std::stringstream& s) const
{
    double t = info.elapsed_time();
    std::streamsize precision = std::cout.precision();

    info.os()
            << std::setw(10) << std::fixed << std::setprecision(3) << t << std::defaultfloat << std::setprecision(precision)
            << std::setw(14) << upper_bound_string()
            << std::setw(14) << lower_bound_string()
            << std::setw(14) << gap_string()
            << std::setw(10) << std::fixed << std::setprecision(2) << gap() << std::defaultfloat << std::setprecision(precision)
            << std::setw(24) << s.str() << std::endl;

    if (!info.output->only_write_at_the_end)
        info.write_json_output();
}

void Output::update_solution(
        const Solution& solution_new,
        const std::stringstream& s,
        optimizationtools::Info& info)
{
    if (!compare(solution, solution_new))
        return;

    info.lock();

    if (compare(solution, solution_new)) {
        solution = solution_new;
        print(info, s);

        info.output->number_of_solutions++;
        double t = (double)std::round(info.elapsed_time() * 10000) / 10000;
        std::string sol_str = "Solution" + std::to_string(info.output->number_of_solutions);
        info.add_to_json(sol_str, "Value", solution.cost());
        info.add_to_json(sol_str, "Time", t);
        if (!info.output->only_write_at_the_end)
            solution.write(info.output->certificate_path);
    }

    info.unlock();
}

void Output::update_lower_bound(
        Cost lower_bound_new,
        const std::stringstream& s,
        optimizationtools::Info& info)
{
    if (lower_bound != -1 && lower_bound >= lower_bound_new)
        return;

    info.lock();

    if (lower_bound == -1 || lower_bound < lower_bound_new) {
        lower_bound = lower_bound_new;
        print(info, s);

        info.output->number_of_bounds++;
        double t = (double)std::round(info.elapsed_time() * 10000) / 10000;
        std::string sol_str = "Bound" + std::to_string(info.output->number_of_bounds);
        info.add_to_json(sol_str, "Value", lower_bound);
        info.add_to_json(sol_str, "Time", t);
        if (!info.output->only_write_at_the_end)
            solution.write(info.output->certificate_path);
    }

    info.unlock();
}

Output& Output::algorithm_end(optimizationtools::Info& info)
{
    time = (double)std::round(info.elapsed_time() * 10000) / 10000;

    info.add_to_json("Solution", "Value", upper_bound_string());
    info.add_to_json("Bound", "Value", lower_bound_string());
    info.add_to_json("Solution", "Time", time);
    info.add_to_json("Bound", "Time", time);
    info.os()
            << std::endl
            << "Final statistics" << std::endl
            << "----------------" << std::endl
            << "Value:                    " << upper_bound_string() << std::endl
            << "Bound:                    " << lower_bound_string() << std::endl
            << "Gap:                      " << gap_string() << std::endl
            << "Gap (%):                  " << gap() << std::endl
            << "Time (s):                 " << time << std::endl;

    info.write_json_output();
    solution.write(info.output->certificate_path);
    return *this;
}

Cost generalizedassignmentsolver::algorithm_end(
        Cost lower_bound,
        optimizationtools::Info& info)
{
    double t = (double)std::round(info.elapsed_time() * 10000) / 10000;
    info.add_to_json("Bound", "Value", lower_bound);
    info.add_to_json("Bound", "Time", t);
    info.os()
            << std::endl
            << "Final statistics" << std::endl
            << "----------------" << std::endl
            << "Bound:                    " << lower_bound << std::endl
            << "Time (s):                 " << t << std::endl;

    info.write_json_output();
    return lower_bound;
}

