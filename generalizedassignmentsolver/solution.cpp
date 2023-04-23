#include "generalizedassignmentsolver/solution.hpp"

#include <iomanip>
#include <iostream>

using namespace generalizedassignmentsolver;

Solution::Solution(const Instance& instance):
    instance_(&instance),
    x_(instance.number_of_items(), -1),
    agents_(instance.number_of_agents())
{ }

Solution::Solution(
        const Instance& instance,
        std::string certificate_path):
    Solution(instance)
{
    if (certificate_path.empty())
        return;
    std::ifstream file(certificate_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + certificate_path + "\".");
    }

    AgentIdx agent_id = -1;
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        file >> agent_id;
        set(item_id, agent_id);
    }
}

Solution::Solution(const Instance& instance, const std::vector<std::vector<ItemIdx>>& agents):
    Solution(instance)
{
    for (AgentIdx agent_id = 0; agent_id < (AgentIdx)agents.size(); agent_id++)
        for (ItemIdx item_id: agents[agent_id])
            set(item_id, agent_id);
}

void Solution::set(
        ItemIdx item_id,
        AgentIdx agent_id)
{
    assert(agent_id >= -1 || agent_id < instance().number_of_agents());
    assert(item_id >= 0 && item_id < instance().number_of_items());

    AgentIdx agent_id_old = agent(item_id);
    if (agent_id_old == agent_id)
        return;

    if (agent_id_old != -1) {
        Weight w_old = instance().weight(item_id, agent_id_old);
        Cost   c_old = instance().cost(item_id, agent_id_old);
        if (agents_[agent_id_old].weight <= instance().capacity(agent_id_old)) {
        } else if (agents_[agent_id_old].weight - w_old >= instance().capacity(agent_id_old)) {
            agents_[agent_id_old].overcapacity -= w_old;
            total_overcapacity_                -= w_old;
        } else {
            agents_[agent_id_old].overcapacity -= agents_[agent_id_old].weight - instance().capacity(agent_id_old);
            total_overcapacity_                -= agents_[agent_id_old].weight - instance().capacity(agent_id_old);
        }
        agents_[agent_id_old].cost   -= c_old;
        total_cost_                  -= c_old;
        agents_[agent_id_old].weight -= w_old;
        total_weight_                -= w_old;
        number_of_items_--;
    }

    if (agent_id != -1) {
        Weight w = instance().weight(item_id, agent_id);
        Cost   c = instance().cost(item_id, agent_id);
        if (agents_[agent_id].weight >= instance().capacity(agent_id)) {
            agents_[agent_id].overcapacity    += w;
            total_overcapacity_               += w;
        } else if (agents_[agent_id].weight + w <= instance().capacity(agent_id)) {
        } else {
            Weight w_tmp = agents_[agent_id].weight + w - instance().capacity(agent_id);
            agents_[agent_id].overcapacity += w_tmp;
            total_overcapacity_            += w_tmp;
        }
        agents_[agent_id].cost   += c;
        total_cost_              += c;
        agents_[agent_id].weight += w;
        total_weight_            += w;
        number_of_items_++;
    }

    x_[item_id] = agent_id;
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

