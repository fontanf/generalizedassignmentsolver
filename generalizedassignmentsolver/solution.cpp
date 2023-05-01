#include "generalizedassignmentsolver/solution.hpp"

#include "optimizationtools/utils/utils.hpp"

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

std::ostream& Solution::print(
        std::ostream& os,
        int verbose) const
{
    if (verbose >= 1) {
        os
            << "Number of items:  " << optimizationtools::Ratio<ItemIdx>(number_of_items(), instance().number_of_items()) << std::endl
            << "Feasible:         " << feasible() << std::endl
            << "Cost:             " << cost() << std::endl
            ;
    }

    if (verbose >= 2) {
        os << std::endl
            << std::setw(12) << "Item"
            << std::setw(12) << "Agent"
            << std::endl
            << std::setw(12) << "----"
            << std::setw(12) << "-----"
            << std::endl;
        for (ItemIdx item_id = 0;
                item_id < instance().number_of_items();
                ++item_id) {
            os
                << std::setw(12) << item_id
                << std::setw(12) << agent(item_id)
                << std::endl;
        }
    }

    return os;
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
    bound(instance.combinatorial_relaxation())
{
    info.os()
        << std::setw(12) << "T (s)"
        << std::setw(12) << "UB"
        << std::setw(12) << "LB"
        << std::setw(12) << "GAP"
        << std::setw(12) << "GAP (%)"
        << std::setw(24) << "Comment"
        << std::endl
        << std::setw(12) << "-----"
        << std::setw(12) << "--"
        << std::setw(12) << "--"
        << std::setw(12) << "---"
        << std::setw(12) << "-------"
        << std::setw(24) << "-------"
        << std::endl;
    print(info, std::stringstream(""));
}

void Output::print(
        optimizationtools::Info& info,
        const std::stringstream& s) const
{
    std::string solution_value = optimizationtools::solution_value(
            optimizationtools::ObjectiveDirection::Minimize,
            solution.feasible(),
            solution.cost());
    double absolute_optimality_gap = optimizationtools::absolute_optimality_gap(
            optimizationtools::ObjectiveDirection::Minimize,
            solution.feasible(),
            solution.cost(),
            bound);
    double relative_optimality_gap = optimizationtools::relative_optimality_gap(
            optimizationtools::ObjectiveDirection::Minimize,
            solution.feasible(),
            solution.cost(),
            bound);
    double t = info.elapsed_time();
    std::streamsize precision = std::cout.precision();

    info.os()
        << std::setw(12) << std::fixed << std::setprecision(3) << t << std::defaultfloat << std::setprecision(precision)
        << std::setw(12) << solution_value
        << std::setw(12) << bound
        << std::setw(12) << absolute_optimality_gap
        << std::setw(12) << std::fixed << std::setprecision(2) << relative_optimality_gap * 100 << std::defaultfloat << std::setprecision(precision)
        << std::setw(24) << s.str()
        << std::endl;

    if (!info.output->only_write_at_the_end)
        info.write_json_output();
}

void Output::update_solution(
        const Solution& solution_new,
        const std::stringstream& s,
        optimizationtools::Info& info)
{
    info.lock();

    if (solution_new.feasible()
            && (!solution.feasible() || solution.cost() > solution_new.cost())) {
        solution = solution_new;
        print(info, s);

        std::string solution_value = optimizationtools::solution_value(
                optimizationtools::ObjectiveDirection::Minimize,
                solution.feasible(),
                solution.cost());
        double t = info.elapsed_time();

        info.output->number_of_solutions++;
        std::string sol_str = "Solution" + std::to_string(info.output->number_of_solutions);
        info.add_to_json(sol_str, "Value", solution_value);
        info.add_to_json(sol_str, "Time", t);
        info.add_to_json(sol_str, "String", s.str());
        if (!info.output->only_write_at_the_end) {
            info.write_json_output();
            solution.write(info.output->certificate_path);
        }
    }

    info.unlock();
}

void Output::update_bound(
        Cost bound_new,
        const std::stringstream& s,
        optimizationtools::Info& info)
{
    if (bound >= bound_new)
        return;

    info.lock();

    if (bound < bound_new) {
        bound = bound_new;
        print(info, s);

        double t = info.elapsed_time();

        info.output->number_of_bounds++;
        std::string sol_str = "Bound" + std::to_string(info.output->number_of_bounds);
        info.add_to_json(sol_str, "Bound", bound);
        info.add_to_json(sol_str, "Time", t);
        info.add_to_json(sol_str, "String", s.str());
        if (!info.output->only_write_at_the_end)
            info.write_json_output();
    }

    info.unlock();
}

Output& Output::algorithm_end(optimizationtools::Info& info)
{
    std::string solution_value = optimizationtools::solution_value(
            optimizationtools::ObjectiveDirection::Minimize,
            solution.feasible(),
            solution.cost());
    double absolute_optimality_gap = optimizationtools::absolute_optimality_gap(
            optimizationtools::ObjectiveDirection::Minimize,
            solution.feasible(),
            solution.cost(),
            bound);
    double relative_optimality_gap = optimizationtools::relative_optimality_gap(
            optimizationtools::ObjectiveDirection::Minimize,
            solution.feasible(),
            solution.cost(),
            bound);
    time = info.elapsed_time();

    info.add_to_json("Solution", "Value", solution_value);
    info.add_to_json("Bound", "Value", bound);
    info.add_to_json("Solution", "Time", time);
    info.add_to_json("Bound", "Time", time);
    info.os()
        << std::endl
        << "Final statistics" << std::endl
        << "----------------" << std::endl
        << "Value:                        " << solution_value << std::endl
        << "Bound:                        " << bound << std::endl
        << "Absolute optimality gap:      " << absolute_optimality_gap << std::endl
        << "Relative optimality gap (%):  " << relative_optimality_gap * 100 << std::endl
        << "Time (s):                     " << time << std::endl
        ;
    print_statistics(info);
    info.os() << std::endl
        << "Solution" << std::endl
        << "--------" << std::endl ;
    solution.print(info.os(), info.verbosity_level());

    info.write_json_output();
    solution.write(info.output->certificate_path);
    return *this;
}

