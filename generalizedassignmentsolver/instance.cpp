#include "generalizedassignmentsolver/instance.hpp"

#include "generalizedassignmentsolver/solution.hpp"

using namespace generalizedassignmentsolver;

Instance::Instance(AgentIdx m):
    capacities_(m, 0)
{ }

void Instance::set_capacities(const std::vector<Weight>& t)
{
    for (AgentIdx i = 0; i < (AgentIdx)t.size(); ++i)
        set_capacity(i, t[i]);
}

void Instance::add_item()
{
    ItemIdx j = items_.size();
    items_.push_back({});
    items_[j].j = j;
    items_[j].alternatives.resize(number_of_agents());
    for (AgentIdx i = 0; i < number_of_agents(); ++i) {
        items_[j].alternatives[i].j = j;
        items_[j].alternatives[i].i = i;
    }
}

void Instance::add_item(const std::vector<std::pair<Weight, Cost>>& a)
{
    ItemIdx j = number_of_items();
    add_item();
    for (AgentIdx i = 0; i < (AgentIdx)a.size(); ++i)
        set_alternative(j, i, a[i].first, a[i].second);
}

void Instance::set_alternative(ItemIdx j, AgentIdx i, Weight weight, Cost cost)
{
    // Update the weight and the cost of the alternative.
    items_[j].alternatives[i].weight = weight;
    items_[j].alternatives[i].cost = cost;

    // Update the total weight of the item.
    items_[j].total_weight += weight;
    // Update the total cost of the item.
    items_[j].total_cost += cost;

    // Update the minimum costl of the item.
    if (items_[j].i_minimum_cost != -1 && items_[j].minimum_cost > cost)
        sum_of_minimum_costs_ -= items_[j].minimum_cost;
    if (items_[j].i_minimum_cost == -1 || items_[j].minimum_cost > cost) {
        items_[j].i_minimum_cost = i;
        items_[j].minimum_cost = cost;
        sum_of_minimum_costs_ += cost;
    }
    // Update the minimum weight of the item.
    if (items_[j].i_minimum_weight == -1 || items_[j].minimum_weight > weight) {
        items_[j].i_minimum_weight = i;
        items_[j].minimum_weight = weight;
    }
    // Update the maximum cost of the item.
    if (items_[j].maximum_cost < cost) {
        items_[j].i_maximum_cost = i;
        items_[j].maximum_cost = cost;
    }
    // Update to maximum weight of the item.
    if (items_[j].maximum_weight < weight) {
        items_[j].i_maximum_weight = i;
        items_[j].maximum_weight = weight;
    }

    // Update the maximum cost of the instance.
    if (maximum_cost_ < cost)
        maximum_cost_ = cost;
    // Update the maximum weight of the instance.
    if (maximum_weight_ < weight)
        maximum_weight_ = weight;
    // Update the total cost of the instance.
    total_cost_ += cost;
}

void Instance::set_optimal_solution(Solution& solution)
{
    optimal_solution_ = std::make_shared<Solution>(solution);
}

void Instance::clear()
{
    name_ = "";
    items_.clear();
    capacities_.clear();
    maximum_cost_ = 0;
    total_cost_ = 0;
    maximum_weight_ = 0;
    sum_of_minimum_costs_ = 0;
    optimal_solution_ = NULL;
}

Instance::Instance(std::string instance_path, std::string format):
    name_(instance_path)
{
    std::ifstream file(instance_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + instance_path + "\".");
    }

    if (format == "orlibrary") {
        read_orlibrary(file);
    } else if (format == "standard") {
        read_standard(file);
    } else {
        throw std::invalid_argument(
                "Unknown instance format \"" + format + "\".");
    }

    file.close();
}

void Instance::read_orlibrary(std::ifstream& file)
{
    ItemIdx n;
    AgentIdx m;
    file >> m >> n;

    capacities_.resize(m);
    items_.reserve(n);
    for (ItemPos j = 0; j < n; ++j)
        add_item();
    for (AgentIdx i = 0; i < m; ++i)
        for (ItemPos j = 0; j < n; ++j)
            file >> items_[j].alternatives[i].cost;
    for (AgentIdx i = 0; i < m; ++i)
        for (ItemPos j = 0; j < n; ++j)
            file >> items_[j].alternatives[i].weight;
    for (AgentIdx i = 0; i < m; ++i)
        for (ItemPos j = 0; j < n; ++j)
            set_alternative(j, i, items_[j].alternatives[i].weight, items_[j].alternatives[i].cost);

    Weight t = -1;
    for (AgentIdx i = 0; i < m; ++i) {
        file >> t;
        set_capacity(i, t);
    }
}

void Instance::read_standard(std::ifstream& file)
{
    ItemIdx n;
    AgentIdx m;
    file >> m >> n;

    capacities_.resize(m);
    Weight t = -1;
    for (AgentIdx i = 0; i < m; ++i) {
        file >> t;
        set_capacity(i, t);
    }

    items_.reserve(n);
    Weight w;
    Cost c;
    for (ItemPos j = 0; j < n; ++j) {
        add_item();
        for (AgentIdx i = 0; i < m; ++i) {
            file >> w >> c;
            set_alternative(j, i, w, c);
        }
    }
}

Cost Instance::optimum() const
{
    return optimal_solution()->cost();
}

std::ostream& Instance::print(
        std::ostream& os,
        int verbose) const
{
    if (verbose >= 1) {
        os
            << "Number of agents:         " << number_of_agents() << std::endl
            << "Number of items:          " << number_of_items() << std::endl
            ;
    }

    if (verbose >= 2) {
        os
            << std::endl
            << std::setw(12) << "Agent"
            << std::setw(12) << "Capacity"
            << std::endl
            << std::setw(12) << "-----"
            << std::setw(12) << "--------"
            << std::endl;
        for (AgentIdx i = 0; i < number_of_agents(); ++i) {
            os
                << std::setw(12) << i
                << std::setw(12) << capacity(i)
                << std::endl;
        }

        os
            << std::endl
            << std::setw(12) << "Item"
            << std::setw(12) << "Agent"
            << std::setw(12) << "Weight"
            << std::setw(12) << "Cost"
            << std::endl
            << std::setw(12) << "----"
            << std::setw(12) << "-----"
            << std::setw(12) << "------"
            << std::setw(12) << "----"
            << std::endl;
        for (ItemIdx j = 0; j < number_of_items(); ++j) {
            for (AgentIdx i = 0; i < number_of_agents(); ++i) {
                os
                    << std::setw(12) << j
                    << std::setw(12) << i
                    << std::setw(12) << weight(j, i)
                    << std::setw(12) << cost(j, i)
                    << std::endl;
            }
        }
    }

    return os;
}

void Instance::write(std::string instance_path)
{
    std::ofstream file(instance_path);
    if (!file.good()) {
        throw std::runtime_error(
                "Unable to open file \"" + instance_path + "\".");
    }

    file << number_of_agents() << " " << number_of_items() << std::endl;
    for (AgentIdx i = 0; i < number_of_agents(); ++i) {
        for (ItemIdx j = 0; j < number_of_items(); ++j)
            file << item(j).alternatives[i].cost << " ";
        file << std::endl;
    }
    for (AgentIdx i = 0; i < number_of_agents(); ++i) {
        for (ItemIdx j = 0; j < number_of_items(); ++j)
            file << item(j).alternatives[i].weight << " ";
        file << std::endl;
    }
    for (AgentIdx i = 0; i < number_of_agents(); ++i)
        file << capacity(i) << " ";
    file << std::endl;
    file.close();
}

void generalizedassignmentsolver::init_display(
        const Instance& instance,
        optimizationtools::Info& info)
{
    info.os()
            << "=====================================" << std::endl
            << "    Generalized Assignment Solver    " << std::endl
            << "=====================================" << std::endl
            << std::endl
            << "Instance" << std::endl
            << "--------" << std::endl;
    instance.print(info.os(), info.verbosity_level());
    info.os() << std::endl;
}
