#include "generalizedassignmentsolver/instance.hpp"

#include "generalizedassignmentsolver/solution.hpp"

using namespace generalizedassignmentsolver;

Instance::Instance(AgentIdx m)
{
    t_.resize(m);
}

void Instance::set_capacities(const std::vector<Weight>& t)
{
    for (AgentIdx i = 0; i < (AgentIdx)t.size(); ++i)
        set_capacity(i, t[i]);
}

void Instance::clear()
{
    name_ = "";
    items_.clear();
    t_.clear();
    c_max_ = 0;
    c_tot_ = 0;
    sol_opt_ = NULL;
}

void Instance::add_item(const std::vector<std::pair<Weight, Cost>>& a)
{
    ItemIdx j = number_of_items();
    add_item();
    for (AgentIdx i = 0; i < (AgentIdx)a.size(); ++i)
        set_alternative(j, i, a[i].first, a[i].second);
}

void Instance::set_optimal_solution(Solution& solution)
{
    sol_opt_ = std::make_unique<Solution>(solution);
}

Instance::Instance(std::string filepath, std::string format): name_(filepath)
{
    std::ifstream file(filepath);
    if (!file.good()) {
        std::cerr << "\033[31m" << "ERROR, unable to open file \"" << filepath << "\"" << "\033[0m" << std::endl;
        assert(false);
        return;
    }

    if (format == "orlibrary") {
        read_orlibrary(file);
    } else if (format == "standard") {
        read_standard(file);
    } else {
        std::cerr << "\033[31m" << "ERROR, unknown instance format \"" << format << "\"" << "\033[0m" << std::endl;
    }

    file.close();
}

void Instance::read_orlibrary(std::ifstream& file)
{
    ItemIdx n;
    AgentIdx m;
    file >> m >> n;

    t_.resize(m);
    items_.reserve(n);
    for (ItemPos j = 0; j < n; ++j)
        add_item();
    for (AgentIdx i = 0; i < m; ++i)
        for (ItemPos j = 0; j < n; ++j)
            file >> items_[j].alternatives[i].c;
    for (AgentIdx i = 0; i < m; ++i)
        for (ItemPos j = 0; j < n; ++j)
            file >> items_[j].alternatives[i].w;
    for (AgentIdx i = 0; i < m; ++i)
        for (ItemPos j = 0; j < n; ++j)
            set_alternative(j, i, items_[j].alternatives[i].w, items_[j].alternatives[i].c);
    for (AgentIdx i = 0; i < m; ++i)
        file >> t_[i];
}

void Instance::read_standard(std::ifstream& file)
{
    ItemIdx  n;
    AgentIdx m;
    file >> m >> n;

    t_.resize(m);
    for (AgentIdx i = 0; i < m; ++i)
        file >> t_[i];

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

Instance::~Instance() {  }

Instance::Instance(const Instance& instance):
    name_(instance.name_),
    items_(instance.items_),
    t_(instance.t_),
    c_max_(instance.c_max_),
    c_tot_(instance.c_tot_)
{
    if (instance.optimal_solution() != NULL) {
        sol_opt_ = std::make_unique<Solution>(*this);
        *sol_opt_ = *instance.optimal_solution();
    }
}

Instance& Instance::operator=(const Instance& instance)
{
    if (this != &instance) {
        name_         = instance.name_;
        items_        = instance.items_;
        t_            = instance.t_;
        c_max_        = instance.c_max_;
        c_tot_        = instance.c_tot_;

        if (instance.optimal_solution() != NULL) {
            sol_opt_ = std::make_unique<Solution>(*this);
            *sol_opt_ = *instance.optimal_solution();
        }
    }
    return *this;
}

Cost Instance::optimum() const
{
    return optimal_solution()->cost();
}

std::ostream& generalizedassignmentsolver::operator<<(std::ostream& os, const Alternative& alternative)
{
    os << "(" << alternative.i
        << " " << alternative.c
        << " " << alternative.w
        << " " << alternative.efficiency()
        << ")";
    return os;
}

std::ostream& generalizedassignmentsolver::operator<<(std::ostream& os, const Instance& instance)
{
    os << "m " << instance.number_of_agents() << " n " << instance.number_of_items() << std::endl;
    os << "c";
    for (AgentIdx i = 0; i < instance.number_of_agents(); ++i)
        os << " " << instance.capacity(i);
    os << std::endl;

    for (ItemPos j = 0; j < instance.number_of_items(); ++j) {
        os << j << ": " << std::flush;
        os << std::endl;
    }
    return os;
}

void Instance::write(std::string filename)
{
    std::ofstream file(filename);
    file << number_of_agents() << " " << number_of_items() << std::endl;
    for (AgentIdx i = 0; i < number_of_agents(); ++i) {
        for (ItemIdx j = 0; j < number_of_items(); ++j)
            file << item(j).alternatives[i].c << " ";
        file << std::endl;
    }
    for (AgentIdx i = 0; i < number_of_agents(); ++i) {
        for (ItemIdx j = 0; j < number_of_items(); ++j)
            file << item(j).alternatives[i].w << " ";
        file << std::endl;
    }
    for (AgentIdx i = 0; i < number_of_agents(); ++i)
        file << capacity(i) << " ";
    file << std::endl;
    file.close();
}

