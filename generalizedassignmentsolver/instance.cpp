#include "generalizedassignmentsolver/instance.hpp"

#include "generalizedassignmentsolver/solution.hpp"

using namespace generalizedassignmentsolver;

Instance::Instance(AgentIdx m)
{
    t_.resize(m);
}

void Instance::add_item()
{
    ItemIdx j = items_.size();
    AltIdx k = alternatives_.size();
    items_.push_back({.j = j});
    for (AgentIdx i=0; i<agent_number(); ++i) {
        alternatives_.push_back({.k = k, .j = j, .i = i});
        items_[j].alt.push_back(k);
        k++;
    }
}

void Instance::set_alternative(ItemIdx j, AgentIdx i, Weight w, Cost v)
{
    alternatives_[items_[j].alt[i]].w = w;
    alternatives_[items_[j].alt[i]].c = v;
    items_[j].w += w;
    items_[j].c += v;
    if (items_[j].i_cmin == -1 || items_[j].c_min > v) {
        items_[j].i_cmin = i;
        items_[j].c_min = v;
    }
    if (items_[j].i_wmin == -1 || items_[j].w_min > w) {
        items_[j].i_wmin = i;
        items_[j].w_min = w;
    }
    if (items_[j].c_max < v) {
        items_[j].i_cmax = i;
        items_[j].c_max = v;
    }
    if (items_[j].w_max < w) {
        items_[j].i_wmax = i;
        items_[j].w_max = w;
    }
    if (c_max_ < v)
        c_max_ = v;
    if (w_max_ < w)
        w_max_ = w;
    c_tot_ += v;
}

void Instance::set_capacities(const std::vector<Weight>& t)
{
    for (AgentIdx i=0; i<(AgentIdx)t.size(); ++i)
        set_capacity(i, t[i]);
}

void Instance::clear()
{
    name_ = "";
    items_.clear();
    alternatives_.clear();
    t_.clear();
    c_max_ = 0;
    c_tot_ = 0;
    sol_opt_ = NULL;
}

void Instance::add_item(const std::vector<std::pair<Weight, Cost>>& a)
{
    ItemIdx j = item_number();
    add_item();
    for (AgentIdx i=0; i<(AgentIdx)a.size(); ++i)
        set_alternative(j, i, a[i].first, a[i].second);
}

void Instance::set_optimal_solution(Solution& sol)
{
    sol_opt_ = std::make_unique<Solution>(sol);
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
        std::cerr << "ERROR, instance format unknown: " << format << std::endl;
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
    for (ItemPos j=0; j<n; ++j)
        add_item();
    for (AgentIdx i=0; i<m; ++i)
        for (ItemPos j=0; j<n; ++j)
            file >> alternatives_[alternative_index(j, i)].c;
    for (AgentIdx i=0; i<m; ++i)
        for (ItemPos j=0; j<n; ++j)
            file >> alternatives_[alternative_index(j, i)].w;
    for (AgentIdx i=0; i<m; ++i)
        for (ItemPos j=0; j<n; ++j)
            set_alternative(j, i, alternatives_[alternative_index(j, i)].w, alternatives_[alternative_index(j, i)].c);
    for (AgentIdx i=0; i<m; ++i)
        file >> t_[i];
}

void Instance::read_standard(std::ifstream& file)
{
    ItemIdx  n;
    AgentIdx m;
    file >> m >> n;

    t_.resize(m);
    for (AgentIdx i=0; i<m; ++i)
        file >> t_[i];

    items_.reserve(n);
    Weight w;
    Cost c;
    for (ItemPos j=0; j<n; ++j) {
        add_item();
        for (AgentIdx i=0; i<m; ++i) {
            file >> w >> c;
            set_alternative(j, i, w, c);
        }
    }
}

Instance::~Instance() {  }

Instance::Instance(const Instance& ins):
    name_(ins.name_),
    items_(ins.items_),
    alternatives_(ins.alternatives_),
    t_(ins.t_),
    c_max_(ins.c_max_),
    c_tot_(ins.c_tot_)
{
    if (ins.optimal_solution() != NULL) {
        sol_opt_ = std::make_unique<Solution>(*this);
        *sol_opt_ = *ins.optimal_solution();
    }
}

Instance& Instance::operator=(const Instance& ins)
{
    if (this != &ins) {
        name_         = ins.name_;
        items_        = ins.items_;
        alternatives_ = ins.alternatives_;
        t_            = ins.t_;
        c_max_        = ins.c_max_;
        c_tot_        = ins.c_tot_;

        if (ins.optimal_solution() != NULL) {
            sol_opt_ = std::make_unique<Solution>(*this);
            *sol_opt_ = *ins.optimal_solution();
        }
    }
    return *this;
}

Cost Instance::optimum() const
{
    return optimal_solution()->cost();
}

std::ostream& generalizedassignmentsolver::operator<<(std::ostream& os, const Alternative& alt)
{
    os << "(" << alt.i << " " << alt.c << " " << alt.w << " " << alt.efficiency() << ")";
    return os;
}

std::ostream& generalizedassignmentsolver::operator<<(std::ostream& os, const Instance& ins)
{
    os << "m " << ins.agent_number() << " n " << ins.item_number() << std::endl;
    os << "c";
    for (AgentIdx i=0; i<ins.agent_number(); ++i)
        os << " " << ins.capacity(i);
    os << std::endl;

    for (ItemPos j=0; j<ins.item_number(); ++j) {
        os << j << ": " << std::flush;
        for (AltIdx k: ins.item(j).alt)
            os << ins.alternative(k) << "; " << std::flush;
        os << std::endl;
    }
    return os;
}

void Instance::plot(std::string filename)
{
    std::ofstream file(filename);
    file << "w v" << std::endl;
    for (ItemIdx j=0; j<item_number(); ++j)
        for (AgentIdx i=0; i<agent_number(); ++i)
            file << alternative(j, i).w << " " << alternative(j, i).c << std::endl;
    file.close();
}

void Instance::write(std::string filename)
{
    std::ofstream file(filename);
    file << agent_number() << " " << item_number() << std::endl;
    for (AgentIdx i=0; i<agent_number(); ++i) {
        for (ItemIdx j=0; j<item_number(); ++j)
            file << alternative(j, i).c << " ";
        file << std::endl;
    }
    for (AgentIdx i=0; i<agent_number(); ++i) {
        for (ItemIdx j=0; j<item_number(); ++j)
            file << alternative(j, i).w << " ";
        file << std::endl;
    }
    for (AgentIdx i=0; i<agent_number(); ++i)
        file << capacity(i) << " ";
    file << std::endl;
    file.close();
}

