#include "gap/lib/instance.hpp"

#include "gap/lib/solution.hpp"

using namespace gap;

Instance::Instance(AgentIdx m, ItemIdx n)
{
    items_.reserve(n);
    t_.resize(m);
}

void Instance::set_capacity(const std::vector<Weight>& t)
{
    for (AgentIdx i=0; i<(AgentIdx)t.size(); ++i)
        set_capacity(i, t[i]);
}

void Instance::set_optimal_solution(Solution& sol)
{
    sol_opt_ = std::unique_ptr<Solution>(new Solution(sol));
}

ItemIdx Instance::add_item()
{
    ItemIdx j = items_.size();
    AltIdx k = alternatives_.size();
    items_.push_back({.j = j});
    for (AgentIdx i=0; i<agent_number(); ++i) {
        alternatives_.push_back({.k = k, .j = j, .i = i});
        items_[j].alt.push_back(k);
        k++;
    }
    return j;
}

void Instance::add_item(const std::vector<std::pair<Weight, Cost>>& a)
{
    ItemIdx j = add_item();
    for (AgentIdx i=0; i<(AgentIdx)a.size(); ++i)
        set_alternative(j, i, a[i].first, a[i].second);
}

void Instance::set_alternative(ItemIdx j, AgentIdx i, Weight w, Cost v)
{
    alternatives_[items_[j].alt[i]].w = w;
    alternatives_[items_[j].alt[i]].c = v;
    items_[j].w += w;
    items_[j].c += v;
    if (items_[j].i_best == -1 || items_[j].c_min > v) {
        items_[j].i_best = i;
        items_[j].c_min = v;
    }
    if (items_[j].c_max < v)
        items_[j].c_max = v;
    if (c_max_ < v)
        c_max_ = v;
}

Instance::Instance(std::string filepath, std::string format)
{
    std::ifstream file(filepath);
    if (!file.good()) {
        std::cerr << "\033[31m" << "ERROR, unable to open file \"" << filepath << "\"" << "\033[0m" << std::endl;
        assert(false);
        return;
    }

    if (format == "gap_beasley") {
        read_beasley(file);
    } else if (format == "gap_standard") {
        read_standard(file);
    } else {
        std::cerr << "ERROR, instance format unknown: " << format << std::endl;
    }

    file.close();
}

void Instance::read_beasley(std::ifstream& file)
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

Instance::Instance(const Instance& ins)
{
    items_ = ins.items_;
    alternatives_ = ins.alternatives_;
    t_ = ins.t_;
    sol_opt_ = (ins.sol_opt_ == NULL)? NULL: std::unique_ptr<Solution>(new Solution(*ins.sol_opt_));
}

Cost Instance::check(std::string cert_file)
{
    std::ifstream file(cert_file, std::ios_base::in);
    Solution sol(*this);
    AgentIdx i;
    for (ItemPos j=0; j<item_number(); ++j) {
        file >> i;
        if (i == -1)
            return -1;
        sol.set(j, i);
    }
    if (!sol.feasible())
        return -1;
    return sol.cost();
}

Cost Instance::optimum() const
{
    return optimal_solution()->cost();
}

std::ostream& gap::operator<<(std::ostream& os, const Alternative& alt)
{
    os << "(" << alt.i << " " << alt.c << " " << alt.w << " " << alt.efficiency() << ")";
    return os;
}

std::ostream& gap::operator<<(std::ostream& os, const Instance& ins)
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

