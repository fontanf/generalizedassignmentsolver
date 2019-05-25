#include "gap/lib/instance.hpp"

#include "gap/lib/solution.hpp"

using namespace gap;

Instance::Instance(AgentIdx m, ItemIdx n)
{
    items_.reserve(n);
    c_.resize(m);
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

void Instance::add_items(ItemIdx n)
{
    for (ItemPos j=0; j<n; ++j)
        add_item();
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
    if (items_[j].c_max < v) {
        items_[j].c_max = v;
    }
}

Instance::Instance(std::string filepath, std::string format)
{
    if        (format == "gap_beasley") {
        read_beasley(filepath);
    } else if (format == "gap_standard") {
        read_standard(filepath);
    } else {
        std::cout << format << ": Unknown instance format." << std::endl;
        exit(1);
    }
}

void Instance::read_beasley(std::string filepath)
{
    std::ifstream file(filepath, std::ios_base::in);
    ItemIdx n;
    AgentIdx m;
    file >> m >> n;

    c_.resize(m);
    items_.reserve(n);
    add_items(n);
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
        file >> c_[i];

    file.close();
}

void Instance::read_standard(std::string filepath)
{
    std::ifstream file(filepath, std::ios_base::in);
    ItemIdx  n;
    AgentIdx m;
    file >> m >> n;

    c_.resize(m);
    for (AgentIdx i=0; i<m; ++i)
        file >> c_[i];

    items_.reserve(n);
    add_items(n);
    for (ItemPos j=0; j<n; ++j)
        for (AgentIdx i=0; i<m; ++i)
            file >> alternatives_[items_[j].alt[i]].w >> alternatives_[items_[j].alt[i]].c;

    file.close();
}

void Instance::read_standard_solution(std::string filepath)
{
    sol_opt_ = std::unique_ptr<Solution>(new Solution(*this));
    std::ifstream file(filepath, std::ios_base::in);

    int x = 0;
    for (ItemPos j=0; j<item_number(); ++j) {
        file >> x;
        sol_opt_->set(j, x);
    }
}

Instance::Instance(const Instance& ins)
{
    items_ = ins.items_;
    alternatives_ = ins.alternatives_;
    c_ = ins.c_;
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

Instance::~Instance()
{

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

Solution gap::algorithm_end(const Solution& sol, Info& info)
{
    double t = info.elapsed_time();
    std::string feas = (sol.feasible())? "True": "False";
    PUT(info, "Solution.Cost", sol.cost());
    PUT(info, "Solution.Time", t);
    PUT(info, "Solution.Feasible", feas);
    VER(info, "---" << std::endl);
    VER(info, "Feasible: " << feas << std::endl);
    VER(info, "Cost: " << sol.cost() << std::endl);
    VER(info, "Time (s): " << t << std::endl);
    return sol;
}

void gap::algorithm_end(Cost lb, Info& info)
{
    double t = info.elapsed_time();
    PUT(info, "Bound.Cost", lb);
    PUT(info, "Bound.Time", t);
    VER(info, "---" << std::endl);
    VER(info, "Cost: " << lb << std::endl);
    VER(info, "Time (s): " << t << std::endl);
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

