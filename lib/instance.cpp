#include "gap/lib/instance.hpp"

#include "gap/lib/solution.hpp"

using namespace gap;

Instance::Instance(ItemIdx n, AgentIdx m, int obj)
{
    items_.reserve(n);
    c_.resize(m);
    assert(obj == 1 || obj == -1);
    objective_ = obj;
}

ItemIdx Instance::add_item(Label l)
{
    ItemIdx j = items_.size();
    AltIdx k = alternatives_.size();
    items_.push_back(Item(j, l));
    for (AgentIdx i=0; i<agent_number(); ++i) {
        alternatives_.push_back(Alternative(k, j, i));
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

void Instance::set_alternative(ItemIdx j, AgentIdx i, Weight w, Profit p)
{
    alternatives_[items_[j].alt[i]].w = w;
    alternatives_[items_[j].alt[i]].p = p;
}

void Instance::set_weight(ItemIdx j, AgentIdx i, Weight w)
{
    alternatives_[items_[j].alt[i]].w = w;
}

void Instance::set_profit(ItemIdx j, AgentIdx i, Profit p)
{
    alternatives_[items_[j].alt[i]].p = p;
}

Instance::Instance(boost::filesystem::path filepath)
{
    if (!boost::filesystem::exists(filepath)) {
        std::cout << filepath << ": file not found." << std::endl;
        assert(false);
    }

    boost::filesystem::path FORMAT = filepath.parent_path() / "FORMAT.txt";
    if (!boost::filesystem::exists(FORMAT)) {
        std::cout << FORMAT << ": file not found." << std::endl;
        assert(false);
    }

    boost::filesystem::fstream file(FORMAT, std::ios_base::in);
    std::string format;
    std::getline(file, format);
    if        (format == "gap_beasley") {
        read_beasley(filepath);
    } else if (format == "gap_standard") {
        read_standard(filepath);
        boost::filesystem::path sol = filepath;
        sol += ".sol";
        if (boost::filesystem::exists(sol))
            read_standard_solution(sol);
    } else {
        std::cout << format << ": Unknown instance format." << std::endl;
        assert(false);
    }
}

void Instance::read_beasley(boost::filesystem::path filepath)
{
    boost::filesystem::fstream file(filepath, std::ios_base::in);
    ItemIdx n;
    AgentIdx m;
    objective_ = -1;
    file >> m >> n;

    c_.resize(m);
    items_.reserve(n);
    add_items(n);
    for (AgentIdx i=0; i<m; ++i)
        for (ItemPos j=0; j<n; ++j)
            file >> alternatives_[alternative_index(j, i)].p;
    for (AgentIdx i=0; i<m; ++i)
        for (ItemPos j=0; j<n; ++j)
            file >> alternatives_[alternative_index(j, i)].w;
    for (AgentIdx i=0; i<m; ++i)
        file >> c_[i];

    file.close();
}

void Instance::read_standard(boost::filesystem::path filepath)
{
    boost::filesystem::fstream file(filepath, std::ios_base::in);
    ItemIdx  n;
    AgentIdx m;
    file >> objective_ >> m >> n;

    c_.resize(m);
    for (AgentIdx i=0; i<m; ++i)
        file >> c_[i];

    items_.reserve(n);
    add_items(n);
    for (ItemPos j=0; j<n; ++j)
        for (AgentIdx i=0; i<m; ++i)
            file >> alternatives_[items_[j].alt[i]].w >> alternatives_[items_[j].alt[i]].p;

    file.close();
}

void Instance::read_standard_solution(boost::filesystem::path filepath)
{
    sol_opt_ = new Solution(*this);
    boost::filesystem::ifstream file(filepath, std::ios_base::in);

    int x = 0;
    for (ItemPos j=0; j<item_number(); ++j) {
        file >> x;
        sol_opt_->set(j, x);
    }
}

Instance Instance::adjust() const
{
    Instance ins(item_number(), agent_number(), 1);
    ins.add_items(item_number());
    for (AgentIdx i=0; i<ins.agent_number(); ++i)
        ins.set_capacity(i, capacity(i));

    Profit t = 0;
    if (objective() == -1)
        for (AltPos k=0; k<alternative_number(); ++k)
            if (alternative(k).p > t)
                t = alternative(k).p;

    std::cout << "T " << t << std::endl;

    for (ItemPos j=0; j<item_number(); ++j) {
        for (AgentIdx i=0; i<ins.agent_number(); ++i) {
            AltIdx k = alternative_index(j, i);
            Profit p = (objective() == -1)? t-alternative(k).p: alternative(k).p;
            p *= 100000;
            ins.set_alternative(j, i, alternative(k).w, p);
        }
    }
    return ins;
}

Profit Instance::check(boost::filesystem::path cert_file)
{
    if (!boost::filesystem::exists(cert_file))
        return -1;
    boost::filesystem::ifstream file(cert_file, std::ios_base::in);
    Solution sol(*this);
    AgentIdx i;
    for (ItemPos j=0; j<item_number(); ++j) {
        file >> i;
        if (i == -1)
            return -1;
        sol.set(j, i);
    }
    if (!sol.check_capacity())
        return -1;
    return sol.profit();
}

Profit Instance::optimum() const
{
    return optimal_solution()->profit();
}

std::ostream& gap::operator<<(std::ostream& os, const Alternative& alt)
{
    os << "(" << alt.i << " " << alt.w << " " << alt.p << " " << alt.efficiency() << ")";
    return os;
}

std::ostream& gap::operator<<(std::ostream& os, const Instance& ins)
{
    os <<  "N " << ins.item_number();
    os << " C";
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

////////////////////////////////////////////////////////////////////////////////

std::string Instance::print_lb(Profit lb) const
{
    return (optimal_solution() == NULL)?
        "LB " + std::to_string(lb):
        "LB " + std::to_string(lb) + " GAP " + std::to_string(optimum() - lb);
}

std::string Instance::print_ub(Profit ub) const
{
    return (optimal_solution() == NULL)?
        "UB " + std::to_string(ub):
        "UB " + std::to_string(ub) + " GAP " + std::to_string(ub - optimum());
}

std::string Instance::print_opt(Profit opt) const
{
    return (optimal_solution() != NULL && optimum() != opt)?
        "OPT " + std::to_string(opt) + " ERROR!":
        "OPT " + std::to_string(opt);
}

