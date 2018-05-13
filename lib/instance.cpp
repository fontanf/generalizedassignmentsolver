#include "gap/lib/instance.hpp"

#include "gap/lib/solution.hpp"

using namespace gap;

Instance::Instance(const std::vector<Item>& items, const std::vector<Weight>& c):
    name_(""), format_(""), items_(items), c_(c) { }

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
    std::getline(file, format_);
    if        (format_ == "gap_beasley") {
        read_beasley(filepath);
    } else if (format_ == "gap_standard") {
        read_standard(filepath);
        boost::filesystem::path sol = filepath;
        sol += ".sol";
        if (boost::filesystem::exists(sol))
            read_standard_solution(sol);
    } else {
        std::cout << format_ << ": Unknown instance format." << std::endl;
        assert(false);
    }
}

void Instance::read_beasley(boost::filesystem::path filepath)
{
    name_ = filepath.stem().string();
    boost::filesystem::fstream file(filepath, std::ios_base::in);
    ItemIdx n;
    AgentIdx m;
    file >> m >> n;

    AltIdx alt_idx = 0;
    for (ItemPos j=0; j<n; ++j) {
        items_.push_back(Item(j));
        for (AgentIdx i=0; i<m; ++i) {
            alternatives_.push_back(Alternative(alt_idx, j, i));
            items_.back().alt.push_back(alt_idx);
            alt_idx++;
        }
    }

    for (AgentIdx i=0; i<m; ++i)
        for (ItemPos j=0; j<n; ++j)
            file >> alternatives_[items_[j].alt[i]].p;
    for (AgentIdx i=0; i<m; ++i)
        for (ItemPos j=0; j<n; ++j)
            file >> alternatives_[items_[j].alt[i]].w;
    c_.resize(m);
    for (AgentIdx i=0; i<m; ++i)
        file >> c_[i];

    file.close();
}

void Instance::read_standard(boost::filesystem::path filepath)
{
    name_ = filepath.stem().string();
    boost::filesystem::fstream file(filepath, std::ios_base::in);
    ItemIdx  n;
    AgentIdx m;
    file >> objective_ >> m >> n;

    AltIdx alt_idx = 0;
    for (ItemPos j=0; j<n; ++j) {
        items_.push_back(Item(j));
        for (AgentIdx i=0; i<m; ++i) {
            alternatives_.push_back(Alternative(alt_idx, j, i));
            items_.back().alt.push_back(alt_idx);
            alt_idx++;
        }
    }

    c_.resize(m);
    for (AgentIdx i=0; i<m; ++i)
        file >> c_[i];

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

void Instance::toggle_objective()
{
    if (t_ == 0)
        for (AltPos k=0; k<=alternative_number(); ++k)
            if (alternative(k).p > t_)
                t_ = alternative(k).p;

    for (AltPos k=0; k<=alternative_number(); ++k)
        alternatives_[k].p = t_ - alternatives_[k].p;

    objective_ *= -1;
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

std::ostream& operator<<(std::ostream& os, const Alternative& alt)
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
        os << j << ":" << std::flush;
        for (AltIdx k: ins.item(j).alt)
            os << "; " << ins.alternative(k);
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

