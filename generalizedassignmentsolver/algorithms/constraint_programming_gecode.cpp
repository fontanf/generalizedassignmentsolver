#if GECODE_FOUND

#include "generalizedassignmentsolver/algorithms/constraint_programming_gecode.hpp"

#include <gecode/int.hh>
#include <gecode/search.hh>
#include <gecode/minimodel.hh>
#include <gecode/gist.hh>

using namespace generalizedassignmentsolver;
using namespace Gecode;

/**
 * Useful links
 * https://www.gecode.org/doc-latest/MPG.pdf
 */

class GapGecode: public IntMinimizeSpace
{

public:

    GapGecode(const Instance& instance, ConstraintProgrammingGecodeParameters& p):
        instance_(instance),
        p_(p),
        xij0_(*this, instance.number_of_items() * instance.number_of_agents(), 0, 1),
        xj_(*this, instance.number_of_items(), 0, instance.number_of_agents() - 1),
        cj_(*this, instance.number_of_items()),
        load_(*this, instance.number_of_agents())
    {
        ItemIdx n = instance.number_of_items();
        AgentIdx m = instance.number_of_agents();

        // Channel xij == 1 <=> xj == i
        Matrix<BoolVarArgs> xij(xij0_, n, m);
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            channel(*this, xij.col(item_id), xj_[item_id]);
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            rel(*this, sum(xij.col(item_id)) == 1);

        // Weights
        weights_ = std::vector<IntArgs>(m, IntArgs(n));
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
            for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
                weights_[agent_id][item_id] = instance.weight(item_id, agent_id);

        // Costs
        costs_ = std::vector<IntArgs>(n, IntArgs(m));
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                costs_[item_id][agent_id] = instance.cost(item_id, agent_id);

        // Cost variables
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            cj_[item_id] = IntVar(
                    *this,
                    instance.item(item_id).minimum_cost,
                    instance.item(item_id).maximum_cost);
            element(*this, costs_[item_id], xj_[item_id], cj_[item_id]);
        }
        c_ = expr(*this, sum(cj_));

        // Load variables (and capacity constraints)
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
            load_[agent_id] = IntVar(*this, 0, instance.capacity(agent_id));
            linear(*this, weights_[agent_id], xij.row(agent_id), IRT_EQ, load_[agent_id]);
        }

        // Branch on the most efficient agent
        auto v = [](const Space& home, IntVar x, int j)
        {
            const Instance& instance = static_cast<const GapGecode&>(home).instance();
            AgentIdx agent_id_best = -1;
            double c_best = -1;
            for (IntVarValues i(x); i(); ++i) {
                double c = instance.profit(j, i.val()) / instance.weight(j, i.val());
                if (agent_id_best == -1 || c_best < c) {
                    agent_id_best = i.val();
                    c_best = c;
                }
            }
            return agent_id_best;
        };
        branch(*this, xj_, INT_VAR_NONE(), INT_VAL(v));
    }

    ~GapGecode() {  }

    GapGecode(GapGecode& s):
        IntMinimizeSpace(s), instance_(s.instance_), p_(s.p_), weights_(s.weights_), costs_(s.costs_)
    {
        xij0_.update(*this, s.xij0_);
        xj_.update(*this, s.xj_);
        cj_.update(*this, s.cj_);
        c_.update(*this, s.c_);
        load_.update(*this, s.load_);
    }

    virtual IntMinimizeSpace* copy(void) { return new GapGecode(*this); }

    virtual IntVar cost(void) const { return c_; }

    void print(void) const
    {
        std::cout << "Cost " << c_ << std::endl;
    }

    AgentIdx agent(ItemIdx item_id) const { return xj_[item_id].val(); }
    const Instance& instance() const { return instance_; }

private:

    const Instance& instance_;
    ConstraintProgrammingGecodeParameters& p_;
    std::vector<IntArgs> weights_;
    std::vector<IntArgs> costs_;
    BoolVarArray xij0_;
    IntVarArray xj_;
    IntVarArray cj_;
    IntVar c_;
    IntVarArray load_;

};

const Output generalizedassignmentsolver::constraintprogramming_gecode(
        const Instance& instance,
        const ConstraintProgrammingGecodeParameters& parameters)
{
    Output output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("Constraint programming (Gecode));
    algorithm_formatter.print_header();

    if (instance.number_of_items() == 0) {
        algorithm_formatter.end();
        return output;
    }

    GapGecode model(instance, parameters);
    //Gist::bab(&model);

    Search::Options options;

    // Time limit
    if (parameters.timer.remaining_time() != std::numeric_limits<double>::infinity()) {
        Search::Stop* stoptime = Search::Stop::time(parameters.timer.remaining_time() * 1000);
        options.stop = stoptime;
    }

    BAB<GapGecode> engine(&model, options);
    GapGecode* sol_ptr = NULL;
    Solution sol_best(instance);
    while ((sol_ptr = engine.next())) {
        Solution sol_curr(instance);
        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                item_id++) {
            sol_curr.set(item_id, sol_ptr->agent(item_id));
        }
        algorithm_formatter.update_solution(sol_curr, "");
        delete sol_ptr;
    }

    if (!parameters.timer.needs_to_end()) {
        Cost lb = (output.solution.feasible())? output.solution.cost(): instance.bound();
        algorithm_formatter.update_bound(lb, "");
    }

    algorithm_formatter.end();
    return output;
}

#endif

