#include "generalizedassignmentsolver/algorithms/milp.hpp"

#include "generalizedassignmentsolver/algorithm_formatter.hpp"

using namespace generalizedassignmentsolver;

namespace
{

struct Model
{
    /** Model. */
    mathoptsolverscmake::MilpModel model;

    /** x_{i, j} = 1 iff job j is assigned to agent i. */
    std::vector<std::vector<int>> x;
};

Model create_milp_model(
        const Instance& instance)
{
    Model model;

    /////////////////////////////
    // Variables and objective //
    /////////////////////////////

    model.model.objective_direction = mathoptsolverscmake::ObjectiveDirection::Minimize;

    // Variables x.
    model.x = std::vector<std::vector<int>>(instance.number_of_agents());
    for (AgentIdx agent_id = 0;
            agent_id < instance.number_of_agents();
            ++agent_id) {
        model.x[agent_id] = std::vector<int>(instance.number_of_items());
        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            model.x[agent_id][item_id] = model.model.variables_lower_bounds.size();
            model.model.variables_lower_bounds.push_back(0);
            model.model.variables_upper_bounds.push_back(1);
            model.model.variables_types.push_back(mathoptsolverscmake::VariableType::Binary);
            model.model.objective_coefficients.push_back(instance.cost(item_id, agent_id));
        }
    }

    /////////////////
    // Constraints //
    /////////////////

    // Every item needs to be assigned
    // sum_i xij = 1 for all j
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        model.model.constraints_starts.push_back(model.model.elements_variables.size());
        // Add row elements
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
            model.model.elements_variables.push_back(model.x[agent_id][item_id]);
            model.model.elements_coefficients.push_back(1.0);
        }
        model.model.constraints_lower_bounds.push_back(1);
        model.model.constraints_upper_bounds.push_back(1);
    }

    // Capacity constraint
    // sum_j wj xij <= ci
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
        model.model.constraints_starts.push_back(model.model.elements_variables.size());
        // Add row elements
        for (ItemIdx item_id = 0;
                item_id < instance.number_of_items();
                ++item_id) {
            model.model.elements_variables.push_back(model.x[agent_id][item_id]);
            model.model.elements_coefficients.push_back(instance.weight(item_id, agent_id));
        }
        // Add row bounds
        model.model.constraints_lower_bounds.push_back(-std::numeric_limits<double>::infinity());
        model.model.constraints_upper_bounds.push_back(instance.capacity(agent_id));
    }

    return model;
}

Solution retrieve_solution(
        const Instance& instance,
        const Model& model,
        const std::vector<double>& milp_solution)
{
    Solution solution(instance);
    for (ItemIdx item_id = 0;
            item_id < instance.number_of_items();
            ++item_id) {
        for (AgentIdx agent_id = 0;
                agent_id < instance.number_of_agents();
                ++agent_id) {
            if (std::round(milp_solution[model.x[agent_id][item_id]]) == 1)
                solution.set(item_id, agent_id);
        }
    }
    return solution;
}

#ifdef CBC_FOUND

class EventHandler: public CbcEventHandler
{

public:

    virtual CbcAction event(CbcEvent which_event);

    EventHandler(
            const Instance& instance,
            const MilpDisjunctiveParameters& parameters,
            const Model& milp_model,
            Output& output,
            AlgorithmFormatter& algorithm_formatter):
        CbcEventHandler(),
        instance_(instance),
        parameters_(parameters),
        milp_model_(milp_model),
        output_(output),
        algorithm_formatter_(algorithm_formatter) { }

    virtual ~EventHandler() { }

    EventHandler(const EventHandler &rhs):
        CbcEventHandler(rhs),
        instance_(rhs.instance_),
        parameters_(rhs.parameters_),
        milp_model_(rhs.milp_model_),
        output_(rhs.output_),
        algorithm_formatter_(rhs.algorithm_formatter_) { }

    virtual CbcEventHandler* clone() const { return new EventHandler(*this); }

private:

    const Instance& instance_;
    const MilpDisjunctiveParameters& parameters_;
    const Model& milp_model_;
    Output& output_;
    AlgorithmFormatter& algorithm_formatter_;

};

CbcEventHandler::CbcAction EventHandler::event(CbcEvent which_event)
{
    // Not in subtree.
    if ((model_->specialOptions() & 2048) != 0)
        return noAction;
    const CbcModel& cbc_model = *model_;

    int number_of_nodes = mathoptsolverscmake::get_number_of_nodes(cbc_model);

    // Retrieve solution.
    double milp_objective_value = mathoptsolverscmake::get_solution_value(cbc_model);
    if (!output_.solution.feasible()
            || output_.solution.cost() > milp_objective_value) {
        std::vector<double> milp_solution = mathoptsolverscmake::get_solution(cbc_model);
        Solution solution = retrieve_solution(instance_, milp_model_, milp_solution);
        algorithm_formatter_.update_solution(solution, "node " + std::to_string(number_of_nodes));
    }

    // Retrieve bound.
    Cost bound = std::ceil(mathoptsolverscmake::get_bound(cbc_model) - 1e5);
    algorithm_formatter_.update_bound(bound, "node " + std::to_string(number_of_nodes));

    // Check end.
    if (parameters_.timer.needs_to_end())
        return stop;

    return noAction;
}

#endif

#ifdef XPRESS_FOUND

struct XpressCallbackUser
{
    const Instance& instance;
    const MilpDisjunctiveParameters& parameters;
    Output& output;
    AlgorithmFormatter& algorithm_formatter;
};

void xpress_callback(
        XPRSprob xpress_model,
        void* user,
        int*)
{
    const XpressCallbackUser& d = *(const XpressCallbackUser*)(user);

    // Retrieve solution.
    double milp_objective_value = mathoptsolverscmake::get_solution_value(xpress_model);
    if (!d.output.solution.feasible()
            || d.output.solution.cost() > milp_objective_value) {
        std::vector<double> milp_solution = mathoptsolverscmake::get_solution(xpress_model);
        Solution solution = retrieve_solution(d.instance, milp_solution);
        d.algorithm_formatter.update_solution(solution, "");
    }

    // Retrieve bound.
    Cost bound = std::ceil(mathoptsolverscmake::get_bound(xpress_model) - 1e-5);
    d.algorithm_formatter.update_bound(bound, "");

    // Check end.
    if (d.parameters.timer.needs_to_end())
        XPRSinterrupt(xpress_model, XPRS_STOP_USER);
};

#endif

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

MilpOutput generalizedassignmentsolver::milp(
        const Instance& instance,
        const Solution* initial_solution,
        const MilpParameters& parameters)
{
    MilpOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("MILP");

    algorithm_formatter.print_header();

    Model milp_model = create_milp_model(instance);

    std::vector<double> milp_solution;
    double milp_bound = 0;

    if (parameters.solver == mathoptsolverscmake::SolverName::Cbc) {
#ifdef CBC_FOUND
        OsiCbcSolverInterface osi_solver;
        CbcModel cbc_model(osi_solver);
        mathoptsolverscmake::reduce_printout(cbc_model);
        mathoptsolverscmake::set_time_limit(cbc_model, parameters.timer.remaining_time());
        mathoptsolverscmake::load(cbc_model, milp_model.model);
        EventHandler cbc_event_handler(instance, parameters, milp_model, output, algorithm_formatter);
        cbc_model.passInEventHandler(&cbc_event_handler);
        mathoptsolverscmake::solve(cbc_model);
        milp_solution = mathoptsolverscmake::get_solution(cbc_model);
        milp_bound = mathoptsolverscmake::get_bound(cbc_model);
#else
        throw std::invalid_argument("");
#endif

    } else if (parameters.solver == mathoptsolverscmake::SolverName::Highs) {
#ifdef HIGHS_FOUND
        Highs highs;
        mathoptsolverscmake::reduce_printout(highs);
        mathoptsolverscmake::set_time_limit(highs, parameters.timer.remaining_time());
        mathoptsolverscmake::set_log_file(highs, "highs.log");
        mathoptsolverscmake::load(highs, milp_model.model);
        highs.setCallback([
                &instance,
                &parameters,
                &milp_model,
                &output,
                &algorithm_formatter](
                    const int,
                    const std::string& message,
                    const HighsCallbackOutput* highs_output,
                    HighsCallbackInput* highs_input,
                    void*)
                {
                    if (!highs_output->mip_solution.empty()) {
                        // Retrieve solution.
                        double milp_objective_value = highs_output->mip_primal_bound;
                        if (!output.solution.feasible()
                                || output.solution.cost() > milp_objective_value) {
                            Solution solution = retrieve_solution(instance, milp_model, highs_output->mip_solution);
                            algorithm_formatter.update_solution(solution, "node " + std::to_string(highs_output->mip_node_count));
                        }

                        // Retrieve bound.
                        Cost bound = std::ceil(highs_output->mip_dual_bound - 1e-5);
                        if (bound != std::numeric_limits<double>::infinity())
                            algorithm_formatter.update_bound(bound, "node " + std::to_string(highs_output->mip_node_count));
                    }

                    // Check end.
                    if (parameters.timer.needs_to_end())
                        highs_input->user_interrupt = 1;
                },
                nullptr);
        HighsStatus highs_status;
        highs_status = highs.startCallback(HighsCallbackType::kCallbackMipImprovingSolution);
        highs_status = highs.startCallback(HighsCallbackType::kCallbackMipInterrupt);
        mathoptsolverscmake::solve(highs);
        milp_solution = mathoptsolverscmake::get_solution(highs);
        milp_bound = mathoptsolverscmake::get_bound(highs);
#else
        throw std::invalid_argument("");
#endif

    } else if (parameters.solver == mathoptsolverscmake::SolverName::Xpress) {
#ifdef XPRESS_FOUND
        XPRSprob xpress_model;
        XPRScreateprob(&xpress_model);
        mathoptsolverscmake::set_time_limit(xpress_model, parameters.timer.remaining_time());
        mathoptsolverscmake::set_log_file(xpress_model, "xpress.log");
        mathoptsolverscmake::load(xpress_model, milp_model);
        //mathoptsolverscmake::write_mps(xpress_model, "kpc.mps");
        XpressCallbackUser xpress_callback_user{instance, parameters, output, algorithm_formatter};
        XPRSaddcbprenode(xpress_model, xpress_callback, (void*)&xpress_callback_user, 0);
        mathoptsolverscmake::solve(xpress_model);
        milp_solution = mathoptsolverscmake::get_solution(xpress_model);
        milp_bound = mathoptsolverscmake::get_bound(xpress_model);
        XPRSdestroyprob(xpress_model);
#else
        throw std::invalid_argument("");
#endif

    } else {
        throw std::invalid_argument("");
    }

    // Retrieve solution.
    Solution solution = retrieve_solution(instance, milp_model, milp_solution);
    algorithm_formatter.update_solution(solution, "");

    // Retrieve bound.
    algorithm_formatter.update_bound(milp_bound, "");

    algorithm_formatter.end();
    return output;
}
