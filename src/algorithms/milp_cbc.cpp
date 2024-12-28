#include "generalizedassignmentsolver/algorithms/milp_cbc.hpp"

#include "generalizedassignmentsolver/algorithm_formatter.hpp"

using namespace generalizedassignmentsolver;

CoinLP::CoinLP(const Instance& instance)
{
    // Variables
    int number_of_columns = instance.number_of_agents() * instance.number_of_items();
    column_lower_bounds.resize(number_of_columns, 0);
    column_upper_bounds.resize(number_of_columns, 1);

    // Objective
    objective = std::vector<double>(number_of_columns);
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            objective[instance.number_of_agents() * item_id + agent_id] = instance.cost(item_id, agent_id);

    // Constraints
    int number_of_rows = 0; // will be increased each time we add a constraint
    std::vector<CoinBigIndex> row_starts;
    std::vector<int> number_of_elements_in_rows;
    std::vector<int> element_columns;
    std::vector<double> elements;

    // Every item needs to be assigned
    // sum_i xij = 1 for all j
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
        // Initialize new row
        row_starts.push_back(elements.size());
        number_of_elements_in_rows.push_back(0);
        number_of_rows++;
        // Add elements
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
            elements.push_back(1);
            element_columns.push_back(instance.number_of_agents() * item_id + agent_id);
            number_of_elements_in_rows.back()++;
        }
        // Add row bounds
        row_lower_bounds.push_back(1);
        row_upper_bounds.push_back(1);
    }

    // Capacity constraint
    // sum_j wj xij <= ci
    for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id) {
        // Initialize new row
        row_starts.push_back(elements.size());
        number_of_elements_in_rows.push_back(0);
        number_of_rows++;
        // Add row elements
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id) {
            elements.push_back(instance.weight(item_id, agent_id));
            element_columns.push_back(instance.number_of_agents() * item_id + agent_id);
            number_of_elements_in_rows.back()++;
        }
        // Add row bounds
        row_lower_bounds.push_back(0);
        row_upper_bounds.push_back(instance.capacity(agent_id));
    }

    // Create matrix
    row_starts.push_back(elements.size());
    matrix = CoinPackedMatrix(
            false,
            number_of_columns,
            number_of_rows,
            elements.size(),
            elements.data(),
            element_columns.data(),
            row_starts.data(),
            number_of_elements_in_rows.data());
}

//#include "coin/CbcHeuristicDiveCoefficient.hpp"
//#include "coin/CbcHeuristicDiveFractional.hpp"
//#include "coin/CbcHeuristicDiveGuided.hpp"
//#include "coin/CbcHeuristicDiveVectorLength.hpp"
//#include "coin/CbcLinked.hpp"
//#include "coin/CbcHeuristicGreedy.hpp"
//#include "coin/CbcHeuristicLocal.hpp"
#include "coin/CbcHeuristic.hpp"
//#include "coin/CbcHeuristicRINS.hpp"
//#include "coin/CbcHeuristicRENS.hpp"

//#include "coin/CglAllDifferent.hpp"
//#include "coin/CglClique.hpp"
//#include "coin/CglDuplicateRow.hpp"
//#include "coin/CglFlowCover.hpp"
//#include "coin/CglGomory.hpp"
//#include "coin/CglKnapsackCover.hpp"
//#include "coin/CglLandP.hpp"
//#include "coin/CglLiftAndProject.hpp"
//#include "coin/CglMixedIntegerRounding.hpp"
//#include "coin/CglMixedIntegerRounding2.hpp"
//#include "coin/CglOddHole.hpp"
//#include "coin/CglProbing.hpp"
//#include "coin/CglRedSplit.hpp"
//#include "coin/CglResidualCapacity.hpp"
//#include "coin/CglSimpleRounding.hpp"
//#include "coin/CglStored.hpp"
//#include "coin/CglTwomir.hpp"

/**
 * Useful links:
 * https://github.com/coin-or/Cgl/wiki
 * https://github.com/coin-or/Cbc/blob/master/Cbc/examples/sample2.cpp
 * https://github.com/coin-or/Cbc/blob/master/Cbc/examples/sample3.cpp
 * Callback https://github.com/coin-or/Cbc/blob/master/Cbc/examples/inc.cpp
 */

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Callback ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class EventHandler: public CbcEventHandler
{

public:

    virtual CbcAction event(CbcEvent which_event);

    EventHandler(
            const Instance& instance,
            const MilpCbcParameters& parameters,
            MilpCbcOutput& output,
            AlgorithmFormatter& algorithm_formatter):
        CbcEventHandler(),
        instance_(instance),
        parameters_(parameters),
        output_(output),
        algorithm_formatter_(algorithm_formatter) { }

    EventHandler(
            CbcModel* model,
            const Instance& instance,
            const MilpCbcParameters& parameters,
            MilpCbcOutput& output,
            AlgorithmFormatter& algorithm_formatter):
        CbcEventHandler(model),
        instance_(instance),
        parameters_(parameters),
        output_(output),
        algorithm_formatter_(algorithm_formatter) { }

    virtual ~EventHandler() { }

    EventHandler(const EventHandler &rhs):
        CbcEventHandler(rhs),
        instance_(rhs.instance_),
        parameters_(rhs.parameters_),
        output_(rhs.output_),
        algorithm_formatter_(rhs.algorithm_formatter_) { }

    EventHandler &operator=(const EventHandler &rhs)
    {
        if (this != &rhs) {
            CbcEventHandler::operator=(rhs);
            //this->instance_  = rhs.instance_;
            //this->parameters_ = rhs.parameters_;
            //this->output_ = rhs.output_;
            //this->algorithm_formatter_ = rhs.algorithm_formatter_;
        }
        return *this;
    }

    virtual CbcEventHandler* clone() const { return new EventHandler(*this); }

private:

    const Instance& instance_;
    const MilpCbcParameters& parameters_;
    MilpCbcOutput& output_;
    AlgorithmFormatter& algorithm_formatter_;

};

CbcEventHandler::CbcAction EventHandler::event(CbcEvent which_event)
{
    // Not in subtree.
    if ((model_->specialOptions() & 2048) != 0)
        return noAction;

    // Update output.
    output_.number_of_nodes = model_->getNodeCount();

    std::stringstream ss;
    ss << "node " << model_->getNodeCount();

    Cost lb = std::ceil(model_->getBestPossibleObjValue() - FFOT_TOL);
    algorithm_formatter_.update_bound(lb, ss.str());

    // No solution found.
    if ((which_event != solution && which_event != heuristicSolution))
        return noAction;

    OsiSolverInterface* origSolver = model_->solver();
    const OsiSolverInterface* pps = model_->postProcessedSolver(1);
    const OsiSolverInterface* solver = pps? pps: origSolver;

    if (!output_.solution.feasible() || output_.solution.cost() > solver->getObjValue() + 0.5) {
        const double* solution_cbc = solver->getColSolution();
        Solution solution(instance_);
        for (ItemIdx item_id = 0;
                item_id < instance_.number_of_items();
                ++item_id) {
            for (AgentIdx agent_id = 0;
                    agent_id < instance_.number_of_agents();
                    ++agent_id) {
                if (solution_cbc[instance_.number_of_agents() * item_id + agent_id] > 0.5)
                    solution.set(item_id, agent_id);
            }
        }

        algorithm_formatter_.update_solution(solution, ss.str());
    }

    // Check end.
    if (parameters_.timer.needs_to_end())
        return stop;

    return noAction;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const MilpCbcOutput generalizedassignmentsolver::milp_cbc(
        const Instance& instance,
        const MilpCbcParameters& parameters)
{
    MilpCbcOutput output(instance);
    AlgorithmFormatter algorithm_formatter(parameters, output);
    algorithm_formatter.start("MILP (CBC)");
    algorithm_formatter.print_header();

    if (instance.number_of_items() == 0) {
        algorithm_formatter.end();
        return output;
    }

    CoinLP problem(instance);

    OsiCbcSolverInterface solver1;

    // Reduce printout.
    solver1.getModelPtr()->setLogLevel(0);
    solver1.messageHandler()->setLogLevel(0);

    // Load problem.
    solver1.loadProblem(
            problem.matrix,
            problem.column_lower_bounds.data(),
            problem.column_upper_bounds.data(),
            problem.objective.data(),
            problem.row_lower_bounds.data(),
            problem.row_upper_bounds.data());

    // Mark integer.
    for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
        for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
            solver1.setInteger(instance.number_of_agents() * item_id + agent_id);

    // Pass data and solver to CbcModel.
    CbcModel model(solver1);

    // Callback.
    EventHandler event_handler(instance, parameters, output, algorithm_formatter);
    model.passInEventHandler(&event_handler);

    // Reduce printout.
    model.setLogLevel(0);
    model.solver()->setHintParam(OsiDoReducePrint, true, OsiHintTry);

    // Heuristics.
    //CbcHeuristicDiveCoefficient heuristic_divecoefficient(model);
    //model.addHeuristic(&heuristic_divecoefficient);
    //CbcHeuristicDiveFractional heuristic_divefractional(model);
    //model.addHeuristic(&heuristic_divefractional);
    //CbcHeuristicDiveGuided heuristic_diveguided(model);
    //model.addHeuristic(&heuristic_diveguided);
    //CbcHeuristicDiveVectorLength heuristic_divevetorlength(model);
    //model.addHeuristic(&heuristic_divevetorlength);
    //CbcHeuristicDynamic3 heuristic_dynamic3(model); // crash
    //model.addHeuristic(&heuristic_dynamic3);
    //CbcHeuristicFPump heuristic_fpump(model);
    //model.addHeuristic(&heuristic_fpump);
    //CbcHeuristicGreedyCover heuristic_greedycover(model);
    //model.addHeuristic(&heuristic_greedycover);
    //CbcHeuristicGreedyEquality heuristic_greedyequality(model);
    //model.addHeuristic(&heuristic_greedyequality);
    //CbcHeuristicLocal heuristic_local(model);
    //model.addHeuristic(&heuristic_local);
    //CbcHeuristicPartial heuristic_partial(model);
    //model.addHeuristic(&heuristic_partial);
    //CbcHeuristicRENS heuristic_rens(model);
    //model.addHeuristic(&heuristic_rens);
    //CbcHeuristicRINS heuristic_rins(model);
    //model.addHeuristic(&heuristic_rins);
    //CbcRounding heuristic_rounding(model);
    //model.addHeuristic(&heuristic_rounding);
    //CbcSerendipity heuristic_serendipity(model);
    //model.addHeuristic(&heuristic_serendipity);

    // Cuts.
    //CglClique cutgen_clique;
    //model.addCutGenerator(&cutgen_clique);
    //CglAllDifferent cutgen_alldifferent;
    //model.addCutGenerator(&cutgen_alldifferent);
    //CglDuplicateRow cutgen_duplicaterow;
    //model.addCutGenerator(&cutgen_duplicaterow);
    //CglFlowCover cutgen_flowcover;
    //model.addCutGenerator(&cutgen_flowcover);
    //CglGomory cutgen_gomory;
    //model.addCutGenerator(&cutgen_gomory);
    //CglKnapsackCover cutgen_knapsackcover;
    //model.addCutGenerator(&cutgen_knapsackcover);
    //CglLandP cutgen_landp;
    //model.addCutGenerator(&cutgen_landp);
    //CglLiftAndProject cutgen_liftandproject;
    //model.addCutGenerator(&cutgen_liftandproject);
    //CglMixedIntegerRounding cutgen_mixedintegerrounding;
    //model.addCutGenerator(&cutgen_mixedintegerrounding);
    //CglMixedIntegerRounding2 cutgen_mixedintegerrounding2;
    //model.addCutGenerator(&cutgen_mixedintegerrounding2);
    //CglOddHole cutgen_oddhole;
    //model.addCutGenerator(&cutgen_oddhole);
    //CglProbing cutgen_probing;
    //model.addCutGenerator(&cutgen_probing);
    //CglRedSplit cutgen_redsplit;
    //model.addCutGenerator(&cutgen_redsplit);
    //CglResidualCapacity cutgen_residualcapacity;
    //model.addCutGenerator(&cutgen_residualcapacity);
    //CglSimpleRounding cutgen_simplerounding;
    //model.addCutGenerator(&cutgen_simplerounding);
    //CglStored cutgen_stored;
    //model.addCutGenerator(&cutgen_stored);
    //CglTwomir cutgen_twomir;
    //model.addCutGenerator(&cutgen_twomir);

    // Set time limit.
    model.setMaximumSeconds(parameters.timer.remaining_time());

    // Add initial solution.
    std::vector<double> sol_init(instance.number_of_agents() * instance.number_of_items(), 0);
    if (parameters.initial_solution != NULL
            && parameters.initial_solution->feasible()) {
        for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
            for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                if (parameters.initial_solution->agent(item_id) == agent_id)
                    sol_init[instance.number_of_agents() * item_id + agent_id] = 1;
        model.setBestSolution(
                sol_init.data(),
                instance.number_of_agents() * instance.number_of_items(),
                parameters.initial_solution->cost());
    }

    // Stop af first improvment.
    if (parameters.stop_at_first_improvement)
        model.setMaximumSolutions(1);

    // Set maximum number of nodes.
    if (parameters.maximum_number_of_nodes != -1)
        model.setMaximumNodes(parameters.maximum_number_of_nodes);

    // Do complete search.
    model.branchAndBound();

    if (model.isProvenInfeasible()) {
        // Infeasible.

        // Update dual bound.
        algorithm_formatter.update_bound(
                instance.bound(),
                "");

    } else if (model.isProvenOptimal()) {
        // Optimal.

        // Update primal solution.
        if (!output.solution.feasible()
                || output.solution.cost() > model.getObjValue() + 0.5) {
            const double* solution_cbc = model.solver()->getColSolution();
            Solution solution(instance);
            for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
                for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                    if (solution_cbc[instance.number_of_agents() * item_id + agent_id] > 0.5)
                        solution.set(item_id, agent_id);
            algorithm_formatter.update_solution(
                    solution,
                    "");
        }
        // Update dual bound.
        algorithm_formatter.update_bound(
                output.solution.cost(),
                "");

    } else if (model.bestSolution() != NULL) {
        // Feasible solution found.

        // Update primal solution.
        if (!output.solution.feasible()
                || output.solution.cost() > model.getObjValue() + 0.5) {
            const double* solution_cbc = model.solver()->getColSolution();
            Solution solution(instance);
            for (ItemIdx item_id = 0; item_id < instance.number_of_items(); ++item_id)
                for (AgentIdx agent_id = 0; agent_id < instance.number_of_agents(); ++agent_id)
                    if (solution_cbc[instance.number_of_agents() * item_id + agent_id] > 0.5)
                        solution.set(item_id, agent_id);
            algorithm_formatter.update_solution(
                    solution,
                    "");
        }

        // Update dual bound.
        Cost lb = std::ceil(model.getBestPossibleObjValue() - FFOT_TOL);
        algorithm_formatter.update_bound(lb, "");

    } else {
        // No feasible solution found.

        // Update dual bound.
        Cost lb = std::ceil(model.getBestPossibleObjValue() - FFOT_TOL);
        algorithm_formatter.update_bound(lb, "");
    }

    // Update output.
    output.number_of_nodes = model.getNodeCount();

    algorithm_formatter.end();
    return output;
}
