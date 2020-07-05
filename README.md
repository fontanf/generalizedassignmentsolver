# GeneralizedAssignmentSolver

A solver for the Generalized Assignment Problem.

This problem is interesting because many different optimization methods can and have been applied to solve it (Branch-and-cut, Branch-and-price, Branch-and-relax, Local search, Constraint programming, Column generation heuristics...). Thus, the main goal of this repository is for me to have reference implementations for classical algorithms and optimization solvers.

## Implemented algorithms

### Lower bounds

- Linear relaxation
  - solved with CLP `-a linrelax_clp` :heavy_check_mark:
  - solved with Gurobi `-a linrelax_gurobi` :heavy_check_mark:

- Lagrangian relaxation of knapsack constraints
  - solved with volume method `-a lagrelax_knapsack_volume` :heavy_check_mark:
  - solved with L-BFGS method `-a lagrelax_knapsack_lbfgs` :heavy_check_mark:

- Lagrangian relaxation of assignment constraints
  - solved with volume method `-a lagrelax_assignment_volume` :heavy_check_mark:
  - solved with L-BFGS method `-a lagrelax_assignment_lbfgs` :heavy_check_mark:

- Column generation `-a columngeneration --lp-solver clp` :heavy_check_mark: `-a columngeneration --lp-solver cplex` :heavy_check_mark:

### Upper bounds

Polynomial algorithms from "Generalized Assignment Problems" (Martello et al., 1992), options `f cij` `f wij` `f cij*wij` `f -pij/wij` `f wij/ti`:
- Basic greedy `-a "greedy -f wij"` :heavy_check_mark:
- Greedy with regret measure `-a "greedyregret -f wij"` :heavy_check_mark:
- MTHG, basic greedy (+ n shifts) `-a "mthg -f wij"` :heavy_check_mark:
- MTHG, greedy with regret measure (+ n shifts) `-a "mthgregret -f wij"` :heavy_check_mark:

Classical meta-heuristics based on shift-swap neighborhood optimized for large instances:
- Local search `-a localsearch` :heavy_check_mark:
- Tabu search `-a tabusearch` :heavy_check_mark:
- Simulated annealing `-a simulatedannealing` :heavy_check_mark:

Others heuristics and meta-heuristics:
- Random feasible solution found with a Local search `-a random` :heavy_check_mark:
- Repair linear relaxation solution `-a repairlinrelax_clp` :heavy_check_mark:
- Local search with LocalSolver `-a localsolver` :heavy_check_mark:
- Tree search algorithms based on Dantzig-Wolfe reformulation branching scheme
  - Greedy `-a cgh_greedy --lp-solver clp` :heavy_check_mark:
  - Limited discrepency search `-a cgh_limiteddiscrepencysearch --lp-solver clp` :x:

### Exact algorithms

- Branch-and-cut
  - with CBC `-a branchandcut_cbc` :heavy_check_mark:
  - with CPLEX `-a branchandcut_cplex` :heavy_check_mark:
  - with Gurobi `-a branchandcut_gurobi` :heavy_check_mark:

- Branch-and-price
  - `-a branchandprice --lp-solver clp --tree-search-algorithm dfs --branching-rule most-integer` :heavy_check_mark:
  - `-a branchandprice --lp-solver clp --tree-search-algorithm lds --branching-rule most-integer` :heavy_check_mark:
  - `-a branchandprice --lp-solver clp --tree-search-algorithm bfs --branching-rule most-fractional` :heavy_check_mark:

- Constraint programming
  - with Gecode `-a constraintprogramming_gecode` :heavy_check_mark:
  - with CPLEX `-a constraintprogramming_cplex` :heavy_check_mark:

## Notes

### Linear relaxation gap

The largest gap between the lower bound from the linear relaxation and the best known upper bound is 1.93%.

### Lagrangian relaxation implementation

The bound from the lagrangian relaxation of knapsack constraints is theoritically equal to the bound from the linear relaxation. The optimal bounds are found by the L-BGFS algorithm from DLib; however, the volume method stays rather far from it. 

The same happens for the bound obtained by solving the lagrangian relaxation of assignment constraints; the Volume method give poor results while the L-BFGS algorithm returns the best known ones found in "An exact method with variable fixing for solving the generalized assignment problem" (Posta et al., 2011).

### Finding a feasible solution

* [mthgregret -f wij/ti](https://librallu.gitlab.io/splitted-cell-viz/?u=https://raw.githubusercontent.com/fontanf/generalizedassignment/master/bench/mthgregret_f_wij_ti.json)
* [random](https://librallu.gitlab.io/splitted-cell-viz/?u=https://raw.githubusercontent.com/fontanf/generalizedassignment/master/bench/random.json)

## Usage (command line)

The only required dependency is Boost:
```shell
sudo apt-get install libboost-all-dev
```

Compile:
```shell
bazel build -- //...
```

However, most algorithms require additional libraries (see below).
Compile with additional libraries (or just uncomment the corresponding lines in `.bazelrc`):
```shell
bazel build \
    --define coinor=true \
    --define cplex=true \
    --define gurobi=true \
    --define gecode=true \
    --define dlib=true \
    --define localsolver=true \
    -- //...
```

Solve:
```shell
./bazel-bin/generalizedassignmentsolver/main -v \
    -a 'mthg -f -pij/wij' \
    -i "data/a05100" \
    -o "a05100_output.json" \
    -c "a05100_solution.txt"
```

Unit tests:
```shell
bazel test --compilation_mode=dbg -- //...
```

Checker:
```shell
./bazel-bin/generalizedassignmentsolver/checker data/a05100 output/best/a05100_solution.txt
```

Run benchmarks:
```shell
bazel build -- //...
python3 generalizedassignmentsolver/bench.py yagiura2004 "mthg -f wij/ti"          # no time limit
python3 generalizedassignmentsolver/bench.py yagiura2004 "branchandcut_gurobi" 60  # 1m time limit
```

## Optional dependencies

To enable a dependency, uncomment the corresponding line in the `.bazelrc` file and adapt its path in the `WORKSPACEi` file.

Here are some notes for their installations:

### COIN-OR (CLP, CBC, VOL, DIP)

Install (https://coin-or.github.io/coinbrew/):
```shell
git clone https://www.github.com/coin-or/coinbrew
cd coinbrew
./coinbrew fetch build Dip --no-prompt
./coinbrew fetch build Vol --no-prompt
```

### Gecode

Download latest version: download https://www.gecode.org/download.html

Compile (more info https://www.gecode.org/doc/2.2.0/reference/PageComp.html):
```shell
./configure
make
```

### Gurobi

After installing, execute the following commands:
```shell
cd ${GUROBI_HOME}/linux64/src/build/
make
cp libgurobi_c++.a ../../lib/
```

