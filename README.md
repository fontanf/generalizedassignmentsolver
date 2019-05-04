WORK IN PROGRESS

# Generalized Assignment Problem

Algorithm implementations for the Generalized Assignment Problem.

Compile:
```
bazel build --cxxopt='-std=c++14' --compilation_mode=opt -- //...
```

Solve:
```
./bazel-bin/lib/main -v -a milp -i data/a05100 -o out.ini -c sol.txt
```

Unit tests:
```
bazel test --cxxopt='-std=c++14' --compilation_mode=opt -- //...
```

Generate an instance:
```
bazel build --cxxopt='-std=c++14' --compilation_mode=opt -- //lib:generator_main
./bazel-bin/lib/generator_main -n 200 -m 20 -t f -r 100 -o ./ins.txt
```

Examples:
- `-n 100 -m 10 -t f -r 100`
- `-n 100 -m 10 -t g -r 100`

Instances can be visualized with gnuplot:
```
./bazel-bin/lib/generator_main -n 200 -m 20 -t f -r 100 -o ./ins.txt -p ./ins.plot
gnuplot
gnuplot> set yrange[0:]; set xrange[0:]; plot 'ins.plot' u 1:2
```

## Lower bounds

- Lagrangian relaxation solved with volume method :x:
- Lagrangian relaxation solved with bundle method :x:
- Column generation :x:

## Upper bounds

Classical meta-heuristics based on shift-swap neighborhood:
- Hill climbing, first improvment `-a lsfirst_shiftswap` :heavy_check_mark:
- Hill climbing, best improvment `-a lsbest_shiftswap` :heavy_check_mark:
- Tabu search `-a ts_shiftswap` :heavy_check_mark:
- Simulated annealing `-a sa_shiftswap` :heavy_check_mark:
- Path relinking `-a pr_shiftswap` :heavy_check_mark:

Others:
- Random initial solution `-a random` :heavy_check_mark:
- Variable Depth Neighborhood Search `-a vdns_simple` :heavy_check_mark:

## Exact algorithms

- MILP solved with CBC `-a milp` :heavy_check_mark:
- Branch-and-price :x:

## Results

`vdns_simple` does not compete with state of the art meta-heuristics like the ones presented in "A path relinking approach with ejection chains for the generalized assignment problem" (Yagiura, 2006) or "Variable-fixing then subgradient optimization guided very large scale neighborhood search for the generalized assignment problem" (Haddadi, 2018) in terms of solution quality on a long run. However:
- on a short time run (2 minutes, Processor Intel® Core™ i5-8500 CPU @ 3.00GHz × 6), it provides good solutions (less than 1% gap from optimal for most instances of the literature, between 1% and 2% for some hard instances)
- it is very simple and the implementation is very short
- it is available and free (MIT License)

