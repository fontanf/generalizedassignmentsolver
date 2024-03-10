import run_tests
import os


commands = run_tests.generate_commands(
        "--algorithm column-generation-heuristic-greedy",
        os.path.join("test_results", "column_generation_heuristic_greedy"),
        "row['Dataset'] in ['cattrysse1994', 'chu1997']")


if __name__ == "__main__":
    for command in commands:
        run_tests.run(command)
