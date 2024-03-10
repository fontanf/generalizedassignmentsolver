import run_tests
import os


commands = run_tests.generate_commands(
        "--algorithm milp-cbc --maximum-number-of-nodes 10000",
        os.path.join("test_results", "milp_cbc"),
        "row['Dataset'] in ['cattrysse1994', 'chu1997']")


if __name__ == "__main__":
    for command in commands:
        run_tests.run(command)
