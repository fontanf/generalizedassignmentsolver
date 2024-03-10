import run_tests
import os


commands = run_tests.generate_commands(
        "--algorithm local-search --maximum-number-of-nodes 1024",
        os.path.join("test_results", "local_search"))


if __name__ == "__main__":
    for command in commands:
        run_tests.run(command)
