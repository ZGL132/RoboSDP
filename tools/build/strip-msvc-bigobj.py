import subprocess
import sys


def main() -> int:
    if len(sys.argv) < 2:
        return 1

    compiler = sys.argv[1]
    arguments = [argument for argument in sys.argv[2:] if argument != "/bigobj"]
    completed = subprocess.run([compiler, *arguments], check=False)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
