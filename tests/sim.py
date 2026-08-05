import subprocess

NUMBER_OF_TESTS = 2

for i in range(NUMBER_OF_TESTS):
    p = subprocess.Popen(
        ["../build/ExplorationBot", "square", "2", "test"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    p.wait()
