import subprocess

ENVIRONMENTS = [
    "polygon",
    "polygon2",
    "polygon2withholes",
    "square",
    "triangle",
    "custom",
    "square2withhole",
    "corridor",
    "legs",
    "star",
    "lettere",
    "mono",
    "room",
    "cross",
]
NUMBER_OF_ROBOTS = [1, 2, 4]
NUMBER_OF_TESTS = 2

for env in ENVIRONMENTS:
    for bots in NUMBER_OF_ROBOTS:
        for i in range(NUMBER_OF_TESTS):
            p = subprocess.Popen(
                ["../build/ExplorationBot", env, str(bots), "test"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            p.wait()
