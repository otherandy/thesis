import subprocess

ENVIRONMENTS = [
    "polygon",
    "polygon2",
    "polygon2withholes",
    "square",
    "triangle",
    "custom",
    "square2withhole",
    "square4",
    "square4withholes",
    "maze",
    "corridor",
    "legs",
    "star",
    "lettere",
    "mono",
    "room",
    "cross",
]
NUMBER_OF_ROBOTS = [1, 2, 4, 8]
NUMBER_OF_TESTS = 1
TIMEOUT = 120
WIDTH = 400
HEIGHT = 300

for env in ENVIRONMENTS:
    for bots in NUMBER_OF_ROBOTS:
        for i in range(NUMBER_OF_TESTS):
            p = subprocess.Popen(
                [
                    "../build/ExplorationBot",
                    "--width",
                    str(WIDTH),
                    "--height",
                    str(HEIGHT),
                    "--env",
                    env,
                    "--numbots",
                    str(bots),
                    "--test",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            try:
                p.communicate(timeout=TIMEOUT)
            except subprocess.TimeoutExpired:
                print(f"Timed out: {env}, {bots} bots, test {i + 1}")
                p.kill()
                p.communicate()
