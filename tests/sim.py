import subprocess
from datetime import datetime

ENVIRONMENTS = [
    [
        "polygon2",
        [[3, 3], [12, 12], [17, 13], [18, 19]],
    ],
    [
        "polygon4",
        [[3, 3], [12, 12], [24, 24]],
    ],
    [
        "polygon2withholes",
        [[3, 3], [12, 12], [17, 13], [13, 15]],
    ],
    [
        "square2withhole",
        [[3, 3], [5.2, 10], [16, 16]],
    ],
    [
        "bigroom",
        [[3, 3], [10, 10], [20, 20], [30, 30]],
    ],
    [
        "maze",
        [[2, 2], [3, 3], [13, 11], [19, 1]],
    ],
    [
        "corridor",
        [[3, 3], [5, 5], [15, 4.6], [20, 6]],
    ],
    [
        "legs",
        [[3, 3], [13, 11]],
    ],
    [
        "mono",
        [[3, 3], [3, 14.5], [9, 14]],
    ],
    [
        "room",
        [[3, 3], [17, 3], [4, 15]],
    ],
    [
        "cross",
        [[3, 3], [10, 10], [16, 16]],
    ],
]
ROBOTS = [1, 2, 4, 8, 16]
RADIUS = [1, 2, 4]
STRATEGIES = ["largest", "smallest", "closest"]
REPEATS = 5
TIMEOUT = 1560

WIDTH = 400
HEIGHT = 300

total = (
    REPEATS
    * sum(len(positions) for _, positions in ENVIRONMENTS)
    * len(ROBOTS)
    * len(RADIUS)
    * len(STRATEGIES)
)

completed = 0

for i in range(REPEATS):
    for env, positions in ENVIRONMENTS:
        for bots in ROBOTS:
            for pos in positions:
                for rad in RADIUS:
                    for strat in STRATEGIES:
                        completed += 1
                        percent = completed / total * 100

                        print(
                            f"({datetime.now().strftime('%H:%M:%S')}) [{completed}/{total}] {percent:.2f}%"
                        )

                        p = subprocess.Popen(
                            [
                                "../build/ExplorationBot",
                                "--width",
                                str(WIDTH),
                                "--height",
                                str(HEIGHT),
                                "--env",
                                env,
                                "-x",
                                str(pos[0]),
                                "-y",
                                str(pos[1]),
                                "-n",
                                str(bots),
                                "-r",
                                str(rad),
                                "-s",
                                str(strat),
                                "--test",
                                "--no-graph",
                                "--output-all",
                            ],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True,
                            bufsize=1,
                        )

                        try:
                            p.communicate(timeout=TIMEOUT)
                        except subprocess.TimeoutExpired:
                            print(
                                f"TIMED OUT: {env}, {bots} bots @ {pos}, radius {rad}, strategy {strat}, repeat {i + 1}",
                            )
                            p.kill()
                            p.communicate()
