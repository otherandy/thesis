import subprocess

ENVIRONMENTS = [
    [
        "polygon",
        [[3, 3], [4, 4], [6, 6], [9, 10]],
    ],
    [
        "polygon2",
        [[3, 3], [12, 12], [17, 13], [18, 19]],
    ],
    [
        "polygon2withholes",
        [[3, 3], [12, 12], [17, 13], [13, 15]],
    ],
    [
        "square",
        [[3, 3], [5, 5], [5, 9.6]],
    ],
    [
        "custom",
        [[3, 3], [5, 5], [9, 9], [12, 9.2]],
    ],
    [
        "square2withhole",
        [[3, 3], [5.2, 10], [16, 16]],
    ],
    [
        "square4withholes",
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
        "star",
        [[3, 3], [5, 4]],
    ],
    [
        "lettere",
        [[3, 3], [4, 4], [6, 11], [8, 17]],
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
RADIUS = [0.5, 1, 1.5, 2]
STRATEGIES = ["largest", "smallest", "closest"]
REPEATS = 10
TIMEOUT = 240
WIDTH = 400
HEIGHT = 300

for env, positions in ENVIRONMENTS:
    for pos in positions:
        for bots in ROBOTS:
            for rad in RADIUS:
                for strat in STRATEGIES:
                    for i in range(REPEATS):
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
                                f"Timed out: {env}, {bots} bots @ {pos} with {rad} radius and {strat} strategy, test {i + 1}"
                            )
                            p.kill()
                            p.communicate()
