import subprocess

p = subprocess.Popen(
    ["./build/ExplorationBot", "room", "4", "test"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    bufsize=1,
)

p.wait()
