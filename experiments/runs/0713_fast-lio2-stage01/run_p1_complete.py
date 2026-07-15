#!/usr/bin/env python3
"""Complete P1 straight-line test: start controller, RL chain, run P1 test."""

import os
import sys
import time
import subprocess
import signal

CTRL_BIN = "/home/zzf/search_ws/SimEnv/devel/lib/unitree_guide/junior_ctrl"
LOG_DIR = "/home/zzf/search_ws/SimEnv/experiments/runs/0713_fast-lio2-stage01"

def main():
    os.chdir("/home/zzf/search_ws/SimEnv")

    # Start controller with pty for keyboard input
    import pty
    master, slave = pty.openpty()

    env = os.environ.copy()
    env["PATH"] = "/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

    proc = subprocess.Popen(
        [CTRL_BIN],
        stdin=slave,
        stdout=open(os.path.join(LOG_DIR, "p1_ctrl_stdout.log"), "w"),
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=os.setsid
    )
    os.close(slave)

    print(f"Controller PID: {proc.pid}")

    # Wait for init
    time.sleep(8)

    # State chain: Passive -> FixedStand (2)
    print("Sending: FixedStand (2)")
    os.write(master, b"2\n")
    time.sleep(5)  # Wait for stand to complete

    # FixedStand -> RL (6)
    print("Sending: RL mode (6)")
    os.write(master, b"6\n")
    time.sleep(3)  # Wait for RL to engage

    print("RL mode engaged. Run P1 test now:")
    print(f"  cd /home/zzf/search_ws/SimEnv")
    print(f"  source devel/setup.bash")
    print(f"  FAST_LIO_P1_TARGET_M=1.0 FAST_LIO_P1_SPEED_MPS=0.15 python3 experiments/runs/0713_fast-lio2-stage01/run_p1_straight.py")

    # Keep controller running
    os.close(master)
    proc.wait()

if __name__ == "__main__":
    main()
