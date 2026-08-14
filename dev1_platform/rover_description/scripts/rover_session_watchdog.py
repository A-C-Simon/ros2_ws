#!/usr/bin/env python3
"""Session watchdog for rover_description simulation.launch.py.

Launched as a sibling process. It catches SIGINT/SIGTERM from launch's
shutdown storm and runs the cleanup script, which kills Gazebo, RViz2, and
any terminal-hosted scripts the launch service did not reach. It also polls
its parent so a SIGKILL'd launch is still cleaned up.
"""
import argparse
import os
import signal
import subprocess
import sys
import time


def parent_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--parent', type=int, required=True,
                    help='PID of the ros2 launch process to watch')
    ap.add_argument('--cleanup-script', required=True,
                    help='Absolute path to kill_rover_session.sh')
    args = ap.parse_args()

    def run_cleanup():
        subprocess.run(['bash', args.cleanup_script], check=False)

    def on_shutdown(signum, frame):
        # Launch is shutting down (Ctrl+C in any terminal). Tear down Gazebo,
        # RViz2, and any terminal-hosted scripts the launch service missed.
        run_cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_shutdown)
    signal.signal(signal.SIGTERM, on_shutdown)

    # If launch is SIGKILL'd, we notice it disappeared and clean up then.
    while parent_alive(args.parent):
        time.sleep(0.2)

    run_cleanup()


if __name__ == '__main__':
    main()
