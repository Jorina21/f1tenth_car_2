#!/usr/bin/env python3

import argparse
import curses
import subprocess
import time


class SpeedTuner:
    def __init__(
        self,
        node_name: str,
        param_name: str,
        min_scale: float,
        max_scale: float,
        step: float,
        start_scale: float,
    ):
        self.node_name = node_name
        self.param_name = param_name

        self.min_scale = min_scale
        self.max_scale = max_scale
        self.step = step
        self.start_scale = start_scale

        self.speed_scale = self.clamp(start_scale)

        self.last_ok = False
        self.last_message = "Not started yet"

    def clamp(self, value: float) -> float:
        return max(self.min_scale, min(self.max_scale, value))

    def set_speed_scale(self, value: float):
        self.speed_scale = round(self.clamp(value), 2)

        cmd = [
            "ros2",
            "param",
            "set",
            self.node_name,
            self.param_name,
            str(self.speed_scale),
        ]

        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
        )

        if result.returncode == 0:
            self.last_ok = True
            self.last_message = result.stdout.strip()
        else:
            self.last_ok = False
            err = result.stderr.strip()
            out = result.stdout.strip()
            self.last_message = err if err else out

    def increase(self):
        self.set_speed_scale(self.speed_scale + self.step)

    def decrease(self):
        self.set_speed_scale(self.speed_scale - self.step)

    def stop(self):
        self.set_speed_scale(0.0)

    def reset(self):
        self.set_speed_scale(self.start_scale)


def draw_screen(stdscr, tuner: SpeedTuner):
    stdscr.clear()

    stdscr.addstr(0, 0, "F1TENTH Live Speed Tuner")
    stdscr.addstr(1, 0, "========================")

    stdscr.addstr(3, 0, f"Target node:  {tuner.node_name}")
    stdscr.addstr(4, 0, f"Parameter:    {tuner.param_name}")

    stdscr.addstr(6, 0, f"speed_scale:  {tuner.speed_scale:.2f}")
    stdscr.addstr(7, 0, f"min_scale:    {tuner.min_scale:.2f}")
    stdscr.addstr(8, 0, f"max_scale:    {tuner.max_scale:.2f}")
    stdscr.addstr(9, 0, f"step:         {tuner.step:.2f}")

    stdscr.addstr(11, 0, "Controls:")
    stdscr.addstr(12, 0, "  UP arrow       increase speed")
    stdscr.addstr(13, 0, "  DOWN arrow     decrease speed")
    stdscr.addstr(14, 0, "  SPACE          set speed_scale to 0.0")
    stdscr.addstr(15, 0, "  r              reset speed_scale")
    stdscr.addstr(16, 0, "  q              quit")

    stdscr.addstr(18, 0, "Status:")
    if tuner.last_ok:
        stdscr.addstr(19, 0, f"  OK: {tuner.last_message}")
    else:
        stdscr.addstr(19, 0, f"  ERROR: {tuner.last_message}")

    stdscr.addstr(21, 0, "Safety note:")
    stdscr.addstr(22, 0, "  SPACE is a soft stop. Keep your TTC/brake node active.")

    stdscr.refresh()


def curses_main(stdscr, tuner: SpeedTuner):
    curses.curs_set(0)
    stdscr.keypad(True)
    stdscr.nodelay(False)

    tuner.reset()

    while True:
        draw_screen(stdscr, tuner)

        key = stdscr.getch()

        if key == curses.KEY_UP:
            tuner.increase()

        elif key == curses.KEY_DOWN:
            tuner.decrease()

        elif key == ord(" "):
            tuner.stop()

        elif key == ord("r") or key == ord("R"):
            tuner.reset()

        elif key == ord("q") or key == ord("Q"):
            break

        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(
        description="Keyboard speed_scale tuner for F1TENTH ROS 2 nodes"
    )

    parser.add_argument(
        "--node-name",
        default="/qualifier_node",
        help="Target ROS 2 node name",
    )

    parser.add_argument(
        "--param-name",
        default="speed_scale",
        help="Target ROS 2 parameter name",
    )

    parser.add_argument(
        "--min-scale",
        type=float,
        default=0.0,
        help="Minimum speed scale",
    )

    parser.add_argument(
        "--max-scale",
        type=float,
        default=5.2, #1.2
        help="Maximum speed scale",
    )
    #comtrols how much it goes up 
    parser.add_argument(
        "--step", 
        type=float,
        default=0.5, #goes up 0.1 
        help="Speed scale increment/decrement",
    )

    parser.add_argument(
        "--start-scale",
        type=float,
        default=0.6,
        help="Starting/reset speed scale",
    )

    args = parser.parse_args()

    tuner = SpeedTuner(
        node_name=args.node_name,
        param_name=args.param_name,
        min_scale=args.min_scale,
        max_scale=args.max_scale,
        step=args.step,
        start_scale=args.start_scale,
    )

    curses.wrapper(curses_main, tuner)


if __name__ == "__main__":
    main()