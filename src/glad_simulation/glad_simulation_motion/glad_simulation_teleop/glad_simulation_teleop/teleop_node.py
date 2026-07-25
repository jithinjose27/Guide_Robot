#!/usr/bin/env python3
import curses
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


LOG_FILE = os.path.expanduser("~/.glad_teleop.log")


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")

        self.declare_parameter("linear_speed", 0.2)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("linear_step", 0.05)
        self.declare_parameter("angular_step", 0.1)
        self.declare_parameter("key_timeout", 0.5)

        self.max_linear = self.get_parameter("linear_speed").value
        self.max_angular = self.get_parameter("angular_speed").value
        self.linear_step = self.get_parameter("linear_step").value
        self.angular_step = self.get_parameter("angular_step").value
        self.key_timeout = self.get_parameter("key_timeout").value

        self.linear_speed = self.max_linear
        self.angular_speed = self.max_angular

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel/teleop", 10)
        self.pub_stop = self.create_publisher(Bool, "/cmd_vel/stop", 10)

        self.estop = False
        self.running = True
        self.active_key = None
        self.last_time = 0.0
        self.cmd = Twist()
        self.status_msg = ""

    def _twist(self, lin, ang):
        cmd = Twist()
        cmd.linear.x = lin * self.linear_speed
        cmd.angular.z = ang * self.angular_speed
        return cmd

    def on_key(self, ch):
        now = time_time()
        if ch == ord(" "):
            self.estop = not self.estop
            self.pub_stop.publish(Bool(data=self.estop))
            if self.estop:
                self.cmd = Twist()
                self.pub_cmd.publish(Twist())
                self.status_msg = "ESTOP ENGAGED"
            else:
                self.status_msg = "ESTOP released"
            self.active_key = None
            return
        if self.estop:
            return

        if ch == ord("q"):
            self.cmd = self._twist(1.0, 1.0)
        elif ch == ord("w"):
            self.cmd = self._twist(1.0, 0.0)
        elif ch == ord("e"):
            self.cmd = self._twist(1.0, -1.0)
        elif ch == ord("a"):
            self.cmd = self._twist(0.0, 1.0)
        elif ch == ord("d"):
            self.cmd = self._twist(0.0, -1.0)
        elif ch == ord("x"):
            self.cmd = self._twist(-1.0, 0.0)
        elif ch in (ord("s"), ord("S")):
            self.cmd = Twist()
            self.status_msg = "STOP"
        elif ch in (ord("+"), ord("=")):
            self.linear_speed = min(
                self.max_linear * 3, self.linear_speed + self.linear_step
            )
        elif ch in (ord("-"), ord("_")):
            self.linear_speed = max(0.01, self.linear_speed - self.linear_step)
        elif ch == ord("]"):
            self.angular_speed = min(
                self.max_angular * 3, self.angular_speed + self.angular_step
            )
        elif ch == ord("["):
            self.angular_speed = max(0.01, self.angular_speed - self.angular_step)
        else:
            return

        self.active_key = ch
        self.last_time = now
        self.pub_cmd.publish(self.cmd)

    def update(self):
        if self.estop:
            return
        if (
            self.active_key is not None
            and time_time() - self.last_time > self.key_timeout
        ):
            self.active_key = None
            self.cmd = Twist()
            self.pub_cmd.publish(Twist())

    def state_str(self):
        if self.estop:
            return "ESTOP"
        m = {
            ord("q"): "FWD+LEFT",
            ord("w"): "FORWARD",
            ord("e"): "FWD+RIGHT",
            ord("a"): "LEFT",
            ord("d"): "RIGHT",
            ord("s"): "STOP",
            ord("S"): "STOP",
            ord("x"): "REVERSE",
        }
        return m.get(self.active_key, "STOPPED") if self.active_key else "STOPPED"


RED = 1
GREEN = 2
YELLOW = 3
CYAN = 4
WHITE = 5
MAGENTA = 6
REV = 7


def setup_colors():
    for pair, fg in [
        (RED, curses.COLOR_RED),
        (GREEN, curses.COLOR_GREEN),
        (YELLOW, curses.COLOR_YELLOW),
        (CYAN, curses.COLOR_CYAN),
        (WHITE, curses.COLOR_WHITE),
        (MAGENTA, curses.COLOR_MAGENTA),
    ]:
        try:
            curses.init_pair(pair, fg, -1)
        except curses.error:
            curses.init_pair(pair, fg, 0)
    curses.init_pair(REV, curses.COLOR_BLACK, curses.COLOR_CYAN)


def color(pair):
    return curses.color_pair(pair) | curses.A_BOLD


BOX_W = 42


def draw_ui(stdscr, node):
    stdscr.erase()
    h, w = stdscr.getmaxyx()
    if w < BOX_W + 2 or h < 22:
        stdscr.addstr(0, 0, "Terminal too small \u2014 resize to at least 44x22")
        stdscr.refresh()
        return

    cx = max(0, (w - BOX_W) // 2)
    cy = max(0, (h - 20) // 2)

    def add_line(y, left_text="", right_text=""):
        visible = left_text + right_text
        pad = BOX_W - 2 - len(visible)
        if pad < 0:
            visible = visible[: BOX_W - 2]
            pad = 0
        line_str = "\u2502 " + left_text + right_text + " " * pad + " \u2502"
        stdscr.addstr(cy + y, cx, line_str[:BOX_W])

    state = node.state_str()
    lx = node.cmd.linear.x
    az = node.cmd.angular.z

    stdscr.addstr(cy, cx, "\u250c" + "\u2500" * (BOX_W - 2) + "\u2510", color(WHITE))

    title = "\u2502   GLAD Robot Teleop   " + time_str() + " \u2502"
    stdscr.addstr(cy + 1, cx, title[:BOX_W], color(REV))

    stdscr.addstr(
        cy + 2, cx, "\u251c" + "\u2500" * (BOX_W - 2) + "\u2524", color(WHITE)
    )

    add_line(3, "Linear:  ", f"{node.linear_speed:5.2f} m/s     [ +/- ]")
    add_line(4, "Angular: ", f"{node.angular_speed:5.2f} rad/s   [ [ ] ]")
    add_line(5, "Estop:   ", "ENGAGED" if node.estop else "OFF      ")

    add_line(6, "Status:  ", f"{state:26s}")

    add_line(7, "cmd:     ", f"linear={lx:+.2f}  angular={az:+.2f}")

    stdscr.addstr(
        cy + 8, cx, "\u251c" + "\u2500" * (BOX_W - 2) + "\u2524", color(WHITE)
    )

    add_line(9, " Controls:")
    act = node.active_key
    q_hl = GREEN if act == ord("q") else WHITE
    w_hl = GREEN if act == ord("w") else WHITE
    e_hl = GREEN if act == ord("e") else WHITE
    a_hl = GREEN if act == ord("a") else WHITE
    s_hl = GREEN if act in (ord("s"), ord("S")) else WHITE
    d_hl = GREEN if act == ord("d") else WHITE
    x_hl = GREEN if act == ord("x") else WHITE

    stdscr.addstr(cy + 10, cx, "\u2502       ", color(WHITE))
    stdscr.addstr("Q", color(q_hl))
    stdscr.addstr("   ", color(WHITE))
    stdscr.addstr("W", color(w_hl))
    stdscr.addstr("   ", color(WHITE))
    stdscr.addstr("E", color(e_hl))
    stdscr.addstr("           \u2502", color(WHITE))

    stdscr.addstr(cy + 11, cx, "\u2502       ", color(WHITE))
    stdscr.addstr("A", color(a_hl))
    stdscr.addstr("   ", color(WHITE))
    stdscr.addstr("S", color(s_hl))
    stdscr.addstr("   ", color(WHITE))
    stdscr.addstr("D", color(d_hl))
    stdscr.addstr("           \u2502", color(WHITE))

    stdscr.addstr(cy + 12, cx, "\u2502             ", color(WHITE))
    stdscr.addstr("X", color(x_hl))
    stdscr.addstr("              \u2502", color(WHITE))

    add_line(13, "")
    add_line(14, "  +/-  linear    [/]  angular")
    add_line(15, "  SPACE  emergency stop (toggle)")

    stdscr.addstr(
        cy + 16, cx, "\u2514" + "\u2500" * (BOX_W - 2) + "\u2518", color(WHITE)
    )

    if node.status_msg:
        stdscr.addstr(cy + 18, cx, node.status_msg, color(YELLOW))

    stdscr.refresh()


def time_str():
    import time

    return time.strftime("%H:%M:%S")


time_time = __import__("time").time


def main():
    os.dup2(os.open(LOG_FILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644), 2)

    rclpy.init(args=sys.argv)
    node = TeleopKeyboard()

    stdscr = curses.initscr()
    curses.start_color()
    setup_colors()
    curses.cbreak()
    stdscr.keypad(True)
    curses.curs_set(0)
    stdscr.nodelay(True)

    try:
        while node.running and rclpy.ok():
            ch = stdscr.getch()
            if ch != -1:
                node.on_key(ch)

            rclpy.spin_once(node, timeout_sec=0)

            node.update()

            draw_ui(stdscr, node)

            __import__("time").sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        node.pub_stop.publish(Bool(data=False))
        node.destroy_node()
        curses.curs_set(1)
        curses.endwin()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"GLAD Teleop stopped. Log: {LOG_FILE}")


if __name__ == "__main__":
    main()
