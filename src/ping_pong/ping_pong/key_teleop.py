#! usr/bin/env python3

import curses

# For 'q' keystroke exit
import os
import signal
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node


class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            # TODO(artivis) Why are those floats ??
            self._screen.addstr(int(y), int(x), text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()


class SimpleKeyTeleop(Node):

    def __init__(self, interface):
        super().__init__('key_teleop')

        self._interface = interface

        self._pub_cmd_1 = self.create_publisher(Twist, 'turtlesim1/player_1/cmd_vel', 10)
        self._pub_cmd_2 = self.create_publisher(Twist, 'turtlesim1/player_2/cmd_vel', 10)

        self._hz = self.declare_parameter('hz', 2).value

        self._forward_rate = self.declare_parameter('forward_rate', 0.8).value

        self._last_pressed = {}
        self._linear1 = 0.0
        self._linear2 = 0.0


        self.movement_bindings = {
            curses.KEY_UP:    (1.0,0.0),
            curses.KEY_DOWN:  (-1.0,0.0),
            ord('w'):  (0.0,1.0),
            ord('s'): (0.0,-1.0),
        }


    def run(self):
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._publish()

            time.sleep(1.0/self._hz)


    def _make_twist(self, linear):
        twist = Twist()
        twist.linear.x = linear
        return twist


    def _set_velocity(self):
        now = self.get_clock().now()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < Duration(seconds=0.2):
                keys.append(a)
        linear1 = self._linear1
        linear2 = self._linear2
        for k in keys:
            l1, l2 = self.movement_bindings[k]
            linear1 += l1 * self._forward_rate
            linear2 += l2 * self._forward_rate

        self._linear1 = linear1
        self._linear2 = linear2


    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False

            os.kill(os.getpid(), signal.SIGINT)
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = self.get_clock().now()


    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, 'player1: %f, player2: %f' % (self._linear1, self._linear2))
        self._interface.write_line(5, 'Use arrow keys to move player 1 and w,s for player 2, q to exit.')
        self._interface.refresh()

        twist1 = self._make_twist(self._linear1)
        twist2 = self._make_twist(self._linear2)

        self._pub_cmd_1.publish(twist1)
        self._pub_cmd_2.publish(twist2)


def execute(stdscr):
    rclpy.init()

    app = SimpleKeyTeleop(TextWindow(stdscr))
    app.run()

    app.destroy_node()
    rclpy.shutdown()


def main():
    try:
        curses.wrapper(execute)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()