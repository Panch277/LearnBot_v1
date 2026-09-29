#!/usr/bin/env python3
"""Standalone ROS 2 node (no ament package, desktop-only -- same pattern as
rosbridge.launch.py) that lets the web dashboard start/stop long-running
`ros2 launch` processes (SLAM, Nav2) without a terminal.

Each entry in PROCESSES is exposed as a Trigger service
(`/dashboard/<name>/toggle`) plus a latched state topic
(`/dashboard/<name>/state`), so every connected browser tab -- not just the
one that clicked -- sees the current state, and it stays correct even if the
process dies on its own instead of being stopped from the dashboard.

Nav2 needs a map path, which a Trigger request can't carry (std_srvs/Trigger
has no request fields) -- rather than build a custom .srv (which would need
an ament package + colcon build just for this), the dashboard publishes the
chosen map to `/dashboard/nav2/map_path` (a plain std_msgs/String) whenever
the user picks one, well before they click "Start Nav2"; this node just
remembers the latest value and reads it at start time. `/dashboard/maps/list`
reuses the same trick in reverse: it's a Trigger whose `message` field
carries the comma-separated map names instead of a status string, so the
dashboard can populate its dropdown without a custom .srv either.
"""
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Grace period given to each signal before escalating -- `ros2 launch`
# forwards SIGINT to slam_toolbox/nav2 etc. and waits for them to shut down
# cleanly, which isn't instant.
SIGINT_GRACE_S = 8.0
SIGTERM_GRACE_S = 3.0

# Matches web/src/paths.ts's MAPS_DIR -- where "Save Map" (mapSave.ts, via
# slam_toolbox's own save_map service) writes, and where this node looks for
# maps to offer Nav2. Also matches the convention in SLAM_NAV2.md.
MAPS_DIR = '/home/panchpc/LearnBot_v1/src/learnbot/maps'

LATCHED_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class NoMapSelected(Exception):
    """Raised by a build_cmds callable that needs a map the dashboard hasn't sent yet."""


class ManagedProcess:
    def __init__(self, node: Node, name: str, build_cmds, conflicts_with: tuple[str, ...] = ()):
        self.node = node
        self.name = name
        self.build_cmds = build_cmds
        self.conflicts_with = conflicts_with
        self.state = 'stopped'
        # More than one command for processes like Nav2 (localization +
        # navigation launched together, stopped together).
        self.popens: list[subprocess.Popen] = []
        self.lock = threading.Lock()

        self.state_pub = node.create_publisher(String, f'/dashboard/{name}/state', LATCHED_QOS)
        node.create_service(Trigger, f'/dashboard/{name}/toggle', self._on_toggle)
        self._publish_state()

    def _publish_state(self) -> None:
        self.state_pub.publish(String(data=self.state))

    def _set_state(self, state: str) -> None:
        self.state = state
        self._publish_state()

    def _on_toggle(self, request, response):
        del request
        with self.lock:
            if self.state == 'stopped':
                blocking = [c for c in self.conflicts_with if self.node.managed_by_name[c].state != 'stopped']
                if blocking:
                    response.success = False
                    response.message = f'Stop {", ".join(blocking)} first'
                    return response
                try:
                    cmds = self.build_cmds()
                except NoMapSelected as exc:
                    response.success = False
                    response.message = str(exc)
                    return response
                self._start(cmds)
                response.success = True
                response.message = f'Starting {self.name}'
            elif self.state == 'running':
                self._stop()
                response.success = True
                response.message = f'Stopping {self.name}'
            else:
                response.success = False
                response.message = f'{self.name} is {self.state}, try again shortly'
        return response

    def _start(self, cmds: list[list[str]]) -> None:
        # New session/process group per command so `_stop()` can signal
        # `ros2 launch` and every node it spawns underneath it at once --
        # the same set of processes a terminal's Ctrl+C would reach.
        for cmd in cmds:
            self.node.get_logger().info(f'[{self.name}] starting: {" ".join(cmd)}')
            popen = subprocess.Popen(cmd, start_new_session=True)
            self.popens.append(popen)
            threading.Thread(target=self._monitor, args=(popen,), daemon=True).start()
        self._set_state('running')

    def _monitor(self, popen: subprocess.Popen) -> None:
        # The single place that reaps each child -- `_stop()` only signals
        # and polls, so there's never more than one thread calling wait()
        # on the same Popen.
        popen.wait()
        with self.lock:
            if popen in self.popens:
                self.popens.remove(popen)
            # Only "stopped" once every command belonging to this process
            # has exited -- Nav2's two launch files both need to be gone.
            if not self.popens and self.state != 'stopped':
                self._set_state('stopped')

    def _stop(self) -> None:
        self._set_state('stopping')
        for popen in list(self.popens):
            threading.Thread(target=self._escalate_shutdown, args=(popen,), daemon=True).start()

    def _escalate_shutdown(self, popen: subprocess.Popen) -> None:
        pgid = os.getpgid(popen.pid)
        for sig, grace in ((signal.SIGINT, SIGINT_GRACE_S), (signal.SIGTERM, SIGTERM_GRACE_S), (signal.SIGKILL, None)):
            self.node.get_logger().info(f'[{self.name}] sending {sig.name}')
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            deadline = None if grace is None else time.monotonic() + grace
            while popen.poll() is None and (deadline is None or time.monotonic() < deadline):
                time.sleep(0.2)
            if popen.poll() is not None:
                return


def _slam_cmds() -> list[list[str]]:
    return [['ros2', 'launch', 'learnbot', 'online_async_launch.py', 'use_sim_time:=false']]


class ProcessManagerNode(Node):
    def __init__(self):
        super().__init__('process_manager')
        self.selected_map_path: str | None = None

        self.create_subscription(String, '/dashboard/nav2/map_path', self._on_map_path, 10)
        self.create_service(Trigger, '/dashboard/maps/list', self._on_list_maps)

        # ManagedProcess.conflicts_with looks other entries up by name here,
        # so this dict has to exist (even empty) before any of them can
        # start -- filled in below, right after construction.
        self.managed_by_name: dict[str, ManagedProcess] = {}
        managed = [
            ManagedProcess(self, 'slam', _slam_cmds, conflicts_with=('nav2',)),
            ManagedProcess(self, 'nav2', self._nav2_cmds, conflicts_with=('slam',)),
        ]
        self.managed = managed
        self.managed_by_name.update({m.name: m for m in managed})

    def _on_map_path(self, msg: String) -> None:
        self.selected_map_path = msg.data
        self.get_logger().info(f'[nav2] map selected: {msg.data}')

    def _nav2_cmds(self) -> list[list[str]]:
        if not self.selected_map_path:
            raise NoMapSelected('Select a map first')
        return [
            ['ros2', 'launch', 'learnbot', 'localization_launch.py',
             'use_sim_time:=false', f'map:={self.selected_map_path}'],
            ['ros2', 'launch', 'learnbot', 'navigation_launch.py',
             'use_sim_time:=false', 'map_subscribe_transient_local:=true'],
        ]

    def _on_list_maps(self, request, response):
        del request
        try:
            names = sorted(f[:-len('.yaml')] for f in os.listdir(MAPS_DIR) if f.endswith('.yaml'))
        except OSError as exc:
            response.success = False
            response.message = f'Could not list {MAPS_DIR}: {exc}'
            return response
        response.success = True
        response.message = ','.join(names)
        return response


def main():
    rclpy.init()
    node = ProcessManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # Normal path: stopping the bridge (Ctrl+C on rosbridge.launch.py,
        # which includes this node) delivers SIGINT here too, and rclpy
        # turns that into one of these rather than a plain return -- already
        # shutting down the context itself, so the `finally` below must not
        # call rclpy.shutdown() again.
        pass
    finally:
        # Best-effort cleanup so stopping the bridge doesn't orphan a
        # still-running SLAM/Nav2 process.
        for managed in node.managed:
            for popen in managed.popens:
                try:
                    os.killpg(os.getpgid(popen.pid), signal.SIGINT)
                except ProcessLookupError:
                    pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
