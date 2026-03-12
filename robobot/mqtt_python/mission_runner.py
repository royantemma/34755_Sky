"""
mission_runner.py
-----------------
Generic mission executor.
It reads a TASKS list and runs each task in sequence.
Add new task types by adding a run_<type> method and an elif in run().
"""

import time as t
import numpy as np
from datetime import datetime

from spose import pose
from sedge import edge
from uservice import service
from sir import ir

class MissionRunner:

    def __init__(self, tasks, total_time, goal_time_buffer):
        self.tasks = tasks
        self.total_time = total_time              # seconds allowed for full mission
        self.goal_time_buffer = goal_time_buffer  # go to goal if less than this left
        self.mission_start = None
        self.task_index = 0
        self.task_state = 0
        self.task_start = None

    # time helpers _____________________________

    def time_elapsed(self):
        return (datetime.now() - self.mission_start).total_seconds()

    def time_left(self):
        return self.total_time - self.time_elapsed()

    def task_time(self):
        return (datetime.now() - self.task_start).total_seconds()

    def should_go_to_goal(self):
        return self.time_left() < self.goal_time_buffer

    # task transitions _________________________

    def start_task(self, index):
        self.task_index = index
        self.task_state = 0
        self.task_start = datetime.now()
        pose.tripBreset() # use trip counter/timer B
        task = self.tasks[index]
        print(f"% [MissionRunner] Starting task {index}: {task['type']}")

    def next_task(self):
        self.start_task(self.task_index + 1)

    # main loop _________________________________

    def run(self):
        self.mission_start = datetime.now()
        self.start_task(0)
        service.send("robobot/cmd/T0", "leds 16 0 0 30")   # blue = running

        while not service.stop:
            current_task = self.tasks[self.task_index]
            task_type    = current_task["type"]

            # dispatch to executor
            if task_type == "line_follow":
                self.run_line_follow(current_task)

            elif task_type == "line_follow_brake":
                self.run_line_follow_brake(current_task)

            elif task_type == "drive_straight":
                self.run_drive_straight(current_task)

            elif task_type == "turn":
                self.run_turn(current_task)
            
            elif task_type == "drive_circle":
                self.run_drive_circle(current_task)

            elif task_type == "turn_to_heading":
                self.run_turn_to_heading(current_task)

            else:
                print(f"% [MissionRunner] Unknown task type '{task_type}' — skipping")
                self.next_task()

            t.sleep(0.02)

        # clean up
        edge.lineControl(0, True)
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        service.send("robobot/cmd/T0", "leds 16 0 0 0")
        print("% [MissionRunner] Finished")

    # run methods _______________________________

    def run_line_follow(self, task):
        """
        Follow a line on the chosen side until distance is reached, line is lost,
        or timeout expires.
        """
        follow_left = (task["side"] == "left")
        speed = task.get("speed", 0.2)
        timeout = task.get("timeout", 30)

        if self.task_state == 0:
            print("% [line_follow] Starting — moving forward to find line")
            service.send("robobot/cmd/T0", "leds 16 0 100 0") # green
            service.send("robobot/cmd/ti",f"rc {speed:.3f} 0.0") # (forward m/s, turn-rate rad/sec)
            self.task_state = 1

        elif self.task_state == 1: # approaching line
            if edge.lineValidCnt > 4:
                edge.lineControl(speed, follow_left)
                pose.tripBreset()
                print("% [line_follow] Line found — following")
                self.task_state = 2
            elif pose.tripB > 4.0 or self.task_time() > timeout:
                service.send("robobot/cmd/ti/","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
                self.task_state = 3

        elif self.task_state == 2: # following line
            if self.task_time() > timeout:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("% [line_follow] Timeout")
                self.task_state = 3
            elif edge.lineValidCnt < 2:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% [line_follow] Line lost at {pose.tripB:.3f}m")
                self.task_state = 3
            # elif : we arrived at destination

        elif self.task_state == 3: # stopping
            if abs(pose.velocity()) < 0.001:
                print("% [line_follow] Stopped — next task")
                self.next_task()

    def run_line_follow_brake(self, task):
        """
        Follow a line on the chosen side until line is lost,
        or timeout expires.
        """
        follow_left = (task["side"] == "left")
        speed = task.get("speed", 0.2)
        timeout = task.get("timeout", 30)

        if self.task_state == 0:
            print("% [line_follow] Starting — moving forward to find line")
            service.send("robobot/cmd/T0", "leds 16 0 100 0") # green
            service.send("robobot/cmd/ti",f"rc {speed:.3f} 0.0") # (forward m/s, turn-rate rad/sec)
            self.task_state = 1

        elif self.task_state == 1: # approaching line
            if edge.lineValidCnt > 4:
                edge.lineControl(speed, follow_left)
                pose.tripBreset()
                print("% [line_follow] Line found — following")
                self.task_state = 2
            elif pose.tripB > 4.0 or self.task_time() > timeout:
                service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
                self.task_state = 3

        elif self.task_state == 2:  # following line
            normal_speed = task.get("speed",       0.15)
            brake_speed  = task.get("brake_speed", -0.05)
            max_speed    = task.get("max_speed",    0.25)

            # check actual velocity and adjust
            if pose.velocity() > max_speed:
                # going too fast — switch to braking speed
                edge.lineControl(brake_speed, follow_left)
            else:
                # normal speed
                edge.lineControl(normal_speed, follow_left)
                
            if self.task_time() > timeout:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("% [line_follow] Timeout")
                self.task_state = 3
            elif edge.lineValidCnt < 2:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% [line_follow] Line lost at {pose.tripB:.3f}m")
                self.task_state = 3

        elif self.task_state == 3: # stopping
            if abs(pose.velocity()) < 0.001:
                print("% [line_follow] Stopped — next task")
                self.next_task()


    def run_turn(self, task):
        """
        Turn in place using odometry.

        Task keys:
            angle_deg   degrees; positive = left/CCW, negative = right/CW
            speed       turn rate rad/s (default 0.3)
            timeout     seconds (default 15)
        """
        angle_rad = np.radians(task["angle_deg"])
        turn_rate = task.get("speed", 0.3) * np.sign(angle_rad)
        timeout   = task.get("timeout", 15)
        
        if self.task_state == 0:
            print("% [run_turn] Starting turn")
            service.send("robobot/cmd/T0", "leds 16 0 100 0")
            service.send("robobot/cmd/ti", f"rc 0.0 {turn_rate:.3f}")  # ← f-string, not "rc turn_rate"
            self.task_state = 1

        elif self.task_state == 1:
            if abs(pose.tripBh) >= abs(angle_rad) or pose.tripBtimePassed() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.task_state = 2

        
        elif self.task_state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                print("% [run_turn] Turn complete")
                self.next_task()


    def run_drive_straight(self, task):
        """
        Drive straight for a fixed distance using odometry (no line sensor).

        Task keys:
            distance    metres (default 1.0)
            speed       m/s (default 0.2)
            timeout     seconds (default 20)
        """
        speed    = task.get("speed",    0.2)
        distance = task.get("distance", 1.0)
        timeout  = task.get("timeout",  20)

        if self.task_state == 0:
            service.send("robobot/cmd/ti", f"rc {speed} 0.0")
            self.task_state = 1

        elif self.task_state == 1:
            if pose.tripB >= distance or self.task_time() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.task_state = 2

        elif self.task_state == 2:
            if abs(pose.velocity()) < 0.005:
                self.next_task()


    def run_drive_circle(self, task):
        """
        Drive a full circle using odometry.
        Task keys:
            radius      metres (default 0.3)
            speed       forward speed m/s (default 0.15)
            direction   'left' (CCW) or 'right' (CW) (default 'left')
            timeout     seconds (default 20)
        """
        radius    = task.get("radius",    0.3)
        speed     = task.get("speed",     0.15)
        direction = task.get("direction", "left")
        timeout   = task.get("timeout",   20)

        # positive turn_rate = left, negative = right
        sign      = 1 if direction == "left" else -1
        turn_rate = sign * (speed / radius)

        if self.task_state == 0:
            print(f"% [drive_circle] Starting circle r={radius}m, speed={speed}m/s, {direction}")
            service.send("robobot/cmd/T0", "leds 16 0 100 0")
            service.send("robobot/cmd/ti", f"rc {speed:.3f} {turn_rate:.3f}")
            self.task_state = 1

        elif self.task_state == 1:
            # full circle = 2π radians of heading
            if abs(pose.tripBh) >= 2 * np.pi or pose.tripBtimePassed() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% [drive_circle] Circle done, heading={pose.tripBh:.3f} rad")
                self.task_state = 2

        elif self.task_state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                print("% [drive_circle] Stopped — next task")
                self.next_task()


    def run_turn_to_heading(self, task):
        target_rad = np.radians(task["heading_deg"])
        speed      = task.get("speed",         0.3)
        tolerance  = np.radians(task.get("tolerance_deg", 0.1))
        timeout    = task.get("timeout",       15)

        if self.task_state == 0:
            diff = target_rad - pose.pose[2]
            if diff > np.pi:
                diff -= 2 * np.pi
            elif diff < -np.pi:
                diff += 2 * np.pi
            self._turn_sign = 1 if diff > 0 else -1
            print(f"% [turn_to_heading] current={np.degrees(pose.pose[2]):.1f}°, target={task['heading_deg']}°, diff={np.degrees(diff):.1f}°")
            service.send("robobot/cmd/ti", f"rc 0.0 {self._turn_sign * speed:.3f}")
            self.task_state = 1

        elif self.task_state == 1:
            diff = target_rad - pose.pose[2]
            if diff > np.pi:
                diff -= 2 * np.pi
            elif diff < -np.pi:
                diff += 2 * np.pi
            if abs(diff) <= tolerance or pose.tripBtimePassed() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% [turn_to_heading] Reached {np.degrees(pose.pose[2]):.1f}°")
                self.task_state = 2

        elif self.task_state == 2:
            if abs(pose.turnrate()) < 0.001:
                self.next_task()