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
from sfuse import iwo
from sedge import edge
from uservice import service
from sir import ir

import threading
from line_vision import line_follow_vision
from roundabout_vision import drive_roundabout

class MissionRunner:

    def __init__(self, tasks, total_time, goal_time_buffer):
        self.tasks = tasks
        self.total_time = total_time              # seconds allowed for full mission
        self.goal_time_buffer = goal_time_buffer  # go to goal if less than this left
        self.mission_start = None
        self.task_index = 0
        self.task_state = 0
        self.task_start = None
        self._recovery_start = None
        self._crossing_count = 0
        self._on_crossing    = False
        self._crossing_start = None
        self.stop = False

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
        if self.task_index + 1 < len(self.tasks):
            self.start_task(self.task_index + 1)
        else:
            self.stop = True

    # main loop _________________________________

    def run(self):
        self.mission_start = datetime.now()
        self.start_task(0)
        service.send("robobot/cmd/T0", "leds 16 0 0 30")   # blue = running

        while not self.stop and not service.stop:
            current_task = self.tasks[self.task_index]
            task_type    = current_task["type"]

            # dispatch to executor
            if task_type == "line_follow":
                self.run_line_follow(current_task)

            elif task_type == "line_follow_brake":
                self.run_line_follow_brake(current_task)

            elif task_type == "line_follow_brake_crossing":
                self.run_line_follow_brake_crossing(current_task)

            elif task_type == "drive_straight":
                self.run_drive_straight(current_task)

            elif task_type == "turn":
                self.run_turn(current_task)
            
            elif task_type == "drive_circle":
                self.run_drive_circle(current_task)

            elif task_type == "turn_to_heading":
                self.run_turn_to_heading(current_task)

            elif task_type == "line_follow_vision":
                self.run_line_follow_vision(current_task)

            elif task_type == "roundabout_vision":
                self.run_roundabout_vision(current_task)

            elif task_type == "drive_to_xy":
                self.run_drive_to_xy(current_task)
                
            elif task_type == "line_follow_until_xy":
                self.run_line_follow_until_xy(current_task)

            elif task_type == "drive_curved":
                self.run_drive_curved(current_task)

            elif task_type == "drive_straight_heading":
                self.run_drive_straight_heading(current_task)

            elif task_type == "drive_roundabout":
                self.run_drive_roundabout(current_task)
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
        edge.CUSTOM_CONTROL_ENABLED = True

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


    def run_line_follow_vision(self, task):
        """
        Follow a line using vision (camera-based controller running in background).
        Stops on timeout or when externally stopped.
        """

        timeout = task.get("timeout", 30)
        junction = task.get("junction", 'straight')
        start_speed = task.get("start_speed", 0)
        nominal_speed = task.get("nominal_speed", 0.4)
        turn_sensitivity = task.get("turn_sensitivity", 0.005)
        stop_speed = task.get("stop_speed", 0)
        stop_time = task.get("stop_time", 0)
        time_to_full_speed = task.get("time_to_full_speed", 0)
        min_pixels_to_detect_line = task.get("min_pixels_to_detect_line", -1)



        if self.task_state == 0:
            print("% [line_follow_vision] Starting vision-based line following")

            service.send("robobot/cmd/T0", "leds 16 0 100 0")  # green

            # Start vision controller in background
            self._vision_thread = threading.Thread(target=line_follow_vision,
                                                   kwargs={"junctions": junction,
                                                           "start_speed": start_speed,
                                                           "nominal_speed": nominal_speed,
                                                           "sensitivity": turn_sensitivity,
                                                           "stop_speed": stop_speed,
                                                           "time_to_full_speed": time_to_full_speed,
                                                           "stop_time": stop_time,
                                                           "min_pixels_to_detect_line": min_pixels_to_detect_line,
                                                           "timeout": timeout},
                                                   daemon=True)
            self._vision_thread.start()
            self.task_state = 1
            
        elif self.task_state == 1:
            # Just wait while vision is controlling the robot

            if not self._vision_thread.is_alive():
                print("% [line_follow_vision] Vision thread stopped")
                if stop_speed <= 0:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.task_state = 2

            elif self.task_time() > timeout: # is normally already implemented in line_vision
                print("% [line_follow_vision] Timeout — stopping")
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.task_state = 2

        elif self.task_state == 2:
            # wait for robot to fully stop
            if stop_speed <= 0:
                print("% [line_follow_vision] Stopped — next task")
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.next_task()
            else:
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
                self._recovery_start = None
                self.task_state = 2
            elif pose.tripB > 4.0 or self.task_time() > timeout:
                service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
                self.task_state = 3

        elif self.task_state == 2:  # following line
            normal_speed = task.get("speed",       0.15)
            brake_speed  = task.get("brake_speed", -0.05)
            max_speed    = task.get("max_speed",    0.25)
            recovery     = task.get("recovery",    False)
            recovery_time = task.get("recovery_time", 1.0)

            # check actual velocity and adjust
            if pose.velocity() > max_speed:
                edge.lineControl(brake_speed, follow_left)
            else:
                edge.lineControl(normal_speed, follow_left)

            if self.task_time() > timeout:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("% [line_follow] Timeout")
                self.task_state = 3

            elif edge.lineValidCnt < 2:
                if recovery:
                    # start recovery timer if not already started
                    if self._recovery_start is None:
                        self._recovery_start = datetime.now()
                        print("% [line_follow] Line lost — waiting to recover")
                    # check if recovery time has expired
                    elif (datetime.now() - self._recovery_start).total_seconds() > recovery_time:
                        edge.lineControl(0, True)
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        print("% [line_follow] Recovery failed — stopping")
                        self._recovery_start = None
                        self.task_state = 3
                    # else: still within recovery window, keep going
                else:
                    # no recovery — stop immediately
                    edge.lineControl(0, True)
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    print(f"% [line_follow] Line lost at {pose.tripB:.3f}m")
                    self.task_state = 3

            else:
                # line is valid — reset recovery timer
                self._recovery_start = None

        elif self.task_state == 3: # stopping
            if abs(pose.velocity()) < 0.001:
                print("% [line_follow] Stopped — next task")
                self.next_task()


    def run_line_follow_brake_crossing(self, task):
        """
        Follow a line on the chosen side with:
        - speed control / braking based on actual velocity
        - recovery: wait before giving up if line is lost
        - crossing detection: stop at nth intersection

        Task keys:
            side                'left' or 'right'
            speed               forward speed m/s (default 0.2)
            brake_speed         negative speed when too fast (default -0.05)
            max_speed           velocity threshold to trigger braking (default 0.25)
            timeout             max seconds for whole task (default 30)
            recovery            True/False — wait before giving up on line loss (default False)
            recovery_time       seconds to wait before giving up (default 1.0)
            stop_at_crossing    True/False (default False)
            crossing_count      which crossing to stop at (default 1)
        """
        follow_left      = (task["side"] == "left")
        speed            = task.get("speed",            0.2)
        brake_speed      = task.get("brake_speed",      -0.05)
        max_speed        = task.get("max_speed",        0.25)
        timeout          = task.get("timeout",          30)
        recovery         = task.get("recovery",         False)
        recovery_time    = task.get("recovery_time",    1.0)
        stop_at_crossing = task.get("stop_at_crossing", False)
        crossing_target  = task.get("crossing_count",   1)

        # ── state 0: initialise ───────────────────────────────────────────────────
        if self.task_state == 0:
            print("% [line_follow] Starting — moving forward to find line")
            service.send("robobot/cmd/T0", "leds 16 0 100 0")
            service.send("robobot/cmd/ti", f"rc {speed:.3f} 0.0")
            self._recovery_start  = None
            self._crossing_count  = 0
            self._on_crossing     = False
            self.task_state = 1

        # ── state 1: approach — drive forward until line found ────────────────────
        elif self.task_state == 1:
            if edge.lineValidCnt > 4:
                edge.lineControl(speed, follow_left)
                pose.tripBreset()
                self._recovery_start = None
                print("% [line_follow] Line found — following")
                self.task_state = 2
            elif self.task_time() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("% [line_follow] Timeout during approach")
                self.task_state = 3

        # ── state 2: follow line ──────────────────────────────────────────────────
        elif self.task_state == 2:

            if edge.crossingLineCnt>0:
                print(f"% average={edge.average:.1f}, crossingLineCnt={edge.crossingLineCnt}")

            # ── speed control: brake if going too fast ────────────────────────────
            if pose.velocity() > max_speed:
                edge.lineControl(brake_speed, follow_left)
            else:
                edge.lineControl(speed, follow_left)

            # ── crossing detection ────────────────────────────────────────────────
            if self._on_crossing:
                if edge.crossingLineCnt < 5:
                    edge.crossingOverride = False
                    print(f"% [line_follow] Crossing passed — resuming line follow")
                    edge.lineControl(speed, follow_left)
                    self._on_crossing = False
                else:
                    # actively keep sending straight — overrides any stray followLine() calls
                    service.send("robobot/cmd/ti", f"rc {speed:.3f} 0.0")

            elif edge.crossingLineCnt >= 10:
                if not self._on_crossing:
                    self._crossing_count += 1
                    if self._crossing_count < crossing_target and stop_at_crossing:
                        self._on_crossing = True
                        self._crossing_start = datetime.now()
                        print(f"% [line_follow] Crossing {self._crossing_count} — driving straight through")
                        edge.crossingOverride = True
                        edge.lineControl(0, True)
                        service.send("robobot/cmd/ti", f"rc {speed:.3f} 0.0")
                    else:
                        edge.crossingOverride = False
                        edge.lineControl(0, True)
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        print(f"% [line_follow] Target crossing reached — stopping")
                        self.task_state = 3

            # ── timeout ───────────────────────────────────────────────────────────
            if self.task_state == 2 and pose.tripBtimePassed() > timeout:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("% [line_follow] Timeout while following")
                self.task_state = 3

            # ── line lost: recovery or immediate stop ─────────────────────────────
            elif self.task_state == 2 and edge.lineValidCnt < 2:
                if recovery:
                    if self._recovery_start is None:
                        self._recovery_start = datetime.now()
                        print("% [line_follow] Line lost — waiting to recover")
                    elif (datetime.now() - self._recovery_start).total_seconds() > recovery_time:
                        edge.lineControl(0, True)
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        print("% [line_follow] Recovery failed — stopping")
                        self._recovery_start = None
                        self.task_state = 3
                    # else: still within recovery window — keep going
                else:
                    edge.lineControl(0, True)
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    print(f"% [line_follow] Line lost at {pose.tripB:.3f}m")
                    self.task_state = 3

            # ── line valid: reset recovery timer ──────────────────────────────────
            elif self.task_state == 2 and edge.lineValidCnt >= 2:
                self._recovery_start = None

        # ── state 3: wait for full stop ───────────────────────────────────────────
        elif self.task_state == 3:
            if abs(pose.velocity()) < 0.001:
                print("% [line_follow] Stopped — next task")
                self.next_task()

    def run_roundabout_vision(self, task):
        """
        Drive roundabout using vision controller (runs in background).
        Stops on timeout.
        """

        timeout = task.get("timeout", 30)
        speed = task.get("speed", 0.4)
        start_speed = task.get("start_speed", 0)
        stop_speed = task.get("stop_speed", 0)
        time_to_full_speed = task.get("time_to_full_speed", 0.4)
        Kp = task.get("Kp", 6)
        Ki = task.get("Ki", 0.2)
        percentage_of_white = task.get("percentage_of_white", 30)

        if self.task_state == 0:
            print("% [roundabout_vision] Starting vision-based roundabout")
            service.send("robobot/cmd/T0", "leds 16 100 100 0")  # yellow (different from line follow)

            self._vision_thread = threading.Thread(target=drive_roundabout(speed, start_speed, stop_speed, time_to_full_speed, Kp=Kp, Ki=Ki, percentage_of_white=percentage_of_white),daemon=True)
            self._vision_thread.start()


            self.task_state = 1

        elif self.task_state == 1:
            # Let vision control everything
            if not self._vision_thread.is_alive():
                print("% [roundabout_vision] Vision thread stopped")
                if stop_speed <= 0:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.task_state = 2

            elif self.task_time() > timeout:
                print("% [roundabout_vision] Timeout — stopping")
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.task_state = 2

        elif self.task_state == 2:
            # Wait until robot actually stops
            print("% [roundabout_vision] Stopped — next task")
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
        is_stopping_at_end = task.get("is_stopping_at_end", True)

        if self.task_state == 0:
            print("[run_drive_straight] Start driving forwards")
            service.send("robobot/cmd/ti", f"rc {speed} 0.0")
            self.task_state = 1

        elif self.task_state == 1:
            if pose.tripB >= distance or self.task_time() > timeout:
                if is_stopping_at_end:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.task_state = 2

        elif self.task_state == 2:
            print("[run_drive_straight] Ended Task")
            self.next_task()

    def run_drive_curved(self, task):
        """
        Drive curved until a certain heading is reached.
        """
        nominal_speed     = task.get("nominal_speed",     0.2)
        stop_speed = task.get("stop_speed", 0)
        turn_rate = task.get("turn_rate", 0.01)
        final_turn_rate = task.get("final_turn_rate", 0)
        target_rad = np.radians(task["heading_deg"])
        tolerance = np.radians(task["tolerance"])
        timeout   = task.get("timeout",   20)

        start_time = t.time()

        if self.task_state == 0:
            diff = target_rad - pose.pose[2]
            if diff > np.pi:
                diff -= 2 * np.pi
            elif diff < -np.pi:
                diff += 2 * np.pi
            self._turn_sign = 1 if diff > 0 else -1
            print(f"% [run_drive_curved] current={np.degrees(pose.pose[2]):.1f}°, target={task['heading_deg']}°, diff={np.degrees(diff):.1f}°")
            service.send("robobot/cmd/ti", f"rc {nominal_speed} {self._turn_sign * turn_rate:.3f}")
            self.task_state = 1

        elif self.task_state == 1:
            diff = target_rad - pose.pose[2]
            if diff > np.pi:
                diff -= 2 * np.pi
            elif diff < -np.pi:
                diff += 2 * np.pi
            if abs(diff) <= tolerance or pose.tripBtimePassed() > timeout:
                if stop_speed <= 0:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                else:
                    service.send("robobot/cmd/ti", f"rc {nominal_speed} {final_turn_rate}")
                print(f"% [run_drive_curved] Reached {np.degrees(pose.pose[2]):.1f}°")
                self.task_state = 2

        elif self.task_state == 2:
            self.next_task()

    def run_drive_roundabout(self, task):
        """
        Drive curved until a certain heading is reached.
        """
        nominal_speed     = task.get("nominal_speed",     0.4)
        stop_speed = task.get("stop_speed", 0)
        turn_rate = task.get("turn_rate", 1.15)
        final_turn_rate = task.get("final_turn_rate", 0)
        target_rad = np.radians(task["heading_deg"])
        tolerance = np.radians(task["tolerance"])
        timeout   = task.get("timeout",   20)
        min_run_time = task.get("min_run_time", 2.5)

        start_time = t.time()

        if self.task_state == 0:
            print("[run_drive_roundabout] Start driving roundabout")
            service.send("robobot/cmd/ti", f"rc {nominal_speed} {turn_rate}")
            self.task_state = 1

        elif self.task_state == 1:
            diff = target_rad - pose.pose[2]
            if diff > np.pi:
                diff -= 2 * np.pi
            elif diff < -np.pi:
                diff += 2 * np.pi
            if t.time() > min_run_time and abs(diff) <= tolerance or pose.tripBtimePassed() > timeout:
                if stop_speed <= 0:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                else:
                    service.send("robobot/cmd/ti", f"rc {stop_speed} {final_turn_rate}")
                print(f"% [run_drive_roundabout] Reached {np.degrees(pose.pose[2]):.1f}°")
                self.task_state = 2

        elif self.task_state == 2:
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

    def run_drive_to_xy(self, task):
        """
        Drives to a specific global X, Y coordinate.
        """
        import sys
        target_x = task["target_x"]
        target_y = task["target_y"]
        speed = task.get("speed", 0.2)
        timeout = task.get("timeout", 30)
        dist_tol = task.get("distance_tolerance", 0.05)
        
        curr_x, curr_y, curr_h = pose.pose[0], pose.pose[1], pose.pose[2]
        dx = target_x - curr_x
        dy = target_y - curr_y
        distance = np.hypot(dx, dy)
        target_heading = np.arctan2(dy, dx)
        
        heading_error = target_heading - curr_h
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        if self.task_state == 0:
            print(f"\n% [drive_to_xy] Turning towards {target_x}, {target_y}")
            self.task_state = 1
            
        elif self.task_state == 1: # Turn towards point
            if abs(heading_error) > 0.05:
                turn_speed = max(min(0.8 * heading_error, 1.0), -1.0)
                service.send("robobot/cmd/ti", f"rc 0.0 {turn_speed:.3f}")
            else:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"\n% [drive_to_xy] Facing target. Driving...")
                self.task_state = 2
                
        elif self.task_state == 2: # Drive to point
            # print pos and heading
            sys.stdout.write(f"\r% [drive_to_xy] Current X:{curr_x:.3f} Y:{curr_y:.3f} Heading: {curr_h:.3f} | Target X:{target_x:.3f} Y:{target_y:.3f} | Dist:{distance:.3f}   ")
            sys.stdout.flush()

            if distance > dist_tol and self.task_time() < timeout:
                turn_speed = max(min(0.5 * heading_error, 1.0), -1.0)
                service.send("robobot/cmd/ti", f"rc {speed:.3f} {turn_speed:.3f}")
            else:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"\n\n% [drive_to_xy] Arrived near {target_x}, {target_y}")
                self.next_task()

    def run_line_follow_until_xy(self, task):
        """
        Follows a line until the robot's odometry reaches a specific global (X, Y) coordinate.
        """
        import sys
        
        follow_left = (task.get("side", "left") == "left")
        speed = task.get("speed", 0.2)
        target_x = task["target_x"]
        target_y = task["target_y"]
        dist_tol = task.get("distance_tolerance", 0.1)
        timeout = task.get("timeout", 40)

        if self.task_state == 0:
            print(f"\n% [line_follow_until_xy] Starting — moving to find line towards ({target_x}, {target_y})")
            service.send("robobot/cmd/T0", "leds 16 0 100 0") # green
            service.send("robobot/cmd/ti", f"rc {speed:.3f} 0.0")
            self.task_state = 1

        elif self.task_state == 1: # approaching line
            if edge.lineValidCnt > 4:
                edge.lineControl(speed, follow_left)
                print("\n% [line_follow_until_xy] Line found — following and monitoring coordinates")
                self.task_state = 2
                    
            elif self.task_time() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("\n% [line_follow_until_xy] Timeout during approach")
                self.task_state = 3

        elif self.task_state == 2: # following line
            curr_x, curr_y = pose.pose[0], pose.pose[1]
            distance = np.hypot(target_x - curr_x, target_y - curr_y)


            # print current x,y
            sys.stdout.write(f"\r% [line_follow_until_xy] Current X:{curr_x:.3f} Y:{curr_y:.3f} | Target X:{target_x:.3f} Y:{target_y:.3f} | Dist:{distance:.3f}   ")
            sys.stdout.flush()

            if distance <= dist_tol:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"\n\n% [line_follow_until_xy] REACHED TARGET: ({target_x}, {target_y})!")
                self.task_state = 3
                
            elif self.task_time() > timeout:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("\n\n% [line_follow_until_xy] Timeout while following")
                self.task_state = 3
                
            elif edge.lineValidCnt < 2:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"\n\n% [line_follow_until_xy] Line lost early at X:{curr_x:.3f}, Y:{curr_y:.3f}")
                self.task_state = 3

        elif self.task_state == 3: # wait for full stop
            if abs(pose.velocity()) < 0.001:
                print("% [line_follow_until_xy] Stopped — next task")
                self.next_task()

    def run_drive_straight_heading(self, task):
        """
        Drives a set distance (forward or backward) while actively steering 
        to maintain a specific global heading using the IMU/Odometry.
        """
        speed = task.get("speed", 0.2)
        distance = task.get("distance", 1.0)
        target_heading_deg = task.get("heading_deg", 0)
        timeout = task.get("timeout", 20)
        
        target_heading_rad = np.radians(target_heading_deg)

        if self.task_state == 0:
            pose.tripBreset()
            print(f"\n% [drive_heading] Locking heading to {target_heading_deg}° and driving {distance}m")
            self.task_state = 1

        elif self.task_state == 1:
            if pose.tripB < distance and self.task_time() < timeout:
                # calculate heading error
                curr_h = pose.pose[2]
                heading_error = target_heading_rad - curr_h
                heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

                # P controller to correct heading drift
                turn_speed = max(min(1.0 * heading_error, 1.0), -1.0)
                
                # send combined forward/reverse + turn command
                service.send("robobot/cmd/ti", f"rc {speed:.3f} {turn_speed:.3f}")
            else:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% [drive_heading] Distance reached.")
                self.next_task()