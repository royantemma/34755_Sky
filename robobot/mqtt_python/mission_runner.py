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
from simu import imu
from sedge import edge
from uservice import service
from sir import ir

import threading
from line_vision import line_follow_vision
from roundabout_vision import drive_roundabout
from eight_vision import wait_for_police
from golf import get_ball_location
from golf import get_ball_location_sorting

from SKY114 import go_to_xy, go_to_xy_fast

import golf

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

        self.initial_pitch = None # used in ramp detection
        self.police_pass_time = None

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
        print("[Mission Runner] Pulling servo up")
        #service.send("robobot/cmd/T0","servo 1 -930 400")
        service.send("robobot/cmd/T0","servo 1 -890 400")
        t.sleep(1)
        self.start_task(0)
        service.send("robobot/cmd/T0", "leds 16 0 0 30")   # blue = running

        while not self.stop and not service.stop and self.task_index < len(self.tasks):
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

            elif task_type == "correct_IWO":
                self.correct_IWO(current_task)
            elif task_type == "reset_IWO":
                self.reset_IWO(current_task)

            elif task_type == "roundabout_vision":
                self.run_roundabout_vision(current_task)

            elif task_type == "drive_to_xy":
                self.run_drive_to_xy(current_task)

            elif task_type == "drive_to_xyyaw_IWO":
                self.run_drive_to_xyyaw_IWO(current_task)

            elif task_type == "drive_to_xyyaw_IWO_fast":
                self.run_drive_to_xyyaw_IWO_fast(current_task)
                
            elif task_type == "line_follow_until_xy":
                self.run_line_follow_until_xy(current_task)
            elif task_type == "linefollow_until_ramp_IWO":
                self.run_linefollow_until_ramp_IWO(current_task)

            elif task_type == "line_follow_until_xy_IWO":
                self.run_line_follow_until_xy_IWO(current_task)

            elif task_type == "drive_curved":
                self.run_drive_curved(current_task)

            elif task_type == "drive_curved_distance":
                self.run_drive_curved_distance(current_task)
            
            elif task_type == "drive_curved_iwo":
                self.run_drive_curved_iwo(current_task)

            elif task_type == "drive_straight_heading":
                self.run_drive_straight_heading(current_task)

            elif task_type == "drive_straight_crossing_x_lines":
                self.run_drive_straight_crossing_x_lines(current_task)

            elif task_type == "drive_roundabout":
                self.run_drive_roundabout(current_task)

            elif task_type == "wait_for_police":
                self.wait_for_police(current_task)

            elif task_type == "turn_to_heading_iwo":
                self.run_turn_to_heading_iwo(current_task)
            
            elif task_type == "servo_up":
                self.run_servo_up(current_task)
            
            elif task_type == "servo_down":
                self.run_servo_down(current_task)
            
            elif task_type == "servo_mid":
                self.run_servo_mid(current_task)
            
            elif task_type == "servo_relax":
                self.run_servo_relax(current_task)

            elif task_type == "get_ball":
                self.run_get_ball(current_task)
            elif task_type == "put_ball_in_hole":
                self.run_put_ball_in_hole(current_task)
            
            elif task_type == "drive_until_line_detection":
                self.run_drive_until_line_detection(current_task)

            elif task_type == "linefollow_until_x_intersections":
                self.run_linefollow_until_x_intersections(current_task)

            elif task_type == "golf_visual_approach_and_trap":
                self.run_golf_visual_approach_and_trap(current_task)

            elif task_type == "stairs_down":
                self.run_stairs_down(current_task)
            
            elif task_type == "luggage_intercept":
                if self.run_luggage_catch(current_task):
                    self.task_index += 1
                else:
                    self.task_index += 1 

            elif task_type == "run_get_ball_sorting":
                self.run_get_ball_sorting(current_task)
                self.task_index += 1

            elif task_type == "servo_command":
                cmd = current_task.get("servo_string", "")
                sleep_time = current_task.get("sleep_after", 0)
                print(f"% Executing Servo Command: {cmd}")
                service.send("robobot/cmd/T0", cmd)
                if sleep_time > 0:
                    t.sleep(sleep_time)
                self.task_index += 1

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
        stop_speed = task.get("stop_speed", 0)
        timeout = task.get("timeout", 30)
        max_distance = task.get("max_distance", None)
        edge.CUSTOM_CONTROL_ENABLED = True

        if self.task_state == 0:
            print("% [line_follow] Starting — moving forward to find line")
            service.send("robobot/cmd/T0", "leds 16 0 100 0") # green
            service.send("robobot/cmd/ti",f"rc {speed:.3f} 0.0") # (forward m/s, turn-rate rad/sec)
            self.task_state = 1

        elif self.task_state == 1: # approaching line
            if edge.lineValidCnt > 4:
                pose.tripBreset()
                edge.lineControl(speed, follow_left)
                
                print("% [line_follow] Line found — following")
                self.task_state = 2
            elif pose.tripB > 4.0 or self.task_time() > timeout:
                service.send("robobot/cmd/ti/",f"rc {stop_speed:.3f} 0.0") # (forward m/s, turn-rate rad/sec)
                self.task_state = 3

        elif self.task_state == 2: # following line
            if self.task_time() > timeout:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti/",f"rc {stop_speed:.3f} 0.0") # (forward m/s, turn-rate rad/sec)
                print("% [line_follow] Timeout")
                self.task_state = 3
            elif max_distance is not None and pose.tripB > max_distance:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti/",f"rc {stop_speed:.3f} 0.0") # (forward m/s, turn-rate rad/sec)
                print("% [line_follow] Max distance reached")
                self.task_state = 3
            elif edge.lineValidCnt < 2:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti/",f"rc {stop_speed:.3f} 0.0") # (forward m/s, turn-rate rad/sec)
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
        percentage_height = task.get("percentage_height", 50)
        final_heading = task.get("final_heading", None)
        time_after_turn = task.get("time_after_turn", None)
        time_before_ramp_pitch = task.get("time_before_ramp_pitch", None)
        time_before_ramp_heading = task.get("time_before_ramp_heading", None)



        if self.task_state == 0:
            print("% [line_follow_vision] Starting vision-based line following")

            #service.send("robobot/cmd/T0", "leds 16 0 100 0")  # green

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
                                                           "timeout": timeout,
                                                           "percentage_height": percentage_height,
                                                           "final_heading": final_heading,
                                                           "time_after_turn": time_after_turn,
                                                           "time_before_ramp_pitch": time_before_ramp_pitch,
                                                           "time_before_ramp_heading": time_before_ramp_heading},
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

            elif self.task_time() > timeout+1: # is normally already implemented in line_vision
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
        nominal_speed    = task.get("speed",    0.2)
        start_speed = task.get("start_speed", nominal_speed)
        distance = task.get("distance", 1.0)
        timeout  = task.get("timeout",  20)
        is_stopping_at_end = task.get("is_stopping_at_end", True)
        time_to_full_speed = task.get("time_to_full_speed", 0.02)


        if self.task_state == 0:
            print("[run_drive_straight] Start driving forwards")
            service.send("robobot/cmd/ti", f"rc {start_speed} 0.0")
            self.task_state = 1

        elif self.task_state == 1:
            if pose.tripB >= distance or self.task_time() > timeout:
                if is_stopping_at_end:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.task_state = 2
            else:
                speed = start_speed + (nominal_speed - start_speed) * min(1, self.task_time()/time_to_full_speed)
                service.send("robobot/cmd/ti", f"rc {speed} 0.0")

        elif self.task_state == 2:
            print("[run_drive_straight] Ended Task")
            self.next_task()

    def run_drive_curved_distance(self, task):
        """
        Drive straight for a fixed distance using odometry (no line sensor).

        Task keys:
            distance    metres (default 1.0)
            speed       m/s (default 0.2)
            timeout     seconds (default 20)
        """
        speed    = task.get("speed",    0.2)
        turn_rate = task.get("turn_rate", 0.01)
        distance = task.get("distance", 1.0)
        timeout  = task.get("timeout",  20)
        is_stopping_at_end = task.get("is_stopping_at_end", True)

        if self.task_state == 0:
            print("[run_drive_curved_distance] Start driving forwards")
            service.send("robobot/cmd/ti", f"rc {speed} {turn_rate}")
            self.task_state = 1

        elif self.task_state == 1:
            if pose.tripB >= distance or self.task_time() > timeout:
                if is_stopping_at_end:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.task_state = 2

        elif self.task_state == 2:
            print("[run_drive_curved_distance] Ended Task")
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


    def run_drive_curved_iwo(self, task):
        """
        Drive curved until a certain heading is reached.
        """
        nominal_speed     = task.get("nominal_speed",     0.2)
        start_speed = task.get("start_speed", nominal_speed)
        stop_speed = task.get("stop_speed", 0)
        turn_rate = task.get("turn_rate", 0.01)
        final_turn_rate = task.get("final_turn_rate", 0)
        target_deg = task["heading_deg"]
        tolerance = task.get("tolerance", 2.0)
        timeout   = task.get("timeout",   20)
        time_to_full_speed = task.get("time_to_full_speed", 0.02)


        if self.task_state == 0:
            diff = target_deg - iwo.fused_yaw
            if diff > 180:
                diff -= 2 * 180
            elif diff < -180:
                diff += 2 * 180
            self._turn_sign = 1 if diff > 0 else -1
            print(f"% [run_drive_curved_iwo] current={iwo.fused_yaw:.1f}°, target={task['heading_deg']}°, diff={diff:.1f}°")
            service.send("robobot/cmd/ti", f"rc {start_speed} {self._turn_sign * turn_rate:.3f}")
            self.task_state = 1

        elif self.task_state == 1:
            speed = start_speed + (nominal_speed - start_speed) * min(1, self.task_time()/time_to_full_speed)
            diff = target_deg - iwo.fused_yaw
            if diff > 180:
                diff -= 2 * 180
            elif diff < -180:
                diff += 2 * 180
            if abs(diff) <= tolerance or pose.tripBtimePassed() > timeout:
                if stop_speed <= 0:
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                else:
                    service.send("robobot/cmd/ti", f"rc {speed} {final_turn_rate}")
                print(f"% [run_drive_curved_iwo] Reached {iwo.fused_yaw:.1f}°")
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

    def run_turn_to_heading_iwo(self, task):
        target_deg = task["heading_deg"]
        speed      = task.get("speed",         0.3)
        tolerance  = task.get("tolerance_deg", 0.1)
        timeout    = task.get("timeout",       15)
        pose.tripBreset()  # reset distance timer to measure timeout from start of turn

        if self.task_state == 0:
            diff = target_deg - iwo.fused_yaw
            if diff > 180:
                diff -= 2 * 180
            elif diff < -180:
                diff += 2 * 180
            self._turn_sign = 1 if diff > 0 else -1
            print(f"% [turn_to_heading_iwo] current={iwo.fused_yaw:.1f}°, target={task['heading_deg']}°, diff={diff:.1f}°")
            service.send("robobot/cmd/ti", f"rc 0.0 {self._turn_sign * speed:.3f}")
            self.task_state = 1

        elif self.task_state == 1:
            diff = target_deg - iwo.fused_yaw
            if diff > 180:
                diff -= 2 * 180
            elif diff < -180:
                diff += 2 * 180
            if abs(diff) <= tolerance or pose.tripBtimePassed() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% [turn_to_heading_iwo] Reached {iwo.fused_yaw:.1f}°")
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

    def run_golf_visual_approach_and_trap(self, task):
        timeout = task.get("timeout", 15)
        Kp_fwd = task.get("forward_gain", 0.5)
        Kp_turn = task.get("turn_gain", 1.0)
        
        success = golf.capture_ball(timeout, Kp_fwd, Kp_turn)
        
        if not success:
            print("Warning: Failed to capture the ball")
    
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


    def wait_for_police(self, task):
        """
        Turn in place using odometry.

        Task keys:
            angle_deg   degrees; positive = left/CCW, negative = right/CW
            speed       turn rate rad/s (default 0.3)
            timeout     seconds (default 15)
        """
        timeout   = task.get("timeout", 20)
        percentage_height = task.get("percentage_height", 60)
        min_time_between_checks = task.get("min_time_between_frames", 0.2)
        min_free_frames_to_move = task.get("min_free_frames_to_move", 5)
        min_distance_for_movement = task.get("min_distance_for_movement", 5)
        require_movement_before_start = task.get("require_movement_before_start", False)
        percentage_from_bottom = task.get("percentage_from_bottom", 0)
        
        if self.task_state == 0:
            print("% [wait_for_police] Waiting for clear space")
            service.send("robobot/cmd/T0", "leds 16 0 100 0")
            service.send("robobot/cmd/ti", f"rc 0.0 0") 
            # Start vision controller in background
            self._vision_thread = threading.Thread(target=wait_for_police(percentage_height=percentage_height, time_between_checks=min_time_between_checks, min_distance_for_movement=min_distance_for_movement, min_free_frames_to_move=min_free_frames_to_move, require_movement_before_start=require_movement_before_start, percentage_from_bottom=percentage_from_bottom), daemon=True)
            self._vision_thread.start()
            self.task_state = 1

        elif self.task_state == 1:
            if not self._vision_thread.is_alive():
                print("% [wait_for_police] Clear space detected")
                self.police_pass_time = t.time()
                self.task_state = 2
            elif pose.tripBtimePassed() > timeout:
                print("% [wait_for_police] Timeout, continuing further with mission")
                self.police_pass_time = t.time()
                self.task_state = 2

        
        elif self.task_state == 2:
            print("% [wait_for_police] Stopped - next task")
            self.next_task()
    
    def run_drive_to_xyyaw_IWO(self, task):
        """
        Drives to a specific global X, Y coordinate using IWO
        """
        just_yaw = task.get("just_yaw", False)
        if not just_yaw:
            target_x = task["target_x"]
            target_y = task["target_y"]
        target_yaw = task.get("target_yaw", None)
        max_speed = task.get("max_speed", 0.8)
        timeout = task.get("timeout", 30)
        dist_tol = task.get("distance_tolerance", 0.02)
        print("current position: ({:.3f}, {:.3f}), fused yaw: {:.1f}".format(iwo.fused_x, iwo.fused_y, iwo.fused_yaw))
        
        if just_yaw and target_yaw is not None:
            target_x = iwo.fused_x
            target_y = iwo.fused_y

        
        if self.task_state == 0:
            print(f"\n% [drive_to_xy] Turning towards {target_x}, {target_y}")
            go_to_xy(target_x, target_y, target_yaw, max_speed=max_speed,dist_tolerance=dist_tol)
            self.task_state = 1
            
        elif self.task_state == 1:
            print(f"\n% [drive_to_xy] Tasked finished - Next task")
            self.next_task()


    def run_drive_to_xyyaw_IWO_fast(self, task):
        """
        Drives to a specific global X, Y coordinate using IWO
        """
        
        target_x = task["target_x"]
        target_y = task["target_y"]
        
        target_yaw = task.get("target_yaw", None)
        max_speed = task.get("max_speed", 0.8)
        timeout = task.get("timeout", 30)
        dist_tol = task.get("distance_tolerance", 0.02)
        
        
        if self.task_state == 0:
            print(f"\n% [drive_to_xy] Turning towards {target_x}, {target_y}")
            go_to_xy_fast(target_x, target_y, target_yaw, max_speed=max_speed,dist_tolerance=dist_tol)
            self.task_state = 1
            
        elif self.task_state == 1:
            self.next_task()
            
    def correct_IWO(self, task):
        """
        Update IWO
        """
        print(f"Current Estimate X: {iwo.X}")
        # Print roll pitch yaw from quaternion X[3:7]
        qw, qx, qy, qz = iwo.X[3:7]
        rpy = iwo._quaternion_to_euler(qw, qx, qy, qz)
        print(f"Current roll, pitch, yaw: {np.degrees(rpy)}")
        print(f"\n% [correct_IWO] Updating IWO with measurements: {task}")
        x = task["x"]
        y = task["y"]
        z = task.get("z",0)
        roll = task.get("roll", 0)
        pitch = task.get("pitch", 0)
        yaw = task.get("yaw", 0)

        roll, pitch, yaw = np.radians([roll,pitch,yaw]) # ADDED SO PASS IN AS DEGREES

        att_meas = iwo._normalize_attitude_measurement([roll, pitch, yaw])
        
        acc_meas = imu.acc

        iwo.correct(pos_meas=[x,y,z],acc_meas=acc_meas,att_meas=att_meas)
        print(f"Corrected Estimate: {iwo.X}")
        qw, qx, qy, qz = iwo.X[3:7]
        rpy = iwo._quaternion_to_euler(qw, qx, qy, qz)
        print(f"Updated roll, pitch, yaw: {np.degrees(rpy)}")

        self.next_task()
    
    def reset_IWO(self, task):
        """
        Reset IWO
        """
        print(f"Current Estimate X: {iwo.X}")
        # Print roll pitch yaw from quaternion X[3:7]
        qw, qx, qy, qz = iwo.X[3:7]
        rpy = iwo._quaternion_to_euler(qw, qx, qy, qz)
        print(f"Current roll, pitch, yaw: {np.degrees(rpy)}")
        print(f"\n% [reset_IWO] reseting IWO with: {task}")
        
        x = task.get("x",0)
        y = task.get("y",0)
        z = task.get("z",0)
        roll = task.get("roll", 0)
        pitch = task.get("pitch", 0)
        yaw = task.get("yaw", 0)

        roll, pitch, yaw = np.radians([roll,pitch,yaw])

        iwo.reset_kalman(x,y,z,roll,pitch,yaw)
        print(f"Reset Estimate: {iwo.X}")
        qw, qx, qy, qz = iwo.X[3:7]
        rpy = iwo._quaternion_to_euler(qw, qx, qy, qz)
        print(f"Reset roll, pitch, yaw: {np.degrees(rpy)}")

        self.next_task()

    def run_linefollow_until_x_intersections(self, task):

        """
        Follow a line on the chosen side until x intersections have been counted, line is lost,
        or timeout expires.
        """
        follow_left = (task["side"] == "left")
        speed = task.get("speed", 0.2)
        stop_speed = task.get("stop_speed",0)
        timeout = task.get("timeout", 30)
        intersections_to_pass = task.get("intersection_count",1)
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
            elif edge.crossingLine or self.task_time() > timeout:
                service.send("robobot/cmd/ti/",f"rc {stop_speed} 0.0") # (forward m/s, turn-rate rad/sec)
                self.task_state = 3

        elif self.task_state == 2: # following line
            if self.task_time() > timeout:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", f"rc {stop_speed:.3f} 0.0")
                print("% [line_follow] Timeout")
                self.task_state = 3
            elif edge.lineValidCnt < 2:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% [line_follow] Line lost at {pose.tripB:.3f}m")
                self.task_state = 3
            elif edge.crossingLine:
                # 
                if intersections_to_pass > 1:
                    intersections_to_pass -= 1
                    print(f"% [line_follow] Crossing line detected at {pose.tripB:.3f}m")
                    # wait a bit to avoid double-counting the same intersection
                    t.sleep(0.5)
                    print(f"% [line_follow] Intersection crossed! Remaining: {intersections_to_pass}")
                else:
                    edge.lineControl(0, True)
                    service.send("robobot/cmd/ti", f"rc {stop_speed:.3f} 0.0")
                    print(f"% [line_follow] Final crossing line detected at {pose.tripB:.3f}m")
                    self.task_state = 3
                    # elif : we arrived at destination

        elif self.task_state == 3: # stopping
            # if abs(pose.velocity()) < 0.001:
            print("% [line_follow] Stopped — next task")
            self.next_task()
    
    def run_drive_straight_crossing_x_lines(self, task):
        """
        Drive straight while counting line crossings until a certain number of lines have been crossed.
        """
        speed = task.get("speed", 0.2)
        lines_to_cross = task.get("lines_to_cross", 2)
        timeout = task.get("timeout", 30)

        if self.task_state == 0:
            print(f"% [drive_straight_crossing_x_lines] Starting — driving straight to cross {lines_to_cross} lines")
            service.send("robobot/cmd/ti", f"rc {speed:.3f} 0.0")
            pose.tripBreset()
            self._lines_crossed = 0
            self.task_state = 1

        elif self.task_state == 1:
            if edge.crossingLine:
                self._lines_crossed += 1
                print(f"% [drive_straight_crossing_x_lines] Line crossed! Total: {self._lines_crossed}")
                # wait a bit to avoid double-counting the same line
                # just sleep if crossing should be passed
                t.sleep(0.2)
                
            if self._lines_crossed >= lines_to_cross or pose.tripBtimePassed() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% [drive_straight_crossing_x_lines] Target lines crossed or timeout reached.")
                self.task_state = 2

        elif self.task_state == 2:
            if abs(pose.velocity()) < 0.001:
                print("% [drive_straight_crossing_x_lines] Stopped — next task")
                self.next_task()

    def run_line_follow_until_xy_IWO(self, task):
        """
        Follows a line until the robot's odometry reaches a specific global (X, Y) coordinate.
        """
        import sys
        
        edge.CUSTOM_CONTROL_ENABLED = True

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
            curr_x, curr_y = iwo.fused_x, iwo.fused_y
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

    def run_linefollow_until_ramp_IWO(self, task):
        """
        Follow a line until the robot's pitch goes different from the initial value and resets back to the initial value
        """
        side = task.get("side", "left")
        follow_left = (side == "left")
        speed = task.get("speed", 0.2)
        #pitch_threshold_deg = task.get("pitch_threshold_deg", 7)
        timeout = task.get("timeout", 30)
        edge.CUSTOM_CONTROL_ENABLED = True

        if self.task_state == 0:
            self.initial_pitch = iwo.fused_pitch
            print(f"\n% [line_follow_ramp_IWO] Starting — following line and monitoring pitch. Initial pitch: {self.initial_pitch:.1f}°")
            service.send("robobot/cmd/T0", "leds 16 0 100 0") # green
            #service.send("robobot/cmd/ti", f"rc {speed:.3f} 0.0")
            self.task_state = 1
        if self.task_state == 1:
            edge.lineControl(speed, follow_left)
            curr_pitch = iwo.fused_pitch
            pitch_diff = curr_pitch - self.initial_pitch
            print(f"\r% [line_follow_ramp_IWO] Current pitch: {curr_pitch:.1f}°, diff from initial: {pitch_diff:.1f}°   ", end="")
            if abs(pitch_diff) > 7: # we went up the ramp
                print("\n% [line_follow_ramp_IWO] Detected ramp ascent")
                
                # Make arm a bit stiff to reduce friction
                service.send("robobot/cmd/T0","servo 1 100 400") # (servo low)
                #service.send("robobot/cmd/T0","servo 1 125 400") # (servo low)

                self.task_state = 2
            elif self.task_time() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("\n% [line_follow_ramp_IWO] Timeout while monitoring for ramp")
                self.task_state = 3
        elif self.task_state == 2:
            edge.lineControl(speed, follow_left)
            curr_pitch = iwo.fused_pitch
            pitch_diff = curr_pitch - self.initial_pitch
            print(f"\r% [line_follow_ramp_IWO] Current pitch: {curr_pitch:.1f}°, diff from initial: {pitch_diff:.1f}°   ", end="")

            # if abs(iwo.fused_yaw) > 10:  # Check if yaw is significantly different
            #     print(f"\n% [line_follow_ramp_IWO] Warning: Significant yaw detected: {iwo.fused_yaw:.1f}°")
            #     # Correct yaw 
            #     if iwo.fused_yaw > 0:
            #         go_to_xy(iwo.fused_x, iwo.fused_y, -5, max_speed=0.3, dist_tolerance=0.05)
            #     else:
            #         go_to_xy(iwo.fused_x, iwo.fused_y, 5, max_speed=0.3, dist_tolerance=0.05)
            if iwo.fused_z > 0.45:
                # Relax arm
                service.send("robobot/cmd/T0","servo 1 10000 0") # relax servo
            if abs(pitch_diff) < 3: # off the ramp
                print("\n% [line_follow_ramp_IWO] Detected ramp finished — next task")
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"\n% [line_follow_ramp_IWO] Finished with final pitch: {curr_pitch:.1f}°")
                # print position iwo.fused_x, iwo.fused_y and z
                print(f"\n% [line_follow_ramp_IWO] Final position X: {iwo.fused_x:.2f}, Y: {iwo.fused_y:.2f}, Z: {iwo.fused_z:.2f}")
                # yaw
                print(f"\n% [line_follow_ramp_IWO] Final yaw: {iwo.fused_yaw:.1f}°")
                edge.lineControl(0, True)


                self.next_task()
            elif self.task_time() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("\n% [line_follow_ramp_IWO] Timeout while monitoring for ramp descent")
                edge.lineControl(0, True)
                self.next_task()

    
    def run_put_ball_in_hole(self, task):
        print("[PUT BALL IN HOLE] Putting Ball in Hole")
        
        service.send("robobot/cmd/T0","servo 1 100 400") # (servo low)
        # Logic similar to the get ball method: 
        # Hole coordinates
        hole_x = task["hole_x"]
        hole_y = task["hole_y"]
        # 1. Get global robot state
        cur_x = iwo.fused_x
        cur_y = iwo.fused_y
        cur_yaw = iwo.fused_yaw # in degrees
        # 2. Calculate approach target (0.25m away from global hole position)
        dx = hole_x - cur_x
        dy = hole_y - cur_y
        dist = np.hypot(dx, dy)
        approach_dist = 0.25
        # Only move if we aren't already within the approach distance
        if dist > approach_dist:
            scale = (dist - approach_dist) / dist
            target_x = cur_x + dx * scale
            target_y = cur_y + dy * scale
            print("[run_put_ball_in_hole]target_x = "+ str(target_x) + ("|| target_y = " + str(target_y)))
            print(cur_x, cur_y)
        else:            
            target_x, target_y = cur_x, cur_y
        # # 3. Drive to approach target
        # target_dist = target_x 
        # target_angle = np.arctan2(target_y, target_x)
        # turn_rate = max(-1.0, min(1.0, -target_angle * 3.0))
        # fwd_speed = max(0.05, min(0.2, target_dist * 0.3))
        # service.send("robobot/cmd/ti", f"rc {fwd_speed} {turn_rate}")

        go_to_xy(target_x, target_y, max_speed=0.4, dist_tolerance=0.01)
        print("[run_put_ball_in_hole] Position Reached")

        #perform a sweep forward and another sweep
        speed = 0.4
        duration = 1.5 # 1.5
        
        # While ball detected
        t.sleep(1)
        d1, d2 = get_ball_location()

        # If ball is not detected 
        if d1 is None:
            print("[PUT BALL IN] No ball detected, going to next task!")
            go_to_xy(iwo.fused_x, iwo.fused_y, 160, max_speed=0.4, dist_tolerance=0.01)
            self.next_task()
        
        sweep_count = 0
        yaw_reference = iwo.fused_yaw
        yaw_rotation = 20
        while d1 is not None and d2 is not None and sweep_count < 5: # do at least 5 sweeps to ensure we aren't just losing the ball due to noise
            sweep_count += 1
            # Sweep Left
            go_to_xy(iwo.fused_x, iwo.fused_y, yaw_reference+yaw_rotation, max_speed=0.6, dist_tolerance=0.01)
            #self.drive_to_xyyaw_IWO({"type": "drive_to_xyyaw_IWO", "target_yaw": yaw_reference+yaw_rotation, "just_yaw": True})
            #service.send("robobot/cmd/ti", f"rc 0.0 {speed:.3f}")
            print("yaw: " + str(iwo.fused_yaw))
            t.sleep(duration)

            # Sweep Right
            go_to_xy(iwo.fused_x, iwo.fused_y, yaw_reference-yaw_rotation, max_speed=0.6, dist_tolerance=0.01)
            #self.drive_to_xyyaw_IWO({"type": "drive_to_xyyaw_IWO", "target_yaw": yaw_reference-2*yaw_rotation, "just_yaw": True})
            #service.send("robobot/cmd/ti", f"rc 0.0 {-speed:.3f}")
            print("yaw: " + str(iwo.fused_yaw))
            t.sleep(duration)

            # Return to Center
            go_to_xy(iwo.fused_x, iwo.fused_y, yaw_reference, max_speed=0.6, dist_tolerance=0.01)
            #self.drive_to_xyyaw_IWO({"type": "drive_to_xyyaw_IWO", "target_yaw": yaw_reference+yaw_rotation, "just_yaw": True})
            #service.send("robobot/cmd/ti", f"rc 0.0 {speed:.3f}")
            print("yaw: " + str(iwo.fused_yaw))
            t.sleep(duration)
            service.send("robobot/cmd/ti", f"rc 0.0 0.0")
            t.sleep(1)
            
            d1, d2 = get_ball_location()
            if d1 is not None and d2 is not None:
                print("Ball still detected, repeating sweep")
            else:
                print("Ball no longer detected.")
                break

            # Go forward a little bit
            service.send("robobot/cmd/ti", f"rc 0.2 0.0")
            t.sleep(0.2)
            service.send("robobot/cmd/ti", f"rc 0.0 0.0")


        # Stop
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        t.sleep(0.1)

        # Move arm up
        service.send("robobot/cmd/T0","servo 1 -890 400") # Arm up
        t.sleep(0.5) # wait for servo to reach position
        # 4. Wait a moment and then relax servo to drop ball
        #t.sleep(1)
        #service.send("robobot/cmd/T0","servo 1 10000 0") # relax servo
        #t.sleep(1)
        self.next_task()

    def run_drive_until_line_detection(self, task):
        """
        Drive backwards until line is detected or timeout expires.
        """
        speed = task.get("speed", 0.2) # negative for backwards
        timeout = task.get("timeout", 30)

        if self.task_state == 0:
            print(f"% [drive_until_line_detection] Starting — driving until line is detected")
            service.send("robobot/cmd/ti", f"rc {speed:.3f} 0.0")
            self.task_state = 1

        elif self.task_state == 1:
            if edge.lineValidCnt > 4:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% [drive_until_line_detection] Line detected at {pose.tripB:.3f}m while driving.")
                self.task_state = 2
            elif pose.tripBtimePassed() > timeout:
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print(f"% [drive_until_line_detection] Timeout reached while driving.")
                self.task_state = 2

        elif self.task_state == 2:
            if abs(pose.velocity()) < 0.001:
                print("% [drive_until_line_detection] Stopped — next task")
                self.next_task()
        
    def run_servo_up(self, task):
        print("[Servo UP] Pulling Servo Up")
        service.send("robobot/cmd/T0","servo 1 -890 400") # (servo up)
        t.sleep(1.5) # wait for servo to reach position
        self.next_task()
    def run_servo_down(self, task):
        print("[Servo DOWN] Pushing Servo Down")
        service.send("robobot/cmd/T0","servo 1 150 400")
        t.sleep(1.5) # wait for servo to reach position
        self.next_task()
    def run_servo_mid(self, task):
        print("[Servo MID] Setting Servo to Mid Position")
        service.send("robobot/cmd/T0","servo 1 -200 400")
        t.sleep(1.5) # wait for servo to reach position
        self.next_task()

    def run_servo_relax(self, task):
        print("[Servo RELAX] Relaxing Servo")
        service.send("robobot/cmd/T0","servo 1 10000 0")
        self.next_task()

    def run_get_ball(self, task):
        
        # Move arm up
        service.send("robobot/cmd/T0", "servo 1 -890 400") # Arm up
        t.sleep(1) # wait for servo to reach position
        
        # 1. Get local body-frame coordinates
        d1, d2 = get_ball_location()
        if d1 is None or d2 is None:
            print("[GET BALL] Ball not detected, using default coordinates to search")
            go_to_xy(6.08, 2.76, -165, max_speed=0.2, dist_tolerance=0.01)
            service.send("robobot/cmd/T0", "servo 1 150 400") # Arm down
            # self.next_task()
            # return
        else:
            x_body = d2
            y_body = -d1
            
            # 2. Get global robot state
            cur_x = iwo.fused_x
            cur_y = iwo.fused_y
            cur_yaw = iwo.fused_yaw # in degrees

            # 3. Transform ball position to Global Frame
            # Rotation matrix: [cos -sin; sin cos]
            ball_global_x = cur_x + (x_body * np.cos(np.radians(cur_yaw)) - y_body * np.sin(np.radians(cur_yaw)))
            ball_global_y = cur_y + (x_body * np.sin(np.radians(cur_yaw)) + y_body * np.cos(np.radians(cur_yaw)))

            # 4. Calculate approach target (0.25m away from global ball position)
            dx = ball_global_x - cur_x
            dy = ball_global_y - cur_y
            dist = np.hypot(dx, dy)
            
            approach_dist = 0.25
            # Only move if we aren't already within the approach distance
            if dist > approach_dist:
                scale = (dist - approach_dist) / dist
                target_x = cur_x + dx * scale
                target_y = cur_y + dy * scale
            else:
                target_x, target_y = cur_x, cur_y

            # 5. Execution
            service.send("robobot/cmd/T0", "servo 1 -890 400") # Arm up
            t.sleep(1)

            # Print current position and heading and where ball was detected and where it is going
            print(f"Current Position: ({cur_x:.2f}, {cur_y:.2f}), Yaw: {cur_yaw:.1f}°")
            print(f"Ball detected at body-frame (x={x_body:.2f}, y={y_body:.2f}) -> global (x={ball_global_x:.2f}, y={ball_global_y:.2f})")
            print(f"Approach target: ({target_x:.2f}, {target_y:.2f})")
            
            go_to_xy(target_x, target_y, max_speed=0.6, dist_tolerance=0.01)
            
            service.send("robobot/cmd/T0", "servo 1 150 400") # Arm down
            t.sleep(1)
        print(f"Final Position: ({iwo.fused_x:.2f}, {iwo.fused_y:.2f}), Yaw: {iwo.fused_yaw:.1f}°")
        self.next_task()

    def run_luggage_catch(self, task):
        import aruco
        import numpy as np
        from sfuse import iwo as pose 
        
        target_id = int(task.get("target_box_id", 20))
        shuttle_id = 5  
        timeout = task.get("timeout", 45)
        
        
        client = aruco.get_client()
        client.set_enabled(True)
        print("LUGGAGE CATCH START")
        
        mission_state = "wait_for_conveyor_1"
        go_to_xy(iwo.fused_x, iwo.fused_y, 180)
        while mission_state == "wait_for_conveyor_1":
                data = client.get_data()
                markers = data["markers"] if (data and data.get("markers")) else []
                print("Waiting for conveyor 1 !")
                for m in markers:
                    if int(m["id"]) in [shuttle_id, target_id]:
                        print("% [Luggage] Conveyor passing")
                        t.sleep(3.5) 
                        
                        mission_state = "wait_for_conveyor"
                        service.send("robobot/cmd/T0", "servo 1 -370 400")
                        #mission_state = "wait_for_conveyor"
                        break 
        box_offset_x = 0.0
        start_time = t.time()
        print(start_time)
        shuttle_threshold = 0.0 
        
        while (t.time() - start_time) < timeout and not self.stop and not service.stop:
            data = client.get_data()
            markers = data["markers"] if (data and data.get("markers")) else []
                
            # wait for conveyor aruco before backing up 
            if mission_state == "wait_for_conveyor":
                for m in markers:
                    if int(m["id"]) in [shuttle_id, target_id] or t.time() > (start_time + 30): #and abs(m["x"] / 1000.0) < 0.1:
                        print("% [Luggage] Conveyor passing")
                        t.sleep(3.5) 
                        
                        print("% [Luggage] Backing up...")
                        service.send("robobot/cmd/ti", "rc -0.25 0.0") # back up 25cm
                        t.sleep(1.5)
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        service.send("robobot/cmd/T0", "servo 1 -890 400") # arm up
                        t.sleep(1.0) 
                        
                        mission_state = "detect"
                        break 
                        
            # calculate box 20 offset
            elif mission_state == "detect":
                for m in markers:
                    if int(m["id"]) == target_id:
                        box_offset_x = m["x"] / 1000.0 
                        print(f"% [Luggage] Trapped Box {target_id}! X-Offset: {box_offset_x:.2f}m")
                        
                        if abs(box_offset_x) > 0.05:
                            pose.tripBreset()
                            t.sleep(0.2) 
                            mission_state = "align_turn_1"
                        else:
                            print("% [Luggage] Box already centered! Moving to approach.")
                            mission_state = "final_approach"
                        break
                        
            # align sky to be directly in front of box 20 and perpendicular to conveyor
            elif mission_state == "align_turn_1":
                if box_offset_x > 0: # Box is Right
                    service.send("robobot/cmd/ti", "rc 0.0 -0.5")
                    if pose.tripBh <= -1.57: 
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        pose.tripBreset()
                        t.sleep(0.5) 
                        mission_state = "align_drive"
                else: # Box is Left
                    service.send("robobot/cmd/ti", "rc 0.0 0.5")
                    if pose.tripBh >= 1.57:  
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        pose.tripBreset()
                        t.sleep(0.5)
                        mission_state = "align_drive"
                        
            elif mission_state == "align_drive":
                service.send("robobot/cmd/ti", "rc 0.15 0.0") 
                if pose.tripB >= abs(box_offset_x):
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")
                    pose.tripBreset()
                    t.sleep(0.5)
                    mission_state = "align_turn_2"
                    
            elif mission_state == "align_turn_2":
                if box_offset_x > 0: # Turn Left to face forward
                    service.send("robobot/cmd/ti", "rc 0.0 0.5")
                    if pose.tripBh >= 1.57:
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        print("% [Luggage] Alignment complete! Initiating final approach.")
                        t.sleep(0.5)
                        mission_state = "final_approach"
                else: # Turn Right to face forward
                    service.send("robobot/cmd/ti", "rc 0.0 -0.5")
                    if pose.tripBh <= -1.57:
                        service.send("robobot/cmd/ti", "rc 0.0 0.0")
                        print("% [Luggage] Alignment complete! Initiating final approach.")
                        t.sleep(0.5)
                        mission_state = "final_approach"

            # approach and catch
            # need to add logic to wait for conveyor to be out of frame before lowering arm to trap box
            # also still need to figure out method to trap box 20 w/o trapping other cube under arm preventing it from lowering fully
            # potential solution would be using a different arm design that is skinnier but still has curved sides to guide boxes off conveyor
            elif mission_state == "final_approach":
                box_found = False
                for m in markers:
                    if int(m["id"]) == target_id:
                        box_found = True
                        target_dist = m["y"] / 1000.0 
                        target_angle = np.arctan2(m["x"], m["y"])
                        
                        if target_dist < 0.30: # need to fine tune this distance
                            service.send("robobot/cmd/ti", "rc 0.0 0.0")
                            service.send("robobot/cmd/T0", "servo 1 150 400") # lower arm
                            t.sleep(0.5)
                            service.send("robobot/cmd/ti", "rc 0.1 0.0")
                            t.sleep(0.05)
                            print("% [Luggage] Box caught!")
                            return True
                        else:
                            # move towards the box (need to fine tune as well)
                            turn_rate = max(-1.0, min(1.0, -target_angle * 3.0))
                            fwd_speed = max(0.05, min(0.2, target_dist * 0.3))
                            service.send("robobot/cmd/ti", f"rc {fwd_speed} {turn_rate}")
                        break
                        
                if not box_found:
                    # If we momentarily lose sight of the box, hit the brakes and wait
                    service.send("robobot/cmd/ti", "rc 0.0 0.0")

            t.sleep(0.05) 
            
        print("% [Luggage] Task timed out")
        service.send("robobot/cmd/ti", "rc 0 0")
        return False
    
    def run_stairs_down(self, task):
        """
        Go down stairs dynamically and robustly:
        1. Drive fast forward until inclined (pitch != 0).
        2. Keep traversing stairs (slow on descent, fast on flat).
        3. Tolerate slight sensor noise/vibrations when checking for flat ground.
        4. Stop stair phase ONLY when pitch remains near 0 for > 3 seconds.
        5. Follow line until intersection is detected.
        """
        follow_left = (task.get("side", "left") == "left")
        fast_speed = task.get("fast_speed", 0.5)
        slow_speed = task.get("slow_speed", 0.15)  # Reduced for gentler turns
        pitch_threshold = task.get("pitch_threshold", 5)  # degrees
        timeout = task.get("timeout", 60)
        
        edge.CUSTOM_CONTROL_ENABLED = True
        
        # Initialize tracking variables on first run
        if self.task_state == 0:
            print("% [stairs_down] Starting — driving fast forward to find stairs")
            service.send("robobot/cmd/T0", "leds 16 100 0 0")  # red
            service.send("robobot/cmd/ti", f"rc {fast_speed:.3f} 0.0")
            self._stairs_completed = 0
            self._on_descent = False
            self._waiting_for_horizontal = False
            self._initial_pitch = iwo.fused_pitch
            self._horizontal_start_time = None  # Timer for detecting the end of the stairs
            pose.tripBreset()
            self.task_state = 1
        
        # State 1: Search for first stair (detect pitch change)
        elif self.task_state == 1:
            if self.task_time() > timeout:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("% [stairs_down] Timeout searching for stairs")
                self.task_state = 10
            else:
                curr_pitch = iwo.fused_pitch
                pitch_diff = abs(curr_pitch - self._initial_pitch)
                
                # Detected incline - first stair found
                if pitch_diff > pitch_threshold:
                    print(f"% [stairs_down] First stair detected (pitch change: {pitch_diff:.3f} deg)")
                    edge.lineControl(slow_speed, follow_left, refPosition=2.0)  # Gentler control
                    pose.tripBreset()
                    self._stairs_completed = 0
                    self._on_descent = True
                    self._waiting_for_horizontal = False
                    self.task_state = 2
        
        # State 2: Dynamic loop for stairs with anti-vibration logic
        elif self.task_state == 2:
            curr_pitch = iwo.fused_pitch
            
            # Instead of comparing to the start, check if absolute pitch is near zero
            # Adding a 50% tolerance (1.5 multiplier) to account for vibrations on flat ground
            is_flat = abs(curr_pitch) < (pitch_threshold * 1.5) 
            is_steep = abs(curr_pitch) > pitch_threshold
            
            dist_traveled = pose.tripB
            
            # --- CASE 1: The robot is clearly inclined (Descent) ---
            if not is_flat:
                # If the timer was running, warn that it was canceled by a bump/slope
                if self._horizontal_start_time is not None:
                    print(f"% [stairs_down] Timer canceled! Current pitch: {curr_pitch:.3f} deg")
                    self._horizontal_start_time = None
                
                if not self._on_descent and is_steep:
                    # Start of a new stair descent
                    print(f"% [stairs_down] Starting descent for stair {self._stairs_completed + 1}")
                    edge.lineControl(slow_speed, follow_left, refPosition=2.0)
                    pose.tripBreset()
                    self._on_descent = True
                    self._waiting_for_horizontal = False
                
                # Continue descending slowly
                edge.lineControl(slow_speed, follow_left, refPosition=2.0)
            
            # --- CASE 2: The robot is horizontal (Flat) ---
            else:
                # If it just finished a descent
                if self._on_descent and not self._waiting_for_horizontal:
                    print(f"% [stairs_down] Stair {self._stairs_completed + 1} completed (traveled {dist_traveled:.3f}m)")
                    self._stairs_completed += 1
                    self._on_descent = False
                    self._waiting_for_horizontal = True
                    pose.tripBreset()
                
                # Start or maintain the flat terrain timer
                if self._horizontal_start_time is None:
                    self._horizontal_start_time = self.task_time()
                    print(f"% [stairs_down] Horizontal detected (Pitch: {curr_pitch:.3f}), starting 3s timer...")
                
                time_flat = self.task_time() - self._horizontal_start_time
                
                # Optional debug: Print every second to see if it's progressing
                if int(time_flat * 10) % 10 == 0 and time_flat > 0: 
                    print(f"% [stairs_down] Stable for {time_flat:.1f}s ...")
                
                # If 3 seconds have passed without returning to a slope
                if time_flat >= 3.0:
                    print(f"% [stairs_down] End of stairs confirmed (stable for 3s). Total: {self._stairs_completed} stairs.")
                    pose.tripBreset()
                    self.task_state = 3
                else:
                    # While waiting for the 3 seconds, keep moving forward on the flat ground
                    edge.lineControl(fast_speed, follow_left, refPosition=0.0)
        
        # State 3: Follow line until intersection
        elif self.task_state == 3:
            if self.task_time() > timeout:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("% [stairs_down] Timeout following line to intersection")
                self.task_state = 4
            elif edge.crossingLine:
                # Intersection detected - stop immediately
                print(f"% [stairs_down] Crossing detected at {pose.tripB:.3f}m - stopping")
                edge.lineControl(0, True)  # Disable line control
                # Send multiple stop commands to ensure robot stops
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                t.sleep(0.05)  # Brief pause
                service.send("robobot/cmd/ti", "rc 0.0 0.0")  # Ensure stop
                print(f"% [stairs_down] Intersection stopped")
                self.task_state = 4
            elif edge.lineValidCnt < 2:
                # Line lost
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                print("% [stairs_down] Line lost before intersection")
                self.task_state = 4
            else:
                # Keep following line with gentle control and slower speed
                edge.lineControl(0.1, follow_left, refPosition=0.5)  # Slower, gentler control
        
        # State 4: Stopping
        elif self.task_state == 4:
            if abs(pose.velocity()) < 0.001:
                print("% [stairs_down] Stopped — next task")
                edge.lineControl(0, True)
                # Ensure robot is completely stopped
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                t.sleep(0.1)  # Extra pause to ensure stop
                self.next_task()

    def run_get_ball_sorting(self, task):
        import numpy as np
        from sfuse import iwo
        
        # Extract the target color from the JSON (defaults to red to be safe)
        target_color = task.get("color", "red")
        print(f"[%] Initiating search and capture for {target_color.upper()} ball...")

        # Move arm up
        service.send("robobot/cmd/T0", "servo 1 -890 400") 
        t.sleep(1) 
        
        # 1. Get local body-frame coordinates
        d1, d2 = get_ball_location_sorting(color=target_color) 
        
        if d1 is None or d2 is None:
            print(f"[GET BALL] {target_color.upper()} ball not detected initially. Skipping.")
            service.send("robobot/cmd/T0", "servo 1 150 400")
            self.task_index += 1
            return False
            
        x_body, y_body = d2, -d1
        cur_x, cur_y, cur_yaw = iwo.fused_x, iwo.fused_y, iwo.fused_yaw

        ball_global_x = cur_x + (x_body * np.cos(np.radians(cur_yaw)) - y_body * np.sin(np.radians(cur_yaw)))
        ball_global_y = cur_y + (x_body * np.sin(np.radians(cur_yaw)) + y_body * np.cos(np.radians(cur_yaw)))

        dist_to_ball = np.hypot(ball_global_x - cur_x, ball_global_y - cur_y)
        
        if dist_to_ball > 0.60:
            print(f"[%] Ball is far ({dist_to_ball:.2f}m). Executing intermediate approach...")

            service.send("robobot/cmd/T0", "servo 1 150 400")
            
            # Calculate a waypoint exactly 0.5m away from the initially detected ball
            scale = (dist_to_ball - 0.50) / dist_to_ball
            mid_x = cur_x + (ball_global_x - cur_x) * scale
            mid_y = cur_y + (ball_global_y - cur_y) * scale
            
            go_to_xy(mid_x, mid_y, max_speed=0.6, dist_tolerance=0.01)

            service.send("robobot/cmd/T0", "servo 1 -890 400")
            
            print("[%] Midpoint reached. Pausing 1.0s to clear camera blur...")
            t.sleep(1.0)
            
            # Take a second, highly accurate picture
            d1, d2 = get_ball_location_sorting(color=target_color)
            if d1 is None or d2 is None:
                print(f"[GET BALL] Lost sight of {target_color.upper()} ball at midpoint! Aborting.")
                service.send("robobot/cmd/T0", "servo 1 150 400")
                self.task_index += 1
                return False
                
            # Recalculate global position with the clean, close-range data
            x_body, y_body = d2, -d1
            cur_x, cur_y, cur_yaw = iwo.fused_x, iwo.fused_y, iwo.fused_yaw
            
            ball_global_x = cur_x + (x_body * np.cos(np.radians(cur_yaw)) - y_body * np.sin(np.radians(cur_yaw)))
            ball_global_y = cur_y + (x_body * np.sin(np.radians(cur_yaw)) + y_body * np.cos(np.radians(cur_yaw)))
            print(f"[%] Secondary Lock Acquired! Corrected Global: ({ball_global_x:.2f}, {ball_global_y:.2f})")

        # Re-poll current position in case we drifted or executed Phase 2
        cur_x, cur_y = iwo.fused_x, iwo.fused_y
        dist_to_ball = np.hypot(ball_global_x - cur_x, ball_global_y - cur_y)
        
        approach_dist = 0.25
        if dist_to_ball > approach_dist:
            scale = (dist_to_ball - approach_dist) / dist_to_ball
            target_x = cur_x + (ball_global_x - cur_x) * scale
            target_y = cur_y + (ball_global_y - cur_y) * scale
        else:
            target_x, target_y = cur_x, cur_y

        print(f"[%] Final approach to: ({target_x:.2f}, {target_y:.2f})")
        go_to_xy(target_x, target_y, max_speed=0.6, dist_tolerance=0.01)
        
        service.send("robobot/cmd/T0", "servo 1 150 400") 
        t.sleep(1)
        
        self.task_index += 1
        return True

   
            
