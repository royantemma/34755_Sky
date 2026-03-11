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
        self.task_state = 0   # 0 not started, 1 running, 2 completed
        self.task_start = None

    # time helpers

    def time_elapsed(self):
        return (datetime.now() - self.mission_start).total_seconds()

    def time_left(self):
        return self.total_time - self.time_elapsed()

    def task_time(self):
        return (datetime.now() - self.task_start).total_seconds()

    def should_go_to_goal(self):
        return self.time_left() < self.goal_time_buffer

    # task transitions

    def start_task(self, index):
        self.task_index = index
        self.task_state = 0
        self.task_start = datetime.now()
        pose.tripBreset() # use trip counter/timer B
        task = self.tasks[index]
        print(f"% [MissionRunner] Starting task {index}: {task['type']}")

    def next_task(self):
        self.start_task(self.task_index + 1)

    # main loop

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

            elif task_type == "drive_straight":
                self.run_drive_straight(current_task)

            elif task_type == "turn":
                self.run_turn(current_task)

            else:
                print(f"% [MissionRunner] Unknown task type '{task_type}' — skipping")
                self.next_task()

            t.sleep(0.02)

        # clean up
        edge.lineControl(0, True)
        service.send("robobot/cmd/ti", "rc 0.0 0.0")
        service.send("robobot/cmd/T0", "leds 16 0 0 0")
        print("% [MissionRunner] Finished")


    def run_line_follow(self, task):
        """
        Follow a line on the chosen side until distance is reached, line is lost,
        or timeout expires.
        """
        follow_left = (task["side"] == "left")
        speed = task.get("speed",    0.2)
        timeout = task.get("timeout",  3)

        pose.tripBreset()
        dist_to_line = 0;
        print("% Driving to line ---------------------- right ir start ---")
        service.send("robobot/cmd/T0", "leds 16 0 100 0") # green
        
        print(self.task_state)

        if self.task_state == 0: # forward towards line
            service.send("robobot/cmd/ti","rc 0.2 0.0") # (forward m/s, turn-rate rad/sec)
            self.task_state = 1

        elif self.task_state == 1:
            if pose.tripB > 1.0 or self.task_time() > timeout:
                service.send("robobot/cmd/ti/","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
                self.task_state = 2
            if edge.lineValidCnt > 4:
                # start follow line
                edge.lineControl(speed, follow_left)
                dist_to_line = pose.tripB
                pose.tripBreset()
                print(" to state 10")
                self.task_state = 10
            pass

        elif self.task_state == 2:
            if abs(pose.velocity()) < 0.001:
                print("next task")
                self.next_task()

        elif self.task_state == 10:
            if pose.tripBtimePassed() > timeout:
                service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
                print(" to state 2")
                pose.tripBreset()
                self.task_state = 2

            if edge.lineValidCnt < 2:
                edge.lineControl(speed, follow_left)
                service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
                print(" to state 2")
                pose.tripBreset()
                self.task_state = 2
    
        service.send("robobot/cmd/T0","leds 16 0 0 0") # end
        print("% Driving to line ------------------------- end")

    

    ####################################################################3

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

    ####################################################################3

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

        pose.tripBreset()
        print("% Driving a Pi turn -------------------------")
        service.send("robobot/cmd/T0","leds 16 0 100 0") # green
        
        if self.task_state == 0: # wait for start signal
            service.send("robobot/cmd/ti","rc turn_rate 0.5") # (forward m/s, turn-rate rad/sec)
            self.task_state = 1

        elif self.task_state == 1:
            if pose.tripBh > 3.14 or pose.tripBtimePassed() > 15:
                service.send("robobot/cmd/ti","rc 0.0 0.0") # (forward m/s, turn-rate rad/sec)
                self.task_state = 2
            pass
        
        elif self.task_state == 2:
            if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
                self.task_state = 99

        pass
        service.send("robobot/cmd/T0","leds 16 0 0 0") #end
        print("% Driving a Pi turn ------------------------- end")

                self.next_task()

    ####################################################################3

    def run_go_to_goal(self, task):
        """
        Emergency / final go-to-goal behaviour.
        Follows the line on the right edge back to the goal.
        Customise side/distance/speed to match your actual map.

        Task keys:
            side        'left' or 'right' (default 'right')
            speed       m/s (default 0.2)
            distance    metres to follow (default 2.0)
            timeout     seconds (default 40)
        """
        follow_left = (task.get("side", "right") == "left")
        speed    = task.get("speed",    0.2)
        distance = task.get("distance", 2.0)
        timeout  = task.get("timeout",  40)

        if self.task_state == 0:
            print("% [go_to_goal] Heading to goal")
            service.send("robobot/cmd/T0", "leds 16 100 0 0")   # red = going home
            # wait for line before starting
            if edge.lineValidCnt > 4:
                edge.lineControl(speed, follow_left)
                self.task_state = 1
            elif self.task_time() > 2.0:
                # no line found — just drive straight as fallback
                service.send("robobot/cmd/ti", f"rc {speed:.3f} 0.0")
                self.task_state = 1

        elif self.task_state == 1:
            if pose.tripB >= distance or self.task_time() > timeout:
                edge.lineControl(0, True)
                service.send("robobot/cmd/ti", "rc 0.0 0.0")
                self.task_state = 2

        elif self.task_state == 2:
            if abs(pose.velocity()) < 0.005:
                self.task_state = 99   # signals completion to run()