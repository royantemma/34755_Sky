import time as t
from spose import pose
from uservice import service

def calibrate_straight():
    state = 0
    target_distance = 2.0
    pose.tripBreset()
    service.send("robobot/cmd/T0","leds 16 0 100 0") # Green

    while not (service.stop):
        if state == 0:
            service.send("robobot/cmd/ti","rc 0.15 0.0")
            state = 1
        elif state == 1:
            if pose.tripB >= target_distance:
                service.send("robobot/cmd/ti","rc 0.0 0.0")
                print(f"\nOdom:{pose.tripB:.3f}m")
                state = 99
        elif state == 99:
            break
        
        t.sleep(0.05)
        
    service.send("robobot/cmd/T0","leds 16 0 0 0") # end
    print("% Done")

def calibrate_rotation():
    state = 0
    target_rotations = 5
    target_radians = target_rotations * 2 * 3.14159
    pose.tripBreset()
    service.send("robobot/cmd/T0","leds 16 0 100 0") # Green

    while not (service.stop):
        if state == 0:
            print(f"# Spinning in place at 1.0 rad/s")
            service.send("robobot/cmd/ti","rc 0.0 1.0")
            state = 1
        elif state == 1:
            if abs(pose.tripBh) >= target_radians:
                service.send("robobot/cmd/ti","rc 0.0 0.0")
                print(f"\nOdom: {abs(pose.tripBh):.3f} radians")
                state = 99
        elif state == 99:
            break
        
    service.send("robobot/cmd/T0","leds 16 0 0 0") # end
    print("% Done")