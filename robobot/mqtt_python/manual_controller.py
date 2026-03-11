import sys
import select
import termios
import tty
import time
from uservice import service
from spose import pose

move_speed = 0.15  # m/s
turn_speed = 0.35   # rad/s
timeout = 0.1      # Seconds without input before assuming key release

def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''  
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def controller():
    settings = termios.tcgetattr(sys.stdin)
    
    print("\n--- Robot Manual Controller ---")
    print("Hold W/S : Move Forward/Backward")
    print("Hold A/D : Turn Left/Right")
    print("Release movement keys to stop immediately.")
    print("-----------------------------------------")
    print("Press U  : Servo Up")
    print("Press J  : Servo Down")
    print("Press K  : Servo Zero")
    print("-----------------------------------------")
    print("Press M  : Map Marker")
    print("-----------------------------------------")
    print("Press Q to Quit\n")

    current_vel = 0.0
    current_turn = 0.0

    # Ensure the LEDs show we are in manual drive mode (Blue)
    service.send("robobot/cmd/T0", "leds 16 0 0 30")

    # Clear/create the waypoint file at the start of the run
    with open("map_waypoints.txt", "w") as f:
        f.write("Label,X,Y,Heading\n")

    try:
        target_vel = 0.0
        target_turn = 0.0
        while not service.stop:
            key = get_key(settings, timeout)
            
            
            # --- Drive Controls ---
            if key.lower() == 'w':
                target_vel += move_speed
            elif key.lower() == 's':
                target_vel -= move_speed
            elif key.lower() == 'a':
                target_turn += turn_speed
            elif key.lower() == 'd':
                target_turn -= turn_speed
            elif key.lower() == 'x':
                target_vel, target_turn = 0.0, 0.0
            elif key.lower() == 'q':
                print("\nExiting controller...")
                service.stop = True 
                break
                
            # --- Custom Mapping Controls ---
            elif key.lower() == 'm':
                # 1. Stop the robot immediately so it doesn't drift
                service.send("robobot/cmd/ti", "rc 0.000 0.000")
                current_vel, current_turn = 0.0, 0.0
                target_vel, target_turn = 0.0, 0.0
                
                # 2. Restore normal terminal settings so you can type a full word
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                
                # 3. Get the exact coordinates
                x, y, h = pose.pose[0], pose.pose[1], pose.pose[2]
                
                # 4. Ask the user for a custom label
                print(f"\n\nPAUSED Current Position - X:{x:.3f}, Y:{y:.3f}")
                label = input("Add name for map market and press Enter: ")
                
                # 5. Append directly to the text file
                with open("map_waypoints.txt", "a") as f:
                    f.write(f"{label},{x:.4f},{y:.4f},{h:.4f}\n")
                
                print(f"--> Saved '{label}'! Press any drive key (W/A/S/D) to resume driving.\n")
                
                # 6. The loop continues, and get_key() will automatically put the terminal back in raw mode.
                continue 
            
            # --- Servo Controls ---
            elif key.lower() == 'u':
                service.send("robobot/cmd/T0", "servo 1 -800 300")
            elif key.lower() == 'j':
                service.send("robobot/cmd/T0", "servo 1 500 200")
            elif key.lower() == 'k':
                service.send("robobot/cmd/T0", "servo 1 0 0")

            # Publish the drive command if it changed or is non-zero
            if (target_vel != 0.0 or target_turn != 0.0) or (current_vel != 0.0 or current_turn != 0.0):
                current_vel = target_vel
                current_turn = target_turn
                
                service.send("robobot/cmd/ti", f"rc {current_vel:.3f} {current_turn:.3f}")
                
                sys.stdout.write(f"\rCommand: vel={current_vel:.2f}, turn={current_turn:.2f}      ")
                sys.stdout.flush()

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        service.send("robobot/cmd/ti", "rc 0.000 0.000")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n Done :)")