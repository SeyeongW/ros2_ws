from camera import SiyiGimbal
import time
import sys
import tty
import termios

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main():
    gimbal = SiyiGimbal()

    print(">>> Sending gimbal to LOCK MODE...")
    gimbal.set_lock_mode()

    print("Press: a/d = Yaw Left/Right,  w/s = Pitch Up/Down, q = Quit")

    while True:
        key = getch()

        yaw = 0
        pitch = 0

        if key == "a":
            yaw = -20
        elif key == "d":
            yaw = 20
        elif key == "w":
            pitch = -20
        elif key == "s":
            pitch = 20
        elif key == "q":
            print("Quit.")
            break

        print(f"Yaw: {yaw}, Pitch: {pitch}")
        gimbal.send_speed(yaw, pitch)

        time.sleep(0.1)

if __name__ == "__main__":
    main()