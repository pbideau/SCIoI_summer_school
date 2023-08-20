from buildhat import Motor
from time import sleep
from sshkeyboard import listen_keyboard

DIRECTIONS = {
    'up': (-100, 100),
    'down': (100, -100),
    'right': (-100, -100),
    'left': (100, 100),
}

class LegoRobot():
    def __init__(self):
        self.motor_left = Motor('C')
        self.motor_right = Motor('D')

    def stop(self, key):
        self.motor_left.stop()
        self.motor_right.stop()

    def move(self, direction):
        if direction in DIRECTIONS:
            speed_left, speed_right = DIRECTIONS[direction]
            self.motor_left.start(speed_left)
            self.motor_right.start(speed_right)

if __name__=="__main__":
    robot = LegoRobot()
    print("Start up!")
    while True:
        print("Command loop...")
        def press(key):
            print(f"{key} pressed")
            
            if key == 'up':
                robot.move('up')
            elif key == 'down':
                robot.move('down')
            elif key == 'left':
                robot.move('left')
            elif key == 'right':
                robot.move('right')
            if key == 'q':
                robot.stop()
                sys.exit(0)
        listen_keyboard(on_press=press, on_release=robot.stop, sequential=False, delay_second_char = 0.1, sleep = 0.1, delay_other_chars=0.001)

