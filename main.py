import numpy as np
from bot import Bot
import time

class Operate:
    def __init__(self, map, box_c):
        self.bot = Bot()
        self.map = map
        self.box_c = box_c
        self.ball_num = 0
        self.ang_rotate = 0

    def explore(self):
        ang_rotate = 0.0
        target = self.bot.cam.detect_closest()

        while target == None:
            print('no balls detected: exploring...')
            self.bot.rotate(np.pi/16)
            self.ang_rotate += np.pi/16
            if self.ang_rotate == 2*np.pi:
                self.bot.rotate(np.pi/4)
                self.bot.drive(3.0)
                self.ang_rotate = 0.0
            time.sleep(0.25)
            # re-detecting
            target = self.bot.cam.detect_closest()

    def collection(self):
        while self.ball_num < 5:#5:
            # driving to closest target
            if self.bot.drive_to_target():
                # iterating number of balls
                self.ball_num += 1
                print(f'ball #{self.ball_num}: arrived.')
                self.bot.collect(80)
                print(f'ball #{self.ball_num}: collected.')
                self.ang_rotate = 0
            else:
                self.explore()
                continue



        self.dispose()

    def dispose(self):
        # driving to relative location of box
        # self.bot.drive_to_box(self.box_c)
        # d = 10
        # while d>10:
        #     d, t = self.bot.get_box()
        while True:
            if not self.bot.drive_to_box_vis():
                self.bot.rotate(0.15)
            else:
                break
        # # rotating storage to box
        # self.bot.rotate(self.clamp_angle(np.pi - self.bot.state[2]))

        # releasing storage
        self.bot.storage()
    
    @staticmethod
    def clamp_angle(ang):
        ang = ang % (2 * np.pi)
        if ang > np.pi: ang -= 2*np.pi
        return ang

        
if __name__ == "__main__":
    map = [[0.0, 0.0], [5.5, 0.0], [5.5, 4.11], [0.0, 4.11]]
    box_c = [4.11,5.5]
    #b = [1,0]
    operate = Operate(map, box_c)
    try:
        operate.dispose()
        # operate.collection()


        # operate.bot.rotate(0.6)
        # operate.bot.rotate(-0.6)
        # operate.bot.storage()


    except KeyboardInterrupt:
        print("\nScript interrupted by Ctrl+C")
    finally:
        # Ensure the camera is properly closed when interrupted
        operate.bot.cam.cam.stop()
        print("Camera stopped and cleaned up.")
