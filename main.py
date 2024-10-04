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
            ret = self.bot.drive_to_target()

            if not ret: 
                self.explore()
                continue
            # collecting the ball
            else:
                # iterating number of balls
                
                self.ball_num += 1
                print(f'ball #{self.ball_num}: arrived.')
                self.bot.collect()
                print(f'ball #{self.ball_num}: collected.')
                self.ang_rotate = 0

        self.dispose()

    def dispose(self):
        # driving to relative location of box ( change to location of box )
        self.bot.drive_to_box(self.box_c)

        # self.bot.drive_to_location([0,0])

        self.bot.storage()

        
if __name__ == "__main__":
    
    try:
        map = [[0.0, 0.0], [-6.4, 0.0], [-6.4, 4.1], [0.0, 4.1]]
        box_c = [6.4, 4.1]
        operate = Operate(map, box_c)
        operate.collection()

        # operate.bot.rotate(0.6)
        # operate.bot.rotate(-0.6)
        # operate.bot.storage()


    except KeyboardInterrupt:
        print("\nScript interrupted by Ctrl+C")
    finally:
        # Ensure the camera is properly closed when interrupted
        operate.bot.cam.cam.stop()
        print("Camera stopped and cleaned up.")