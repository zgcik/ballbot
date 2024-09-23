import numpy as np
from bot import Bot

class Operate:
    def __init__(self, map):
        self.bot = Bot()
        self.map = map
        self.ball_num = 0

    def explore(self):
        ang_rotate = 0.0
        target = self.bot.cam.detect_closest()

        while target == None:
            print('no balls detected: exploring...')
            self.bot.rotate(np.pi/4)
            ang_rotate += np.pi/4
            
            if ang_rotate == 2*np.pi:
                self.bot.rotate(np.pi/4)
                self.bot.drive(1.0)
                ang_rotate = 0.0
            
            # re-detecting
            target = self.bot.cam.detect_closest()

    def collection(self):
        while self.ball_num < 5:
            # driving to closest target
            ret = self.bot.drive_to_target()

            if ret is None: 
                self.explore()
                continue

            # iterating number of balls
            self.ball_num += 1
            print(f'ball #{self.ball_num}: arrived.')

            # collecting the ball
            self.bot.flip()
            print(f'ball #{self.ball_num}: collected.')

        self.dispose()

    def dispose(self):
        # driving to relative location of box ( change to location of box )
        box_point = self.map[1]
        self.bot.drive_to_box(box_point)

        # TODO: complete box disposal

        
if __name__ == "__main__":
    map = [[0.0, 0.0], [6.4, 0.0], [6.4, -4.1], [0.0, -4.1]]
    operate = Operate(map)
    operate.collection()