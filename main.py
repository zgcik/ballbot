import numpy as np
from bot import Bot

balls_collected = 0

def explore(bot):

    target = None
    ang_rotate = 0.0

    while target == None:
        try:
            target = bot.cam.get_closest()
            d, t = target
        except:
            print('no balls detected: exploring...')
            bot.rotate(np.pi/4)
            ang_rotate += np.pi/4

            if ang_rotate == 2*np.pi:
                bot.rotate(np.pi/4)
                bot.drive(1.0)
                ang_rotate = 0.0
    return True

def collection(bot):
    balls_collected = 0

    while True:
        try:
            d, t = bot.cam.get_closest()
            bot.rotate(t)
            bot.drive(d/2)

            d, t = bot.cam.get_closest()
            bot.rotate(t)
            bot.drive(d/2)

            # TODO: insert ultrasonic module confirmation

            balls_collected += 1
            print(f'arrived at ball {balls_collected}')
            
        except:
            print('no balls detected: exploring...')
            explore(bot)
        
        if balls_collected == 5:
            dispose(bot)
            break

def dispose(bot):
    ang_rotate = 0.0
    while bot.cam.detect_box() is None:
        bot.rotate(np.pi/4)
        ang_rotate += np.pi/4

        if ang_rotate == 2*np.pi:
            bot.rotate(np.pi/4)
            bot.drive(1.0)
            ang_rotate = 0.0
        
    d, t = bot.cam.detect_box()
    bot.rotate(t)
    bot.drive(d)

    return True
        
if __name__ == "__main__":
    bot = Bot()
    explore(bot)
    collection(bot)