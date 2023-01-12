from Niveau3 import Blinker
from Application import LedBlinkers, Robot, C64

def factory_state_on():
    state_1 = LedBlinkers.OnLedBlinkersState()
    return state_1

def factory_state_off():
    state_1 = LedBlinkers.OffLedBlinkersState()
    return state_1

def main():
    r = Robot()
    r.set_eyes_color((255,255,255), "right")

if __name__ == '__main__':
    quit(main())
