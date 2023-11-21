
import cv2
from djitellopy import Tello



tello = Tello()
tello.connect()
tello.takeoff()



while True:


    state = tello.get_current_state()


    print(state)
    print(" ")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        tello.land()
        break