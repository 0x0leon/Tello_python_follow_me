from djitellopy import Tello
import time


drone = Tello()


time.sleep(1.0) #waiting 2 seconds
print("Connecting...")
drone.connect()
print("akku: ")
print(drone.get_battery())
drone.end()
