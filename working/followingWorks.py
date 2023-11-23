import cv2
import numpy as np
import time
from djitellopy import Tello
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# drone positions
x_positions = []
y_positions = []
z_positions = []

#set points (center of the frame coordinates in pixels)
rifX = 960/2
rifY = 720/2

#PI constant
Kp_X = 0.4
Ki_X = 0.0
Kp_Y = 0.2
Ki_Y = 0.0

Kp_Z = 0.1
Ki_Z = 0.0

# Loop time
Tc = 0.05

# PI terms initialized
integral_X = 0
integral_Y = 0

error_X = 0
error_Y = 0

previous_error_X = 0
previous_error_Y = 0

integral_Z = 0
error_Z = 0
previous_error_Z = 0

pre_center_x = rifX
pre_center_y = rifY

#neural network
net = cv2.dnn.readNetFromCaffe("working/MobileNetSSD_deploy.prototxt.txt", "working/MobileNetSSD_deploy.caffemodel") #modify with the NN path
CLASSES = [	
			"background", "aeroplane", 
			"bicycle", "bird", "boat",
			"bottle", "bus", "car", 
			"cat", "chair", "cow", 
			"diningtable","dog", "horse", 
			"motorbike", "person", "pottedplant", 
			"sheep","sofa", "train", "tvmonitor"
		]


colors = np.random.uniform(0, 255, size=(len(CLASSES), 3))


drone = Tello()  # declaring drone object
time.sleep(1.0) #waiting 2 seconds
print("Connecting...")
drone.connect()
print("BATTERY: ")
print(drone.get_battery())
time.sleep(1.0)
print("Loading...")
drone.streamon()  # start camera streaming
print("stream started")
time.sleep(0.5)
print("Takeoff...")
drone.takeoff() # drone takeoff


while True:
	start = time.time()
	frame = drone.get_frame_read().frame

	cv2.circle(frame, (int(rifX), int(rifY)), 1, (0,0,255), 10)

	h,w,channels = frame.shape

	blob = cv2.dnn.blobFromImage(frame,
		0.007843, (180, 180), (0,0,0),True, crop=False)

	net.setInput(blob)
	detections = net.forward()

	# loop over detections
	for i in np.arange(0, detections.shape[2]):


		idx = int(detections[0, 0, i, 1])

		# probability
		confidence = detections[0, 0, i, 2]

		# detect person, check if class is over 50 percent person
		if CLASSES[idx] == "person" and confidence > 0.5:

			# create box
			box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
			(startX, startY, endX, endY) = box.astype("int")

			# label with probabilty
			label = "{}: {:.2f}%".format(CLASSES[idx],
				confidence * 100)
			
			# rectangle around person
			cv2.rectangle(frame, (startX, startY), (endX, endY),
				colors[idx], 2)
			

			#draw the center of the person detected
			center_x = (startX + endX) / 2
			center_y = (2 * startY + endY) / 3

			pre_center_x = center_x
			pre_center_y = center_y

			cv2.circle(frame, (int(center_x), int(center_y)), 1, (0,0,255), 10)

			error_X = -(rifX - center_x)
			error_Y = rifY - center_y


			cv2.line(frame, (int(rifX),int(rifY)), (int(center_x),int(center_y)), (0,255,255),5 )
			cv2.circle(frame, (int(center_x),int(center_y)), radius=0, color=(0, 0, 255), thickness=-1)

			y = startY - 15 if startY - 15 > 15 else startY + 15
			cv2.putText(frame, label, (startX, y),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[idx], 2)
			
			# PI controller for controlling drone movement to desired position
			integral_X = integral_X + error_X*Tc 	# updating integral PID term
			integral_Y = integral_Y + error_Y*Tc 	# updating integral PID term

			uX = Kp_X*error_X + Ki_X*integral_X 	# updating control variable uX
			uY = Kp_Y*error_Y + Ki_Y*integral_Y		# updating control variable uY

			previous_error_X = error_X 				# update previous error variable
			previous_error_Y = error_Y				# update previous error Variable
			

			speed = 50

			if endX - startX < rifX - 200:
				# need to add first and second parameter too
				drone.send_rc_control(0,speed,round(uY),round(uX))
				#break when a person is recognized
			elif endX - startX < rifX - 100: 
				drone.send_rc_control(0,round(speed / 2),round(uY),round(uX))
			elif endX - startX >= rifX:
				drone.send_rc_control(0,-speed,round(uY),round(uX))
			else:
				drone.send_rc_control(0,0,round(uY),round(uX))
			break	


		else: #if nobody is recognized take as reference centerX and centerY of the previous frame
			center_x = pre_center_x
			center_y = pre_center_y
			cv2.circle(frame, (int(center_x), int(center_y)), 1, (0,0,255), 10)

			error_X = -(rifX - center_x)
			error_Y = rifY - center_y

			cv2.line(frame, (int(rifX),int(rifY)), (int(center_x),int(center_y)), (0,255,255),5 )

			integral_X = integral_X + error_X*Tc 	# updating integral PID term
			integral_Y = integral_Y + error_Y*Tc 	# updating integral PID term

			uX = Kp_X*error_X + Ki_X*integral_X 	# updating control variable uX
			uY = Kp_Y*error_Y + Ki_Y*integral_Y		# updating control variable uY

			previous_error_X = error_X 				# update previous error variable
			previous_error_Y = error_Y				# update previous error Variable

			drone.send_rc_control(0,0,round(uY),round(uX))

			continue

	
	cv2.imshow("Frame", frame)

	end = time.time()

	elapsed= end-start

	if Tc - elapsed > 0:
		time.sleep(Tc - elapsed)
	end_ = time.time()

	elapsed_ = end_ - start


	fps = 1/elapsed_

	# print fps
	print("FPS: ",fps)


	# end flight
	if cv2.waitKey(1) & 0xFF == ord("q"):
		break
	



# Create the animation
#animation = FuncAnimation(fig, update, frames=100, interval=50, blit=True)

# Show the plot
#plt.show()
drone.land()
drone.streamoff()
cv2.destroyAllWindows()
drone.land()
print("Landing...")
print("BATTERY: ")
print(drone.get_battery())
drone.end()