import numpy as np
import cv2
import time
from code import interact

# Function to return 2-d binary of 3-d matrix where pixels == true
# if they are within the specified range from the corosponding 
# value in the vector 'values', which must be of length 3
def threshold3d(image, values, allowedDiff):
	image[:, :, 0] = np.logical_and(\
					image[:, :, 0] > (values[0] - allowedDiff),\
					image[:, :, 0] < (values[0] + allowedDiff))
	image[:, :, 1] = np.logical_and(\
					image[:, :, 1] > (values[1] - allowedDiff),\
					image[:, :, 1] < (values[1] + allowedDiff))
	image[:, :, 2] = np.logical_and(\
					image[:, :, 2] > (values[2] - allowedDiff),\
					image[:, :, 2] < (values[2] + allowedDiff))
	return np.logical_and(image[:, :, 0], image[:, :, 1], image[:, :, 2])	

# Function to select the color in the middle of the image
def selectCenterColor(frame):
	
	# Convert to HSV
	hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# Find color of mid point of frame
	mid = [frame.shape[0]/2, frame.shape[1]/2]
	color = [ hsvFrame[mid[0],mid[1],0], hsvFrame[mid[0],mid[1],1], hsvFrame[mid[0],mid[1],2] ]

	# Display the frame with the selected pixel blacked out
	frame[mid[0]-3:mid[0]+3, mid[1]-3:mid[1]+3, :] = 0
	cv2.imshow('frame', frame)
	if cv2.waitKey(1000) & 0xFF == ord('q'):
	  print "whoa whoa whoa..."
	  #break
	
	return color


# Function to segment the color from the frame
def processFrame(frame, colorWeWant):
    binaryFrame = frame

    # convert to HSV
    binaryFrame = cv2.cvtColor(binaryFrame, cv2.COLOR_BGR2HSV)
    
    # threshold around color we want 
    #colorWeWant = [15, 252, 209]    # <<--------- note: color is hard-coded here -----
    binaryFrame = threshold3d(binaryFrame, colorWeWant, 11)

    # close, then open to get rid of noise and unify object
    kernel = np.ones((10,10),np.uint8)
    binaryFrame = cv2.morphologyEx(binaryFrame, cv2.MORPH_CLOSE, kernel)
    kernel = np.ones((10,10),np.uint8)
    binaryFrame = cv2.morphologyEx(binaryFrame, cv2.MORPH_OPEN, kernel)

    # find and draw contours on original image
    contour_struct = cv2.findContours(binaryFrame, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours = contour_struct[0]
    #print len(contours)
    if len(contours)>0:
        x, y, w, h = cv2.boundingRect(contours[0])  # <<----  This assumes there is only 1 contour
        point1, point2 = (x, y), (x+w, y+h)
        cv2.rectangle(frame, point1, point2, [255, 255, 255], 2)

    return (frame)


##############   Main   ##############  

cap = cv2.VideoCapture()
cap.open(0)
#cap.open('ball.avi')

# Find mid point
ret, frame = cap.read()
mid = [frame.shape[0]/2, frame.shape[1]/2]

# Display frame with color to be selected blacked out
# so the user can see what's about to be selected
#while(True):
for i in range(300):
    time.sleep(.01)

    # Capture frame-by-frame
    ret, frame = cap.read()

	# Display the frame with the selected pixel blacked out
    frame[mid[0]-3:mid[0]+3, mid[1]-3:mid[1]+3, :] = 0
    cv2.imshow('frame', frame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        print "whoa whoa whoa..."

	# every 100 frames give countdown
    if i % 100 == 99:
        print ((i//100) + 1)

# Capture frame to get color from
ret, frame = cap.read()

#print "HERE!"

# Get color in the center of the image
colorWeWant = selectCenterColor(frame)
print colorWeWant

# Track the color in real time
while(True):
    time.sleep(.01)

    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    frame = processFrame(frame, colorWeWant)
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
          break


#interact(local=locals())






	
################# Mess around to try to get color of ball
"""
# Capture frame-by-frame
for i in range(100):
	ret, frame = cap.read()

# Our operations on the frame come here
processedFrame = processFrame(frame)

cv2.imshow('frame', processedFrame*255)
if cv2.waitKey(10) & 0xFF == ord('q'):
	print "something going on with the ord function"

#print processedFrame[150:160, 320:330]
#print processedFrame[50:60, 20:30]

time.sleep(5)

#print processedFrame[150:160, 320:330, 0]
#print processedFrame[150:160, 320:330, 1]
#print processedFrame[150:160, 320:330, 2]
#test1 = processedFrame[150, 320, 0:3]
#print test1.size

#print processedFrame.shape
#processedFrame[371:470, 541:640, 0:3] = processedFrame[1:100, 1:100, 0:3]
#processedFrame[1:100, 1:100, 0:3] = 0

# Display the resulting frame
#cv2.imshow('frame', processedFrame)
#blackImage = np.zeros(processedFrame.shape)
#print blackImage.shape

"""
##################



# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

	
