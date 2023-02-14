#Takes an image using a raspberry pi camera & finds the percentage of pixels in the image that are red
#by Maddie Pero

#import libraries 
from picamera2 import Picamera2 
import cv2 as cv 
import numpy as np
from libcamera import controls
import time

picam2 = Picamera2()

#configure the picamera
capture_config = picam2.create_still_configuration() #automatically 4608x2592 width by height (columns by rows) pixels
picam2.configure(capture_config)
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode

picam2.start() #must start the camera before taking any images
time.sleep(0.1)

img_name = 'image.jpg'
picam2.capture_file(img_name) #take image 

img = cv.imread("image.jpg") #read image with open cv, to get the bgr value of one pixel index using print(img[row][col])

total_pixels = img.shape #returns [2529, 4608] as the shape of the image

#create boundary for red values as two arrays
lower = np.array([0,0,130]) #lower range of bgr values for red
upper = np.array([70,70,255]) #upper range of bgr values for red

#determine if the pixel in the image has bgr values within the range
image_mask = cv.inRange(img,lower,upper) #returns array of 0s & 255s, 255=white=within range, 0=black=not in range
cv.imwrite("image2.jpg", image_mask) #write the mask to a new file so that it can be viewed 

in_range = np.count_nonzero(image_mask) #count the number of elements in the array that are not zero (in other words elements that are in the red range)
not_in_range = total_pixels[0]*total_pixels[1] - in_range 
total = total_pixels[0]*total_pixels[1]

percent_red = round((in_range/total)*100)
print(percent_red, "%")

picam2.stop() #stop the picam 
