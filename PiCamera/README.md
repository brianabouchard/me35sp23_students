# Raspberry Pi Camera Module 3 Instructions

The Raspberry Pi Camera Module 3 will become a key sensor for us throughout the semester. 

## Physically Connecting the Pi Camera Module 3
**First and foremost, unplug your Raspberry Pi before connecting or disconnecting your camera.** On your Pi, locate the camera connector (CSI) labelled `camera` and gently pull up on the black tabs on either end of the connector. Once the connector arm has risen, you can gently push it back towards the USB ports to increase the opening. Take the flex cable attached to your camera and orient it so that the blue side is facing the USB ports on the Pi. Insert the flex cable into the camera connector so that it is firmly seated and then gently slide the connector arm forward and down to lock it in place.

**Note: The connector, flex cable and camera are not particularly durable so please be delicate with them.**

## Testing Your Camera
Raspberry Pi OS comes with all the software and drivers you need to operate your camera (libcamera, picamera2, etc). For details on these built in pieces of software, see the Raspberry Pi documentation: https://www.raspberrypi.com/documentation/computers/camera_software.html#introducing-the-raspberry-pi-cameras

One of these built-in apps is a command line tool: `libcamera-hello` This will turn on your camera and open a preview window to stream the video from your camera. In order to see the results, you should connect to your Pi through VNC Viewer so you can see the desktop and the preview window. Test your camera by entering `libcamera-hello` into the command line. 

Alternatively, if you would prefer to access your Pi via SSH, you can use `libcamera-jpeg -o test.jpg` to take a photo and save it as test.jpg.


## Post-Processing
The picamera2 library provides us with a way to capture images and video from our camera via Python code. But in order to do post-processing, we'll be using the opencv library. To install this library, open terminal and enter: 

`pip install opencv-python`

Once opencv has been installed, try running Maddie's code found in `PiCamera_FilterRed.py`
