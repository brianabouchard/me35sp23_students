# This code detects a single press of a limit switch
# Written by Briana Bouchard 

import RPi.GPIO as GPIO
import time

#Define initial state of button 
pressed = False

# Set the GPIO mode
GPIO.setmode(GPIO.BOARD)

# Set GPIO pins for limit switch as input and activate the internal pull down resistor
GPIO.setup(29, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Callback function that is triggered by interrupt
def my_callback(channel):
    global pressed
    time.sleep(0.1)
    print("Button Pressed")
    pressed = True
    time.sleep(0.1)
    GPIO.remove_event_detect(channel)
 
try:
    # Add interrupt when pin switches from LOW to HIGH
    GPIO.add_event_detect(29, GPIO.RISING, callback=my_callback, bouncetime=200)
    
    # Hold in loop until button is pressed to prevent cleanup
    while not pressed:
        time.sleep(0.1)
        if pressed == True:
            print("Leaving Loop")
            break
        if pressed == False:
            continue
    
    GPIO.cleanup()
    print("Clean-up Complete")
    exit()   
    
except KeyboardInterrupt:
    GPIO.cleanup()