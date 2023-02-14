# Run 2 stepper motors simultaneously with different step inputs per motor
# Written by Briana Bouchard 

from Motors.ThreadStepperLib import Stepper
import RPi.GPIO as GPIO

# Define the GPIO pins for the L298N motor driver
Motor1 = [12,11,13,15]
Motor2 = [33,35,36,37]

try:
    # Define the steps per revolution for the motor 
    steps_rev = 200

    # Set the thread number, thread ID, motor, number of steps to move, steps per revolution,
    # and the speed in revolutions per minute
    stepper1 = Stepper(1,"Motor #1",Motor1, 50, steps_rev, 20)
    stepper2 = Stepper(2,"Motor #2",Motor2, -100, steps_rev, 20)

    # Start the motor threads
    stepper1.start()
    stepper2.start()

    # Check to see if both threads are done and clean up GPIO pins when done
    while True:
        if stepper1.is_alive() == False and stepper2.is_alive() ==False:
            GPIO.cleanup()
            break
except KeyboardInterrupt:
    GPIO.cleanup()
