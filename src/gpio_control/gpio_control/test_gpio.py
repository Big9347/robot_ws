import RPi.GPIO as GPIO
import random
from time import sleep
from gpiozero import OutputDevice
from gpiozero import Button

## I forgot put this in the befor script !!!

GPIO.setwarnings(False)

buttonStart    = Button(16,pull_up=False) 
buttonStop = Button(21,pull_up=False) 
operation_lamp_relay = OutputDevice(17,active_high=True,initial_value=False)

while True:
    if (buttonStart.is_pressed and not operation_lamp_relay.value):  
        print('buttonStart pressed')
        operation_lamp_relay.on()
    elif (buttonStop.is_pressed and operation_lamp_relay.value):  
        print('buttonStop pressed')
        operation_lamp_relay.off() 
    else:
        print('.')
    sleep(.1)
    