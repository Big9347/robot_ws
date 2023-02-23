import RPi.GPIO as GPIO
import random
from time import sleep
from gpiozero import OutputDevice
from gpiozero import Button

## I forgot put this in the befor script !!!

GPIO.setwarnings(False)
button1    = Button(16,pull_up=False) 
button2 = Button(17,pull_up=False) 

while True:
    if (button1.is_pressed):
          print('button1 pressed')
    if (button2.is_pressed):
          print('button2. pressed')
    sleep(0.02)
    print('.')