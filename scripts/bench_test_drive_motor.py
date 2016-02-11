#!/usr/bin/env python

#import
import test2
import time
import RPi.GPIO as GPIO

#init GPIO
GPIO.setmode(GPIO.BOARD)

GPIO.setup(8, GPIO.OUT)
GPIO.setup(10, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)

#init var

choice = 0
dir_ = 0
N = 1000
Fmax = 1000
Fmin = 500
Plat_size = 0.5
pw = 0.001

#GPIO output
GPIO.output(12, dir_)
GPIO.output(10, GPIO.HIGH)
GPIO.output(8, GPIO.LOW)
while 1:
    print " "
    print " "
    print "- 1 : Single Step"
    print "- 2 : Change Direction"
    print "- 3 : Soft Step"
    print "- 4 : Soft Step Parameters"
    print "- 5 : EXIT"
    choice = input("Enter your choice : ")

    if choice == 1:
        GPIO.output(10, GPIO.LOW)
        time.sleep(0.01)

        p = GPIO.PWM(8, 700)
        p.start(50)
        time.sleep(1)
        p.stop()

        GPIO.output(10, GPIO.HIGH)

        print("STEP DONE")
        on = 0
        time.sleep(0.5)

    if choice == 2:
        dir_ = not dir_
        GPIO.output(12, dir_)
        print("Direction changed")

    if choice == 3:
        dt = test2.soft_move(N, Fmax, Fmin, Plat_size)

        GPIO.output(10, GPIO.LOW)
        time.sleep(0.01)

        for x in xrange(N):
            GPIO.output(8, GPIO.HIGH)
            time.sleep(dt[x]/2)
            GPIO.output(8, GPIO.LOW)
            time.sleep(dt[x]/2)
            GPIO.output(10, GPIO.HIGH)

    if choice == 4:
        N=input("Number of pulses : ")
            Fmax = input("Maximum Freqency (Hz) : ")
            Fmin = input("Minimum Frequency (Hz) : ")
            Plat_size = input("Plateau size (0 - 1) : ")
        print("Parameters Set")

    if choice == 5:
        GPIO.cleanup()
        print("GPIO cleaned")
        break

