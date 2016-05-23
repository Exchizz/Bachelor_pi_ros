#!/usr/bin/env python
import rospy
from msgs.msg import nmea
from time import sleep
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, True)

class LedOut:
	def __init__(self):
		rospy.init_node('led_fix_out', anonymous=True)
		rospy.Subscriber("/fmData/nmea_from_device", nmea, self.callback)

		self.led_blink = 0
		rospy.Timer(rospy.Duration(0.2), self.timerCallback)

		self.counter = 0
		self.led_state = False

	def callback(self, data):
		if data.type == "GPGGA":
			fix = data.data[5]
			if self.led_blink == 0:
				self.led_blink = int(fix)*2
			else:
				print "fix: ", fix

	def toggle_led(self):
		if self.led_state:
			self.led_state = False
		    	GPIO.output(18, True)
		else :
			self.led_state = True
		    	GPIO.output(18, False)

		if self.led_blink == 0:
			sleep(1)

	def timerCallback(self, event):
		if self.led_blink > 0:
			self.led_blink-=1
			self.toggle_led()

if __name__ == '__main__':
	ledout = LedOut()
	rospy.spin()



