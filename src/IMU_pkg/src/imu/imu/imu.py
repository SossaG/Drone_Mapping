from Phidget22.Phidget import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.Devices.Gyroscope import *
import time

def onAccelerationChange(self, acceleration, timestamp):
	print("Acceleration: \t"+ str(acceleration[0])+ "  |  "+ str(acceleration[1])+ "  |  "+ str(acceleration[2]))
	print("Timestamp: " + str(timestamp))
	#print("----------")

def onAngularRateUpdate(self, angularRate, timestamp):
	print("AngularRate: \t"+ str(angularRate[0])+ "  |  "+ str(angularRate[1])+ "  |  "+ str(angularRate[2]))
	print("Timestamp: " + str(timestamp))
	#print("----------")

def main():
	accelerometer0 = Accelerometer()
	gyroscope0 = Gyroscope()

	accelerometer0.setOnAccelerationChangeHandler(onAccelerationChange)
	gyroscope0.setOnAngularRateUpdateHandler(onAngularRateUpdate)

	accelerometer0.openWaitForAttachment(5000)
	gyroscope0.openWaitForAttachment(5000)

	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	accelerometer0.close()
	gyroscope0.close()

main()
