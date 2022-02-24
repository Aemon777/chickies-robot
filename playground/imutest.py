from __future__ import print_function
import qwiic_icm20948
import time
import sys

def runExample():

	print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
	IMU = qwiic_icm20948.QwiicIcm20948()

	if IMU.connected == False:
		print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		print("try using sudo to run the file, I2C needs root permissions")
		return

	IMU.begin()

	#IMU.swReset()
	#time.sleep(.05)

	# set full scale range for both accel and gryo (separate functions)
	gpm8 = 0x02
	IMU.setFullScaleRangeAccel(gpm8)
	dps250 = 0x00
	dps500 = 0x01
	dps2000 = 0x03
	IMU.setFullScaleRangeGyro(dps250)

	# set low pass filter for both accel and gyro (separate functions)
	acc_d111bw4_n136bw = 0x02
	IMU.setDLPFcfgAccel(acc_d111bw4_n136bw)
	gyr_d119bw5_n154bw3 = 0x02
	IMU.setDLPFcfgGyro(gyr_d119bw5_n154bw3)

	# disable digital low pass filters on both accel and gyro
	IMU.enableDlpfAccel(True)
	IMU.enableDlpfGyro(True)

	IMU.startupMagnetometer()
	#IMU.enableDlpfAccel('on')
	#IMU.enableDlpfGyro('on')

	file1 = open("myfile.txt","w")
	# \n is placed to indicate EOL (End of Line)
	file1.writelines('X,Y,Z,\n')
	file1.close() #to change file access modes

	while True:
		if IMU.dataReady():
			IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
			"""print(\
			 '{: 06d}'.format(IMU.axRaw)\
			, '\t', '{: 06d}'.format(IMU.ayRaw)\
			, '\t', '{: 06d}'.format(IMU.azRaw)\
			, '\t', '{: 06d}'.format(IMU.gxRaw)\
			, '\t', '{: 06d}'.format(IMU.gyRaw)\
			, '\t', '{: 06d}'.format(IMU.gzRaw)\
			, '\t', '{: 06d}'.format(IMU.mxRaw)\
			, '\t', '{: 06d}'.format(IMU.myRaw)\
			, '\t', '{: 06d}'.format(IMU.mzRaw)\
			)"""
			aX = IMU.axRaw*9.81/1000/4
			aY = IMU.ayRaw*9.81/1000/4
			aZ = IMU.azRaw*9.81/1000/4
			
			gX = IMU.gxRaw/57.3
			gY = IMU.gyRaw/57.3
			gZ = IMU.gzRaw/57.3
			
			print('X: ','{: 06f}'.format(gX),'\t\tY: ','{: 06f}'.format(gY),'\t\tZ: ','{: 06f}'.format(gZ))

			file1 = open("myfile.txt","a")
			# \n is placed to indicate EOL (End of Line)
			file1.writelines(['{: 06f}'.format(gX),',','{: 06f}'.format(gY),',','{: 06f}'.format(gZ),',\n'])
			file1.close() #to change file access modes
			time.sleep(0.03)
		else:
			print("Waiting for data")
			time.sleep(0.5)

if __name__ == '__main__':
	try:
		runExample()
	except (KeyboardInterrupt, SystemExit) as exErr:
		print("\nEnding Example 1")

		sys.exit(0)
