import rospy
import serial
import sys
import yaml
from time import sleep
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import MagneticField,Imu

class imu_pub():
    def __init__(self):
        self.serialPort = ""
        with open('/home/ubuntu/chickies-robot/resources/comports.yaml', "r") as stream:
            try:
                params = yaml.safe_load(stream)
                self.serialPort=serial.Serial(params['NANO_IMU_PORT'],params['NANO_IMU_BAUDRATE'])
            except yaml.YAMLError as exc:
                print(exc)
        rospy.init_node('imu', anonymous=True)
        self.raw_pub = rospy.Publisher('imu_data', Imu, queue_size=10)
        self.mag_pub = rospy.Publisher('imu_mag', MagneticField, queue_size=10)
        self.rate = rospy.Rate(params['NANO_IMU_UPDATE_RATE'])
        rospy.loginfo(rospy.get_caller_id() + "  IMU node launched.")

    def spin(self):
        while not rospy.is_shutdown():
            try:
                read = self.serialPort.readline().decode('utf-8')
                data = read.split(',')
                if(data[0] == 'I' and len(data) == 11 and data[10] == '**\r\n'):
                    raw_msg = Imu()
                    raw_msg.header.stamp = rospy.Time.now()
                    raw_msg.header.frame_id = 'inertial'
                        
                    raw_msg.orientation.w = -1
                    raw_msg.orientation.x = 0
                    raw_msg.orientation.y = 0
                    raw_msg.orientation.z = 0
                        
                    raw_msg.linear_acceleration.x = float(data[1])
                    raw_msg.linear_acceleration.y = float(data[2])
                    raw_msg.linear_acceleration.z = -float(data[3])
                        
                    raw_msg.angular_velocity.x = float(data[4])
                    raw_msg.angular_velocity.y = float(data[5])
                    raw_msg.angular_velocity.z = float(data[6])
                        
                    #raw_msg.orientation_covariance[0] = -1
                    raw_msg.orientation_covariance = [-1, 1, 1, 1, 1, 1, 1, 1, 1]
                    #raw_msg.linear_acceleration_covariance[0] = -1
                    raw_msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
                    #raw_msg.angular_velocity_covariance[0] = -1
                    raw_msg.angular_velocity_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
                    self.raw_pub.publish(raw_msg)
                        
                    mag_msg = MagneticField()
                    mag_msg.header.stamp = rospy.Time.now()
                    mag_msg.magnetic_field.x = float(data[7])
                    mag_msg.magnetic_field.y = float(data[8])
                    mag_msg.magnetic_field.z = float(data[9])
                    mag_msg.magnetic_field_covariance = [1.2, 0, 0, 0, 1.2, 0, 0, 0, 1.2]
                    self.mag_pub.publish(mag_msg)
                    rospy.loginfo(raw_msg.linear_acceleration.z)

                self.rate.sleep()
            except serial.serialutil.SerialException:
                sleep(0.5)
            except Exception as ex:
                template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                message = template.format(type(ex).__name__, ex.args)
                print(message)
                sleep(0.5) 
        self.serialPort.close()

if __name__ == '__main__':
    try:
        imu_pub = imu_pub()
        imu_pub.spin()
    except rospy.ROSInterruptException:
        pass
