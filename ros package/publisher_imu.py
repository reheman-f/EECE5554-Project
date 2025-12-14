#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from vn_driver.msg import Vectornav
from scipy.spatial.transform import Rotation as R

class VnData:
    """Stores VN-100 IMU data"""
    
    def __init__(self, 
                 acceleration: list[float] = None,
                 angular_velocity: list[float] = None,
                 quaternion: list[float] = None,
                 magnetic_field: list[float] = None,
                 vnymr_string: str = ''):
        """
            acceleration: [x, y, z] in m/s²
            angular_velocity: [x, y, z] in rad/s
            quaternion: [x, y, z, w]
            magnetic_field: [x, y, z] in Tesla
            vnymr_string: string providing all raw data
        """
        self.acceleration = acceleration if acceleration is not None else [0.0, 0.0, 0.0]
        self.angular_velocity = angular_velocity if angular_velocity is not None else [0.0, 0.0, 0.0]
        self.quaternion = quaternion if quaternion is not None else [0.0, 0.0, 0.0, 1.0]
        self.magnetic_field = magnetic_field if magnetic_field is not None else [0.0, 0.0, 0.0]
        self.vnymr_string = vnymr_string

    def get_acceleration(self):
        return self.acceleration
    
    def get_angular_velocity(self):
        return self.angular_velocity
    
    def get_magnetic_field(self):
        return self.magnetic_field
    
    def get_quaternion(self):
        return self.quaternion

    def get_vnymr(self):
        return self.vnymr_string
    
    def __str__(self):
        return (f"VnData(\n"
                f"  Accel: {self.acceleration} m/s2\n"
                f"  Gyro:  {self.angular_velocity} rad/s\n"
                f"  Quat: {self.quaternion}\n"
                f"  Mag:   {self.magnetic_field} T\n"
                f"  VNYMR: {self.vnymr_string}\n"
                f")")
    
    def __repr__(self):
        return (f"VnData(acceleration={self.acceleration}, "
                f"angular_velocity={self.angular_velocity}, "
                f"quaternions={self.quaternion}, "
                f"magnetic_field={self.magnetic_field}, "
                f"VNYMR={self.vnymr_string})")
    

class VNDriver(Node):

    def __init__(self):
        super().__init__('publisher_imu')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.serialPortAddr = self.get_parameter('port').value
        self.output_rate = 40

        self.publisher_ = self.create_publisher(Vectornav, '/imu', 10)
        try:
            self.serialPort = serial.Serial(self.serialPortAddr, 115200, timeout=0.1)
        except Exception as error:
            self.get_logger().error(f"Failed to open serial port: {type(error).__name__} – {error}")
            return
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.publish_imu)
        self.get_logger().info("init complete, port: " + self.serialPortAddr)


    def publish_imu(self):
        imu_data = self.readSerial()
        if imu_data is not None:
            msg = Vectornav()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu1_frame'

            msg.orientation.x = imu_data.get_quaternion()[0]
            msg.orientation.y = imu_data.get_quaternion()[1]
            msg.orientation.z = imu_data.get_quaternion()[2]
            msg.orientation.w = imu_data.get_quaternion()[3]

            msg.angular_velocity.x = imu_data.get_angular_velocity()[0]
            msg.angular_velocity.y = imu_data.get_angular_velocity()[1]
            msg.angular_velocity.z = imu_data.get_angular_velocity()[2]

            msg.linear_acceleration.x = imu_data.get_acceleration()[0]
            msg.linear_acceleration.y = imu_data.get_acceleration()[1]
            msg.linear_acceleration.z = imu_data.get_acceleration()[2]

            msg.magnetic_field.x = imu_data.get_magnetic_field()[0]
            msg.magnetic_field.y = imu_data.get_magnetic_field()[1]
            msg.magnetic_field.z = imu_data.get_magnetic_field()[2]

            msg.orientation_covariance = [0.0] * 9
            msg.angular_velocity_covariance = [0.0] * 9
            msg.linear_acceleration_covariance = [0.0] * 9
            msg.magnetic_field_covariance = [0.0] * 9

            msg.imu_read = imu_data.get_vnymr()

            self.publisher_.publish(msg)


    def readSerial(self):
        vnymrRead = self.serialPort.readline().decode('utf-8').strip()
        
        if not (isVNYMRinString(vnymrRead)):
            return None
        # self.get_logger().info(vnymrRead)
        
        vnymrSplit = vnymrRead.split(',')
        numElements = 13
        if len (vnymrSplit) < numElements or len (vnymrSplit) > numElements:
            return
        for value in vnymrSplit:
            if value == '':
                return
        
        orientation_degrees = vnymrSplit[1:4]
        magnetic_field_gauss = vnymrSplit[4:7]
        acceleration_m_s2 = vnymrSplit[7:10]
        angular_velocity_rad_s = vnymrSplit[10:]

        checksum_char = "*"
        angular_velocity_rad_s[2] = angular_velocity_rad_s[2].split(checksum_char)[0]

        orientation_quaternions = eulerToQuaternion(orientation_degrees)
        
        tesla_per_gauss = 1E-4
        magnetic_field_tesla = [float(value_in_gauss) * tesla_per_gauss for value_in_gauss in magnetic_field_gauss]
        
        acceleration = convert_to_float(acceleration_m_s2)
        gyroscope = convert_to_float(angular_velocity_rad_s)

        vector_nav = VnData(acceleration, 
                            gyroscope, 
                            orientation_quaternions, 
                            magnetic_field_tesla, 
                            vnymrRead)
        
        # self.get_logger().info(str(vector_nav))
       
        return vector_nav
    

    def getWriteRegisterCmd(self): 
        header = "VNWRG"
        register_id = "07"
        value_to_set = str(self.output_rate)

        command = header + "," + register_id + "," + value_to_set
        checksum = calculate_checksum(command)
        command = "$" + command + "*" + checksum + "\r\n"
        
        return command
    
    


    def set_output_frequency(self):
        command = self.getWriteRegisterCmd()
        # self.get_logger().info(f"Sending: {"$VNRFS*5F".encode('utf-8')}")
        # self.serialPort.write("$VNRFS*5F".encode('utf-8'))
        # response = self.serialPort.readline().decode('utf-8').strip()
        # self.get_logger().info(f"Response: {response}")
        self.get_logger().info(f"Sending: {command.encode('utf-8')}")
        self.serialPort.write(command.encode('utf-8'))
        self.get_logger().info(f"Wrote command: {command.encode('utf-8')}")
        response = self.serialPort.readline().decode('utf-8').strip()
        self.get_logger().info(f"Response: {response}")
        # self.serialPort.write("$VNASY,1*XX".encode('utf-8'))


        
    def destroy_node(self):
        if hasattr(self, 'serialPort') and self.serialPort.is_open:
            self.serialPort.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()

        
def eulerToQuaternion(eulerAngles):
    rotation = R.from_euler('zyx', eulerAngles, degrees=True)
    return rotation.as_quat().tolist()


def convert_to_float(from_imu):
    return [float(raw_value) for raw_value in from_imu]


def isVNYMRinString(inputString):
    good_string = "$VNYMR"
    return good_string in inputString



    

def calculate_checksum(command):
    """Calculate XOR checksum for VectorNav command"""
        
    checksum = 0
    for char in command:
        checksum ^= ord(char)
    
    return f"{checksum:02X}"
    # return "XX"


def main(args=None):
    rclpy.init(args=args)
    node = VNDriver()
    # node.set_output_frequency()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

