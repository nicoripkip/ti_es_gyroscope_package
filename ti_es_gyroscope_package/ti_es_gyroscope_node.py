import rclpy
from rclpy.node import Node
import lgpio
import smbus
import time
from enum import Enum
from dataclasses import dataclass


from std_msgs.msg import String


# Settings
LED                 = 4
MUL_ADDR            = 0x70 # TODO: Set this value to parameter
I2C_TIMER           = 0.1
BIT_8_MASK          = 0xFF
BIT_16_MASK         = 0xFFFF
MAX_OFFSET_SAMPLES  = 1000
SCALE_CORRECTION    = 0


# Setup i2c
i2c = smbus.SMBus(1)


@dataclass
class GyroMeta:
    id: str


@dataclass
class GyroData:
    x: int
    y: int
    z: int


# Enumeration for the different angular rate options
class GyroAngularRange(Enum):
    GYRO_RANGE_2000 = 0x00
    GYRO_RANGE_1000 = 0x01
    GYRO_RANGE_500  = 0x02
    GYRO_RANGE_250  = 0x03
    GYRO_RANGE_125  = 0x04


# Enumeration for the different output rate options
class GyroOutputRate(Enum):
    GYRO_RATE_25    = 0x06 #0b0110
    GYRO_RATE_100   = 0x08 #0b1000
    GYRO_RATE_1600  = 0x0C #0b1100
    GYRO_RATE_3200  = 0x0D #0b1101


# Enum with all important register addresses
class BMI160Registers(Enum):
    # Allround registers
    BMI160_STATUS       = 0x03  # Allround statuses
    DATA_STATUS         = 0x1B  # Status if data is ready
    COMMAND_REGISTER    = 0x7E

    # Gyros data registers
    GYROSCOPE_X_LSB     = 0x0C
    GYROSCOPE_X_MSB     = 0x0D
    GYROSCOPE_Y_LSB     = 0x0E
    GYROSCOPE_Y_MSB     = 0x0F
    GYROSCOPE_Z_LSB     = 0x10
    GYROSCOPE_Z_MSB     = 0x11

    # Config registers
    GYROSCOPE_CONF      = 0x42  # General settings of the gyro
    GYROSCOPE_RANGE     = 0x43  # Angular range of the gyro

    # interrupt registers


    # FIFO registers
    FIFO_DOWNS          = 0x45


class GyroscopeNode(Node):
    def __init__(self):
        super().__init__('gyroscope_topic')

        self.state = 0

        # Multiplexer address config
        # | A0 | A1 | A2 |
        # ----------------
        # | HH | LL | LL | address: 0x74, position: 1
        # | LL | HH | LL | address: 0x72, position: 2
        # | LL | LL | HH | address: 0x71, position: 3

        # Declare parameters
        self.declare_parameter("gyro_address", 0x00)
        self.declare_parameter("mul_address", 0x00)
        self.declare_parameter("mul_channel", 0)
        self.declare_parameter("mul_enable", False)

        # Define parameters for the node
        self.gyro_address   = self.get_parameter("gyro_address").value        # Use this one if there is no multiplexer used
        self.mul_address    = self.get_parameter("mul_address").value     # Use this one if multiplexer is used
        self.mul_enable     = self.get_parameter("mul_enable").value
        self.mul_channel    = self.get_parameter("mul_channel").value
  
        # Setup the publisher command
        self.gyro_publisher = self.create_publisher(String, "ti/es/gyro_data", 10)
        self.log_publisher  = self.create_publisher(String, "ti/es/log_data", 10)

        # Define gyro offset values for calibration
        self.gyro_offset_x = 0
        self.gyro_offset_y = 0
        self.gyro_offset_z = 0

        # Enable looptimer
        self.timer = self.create_timer(I2C_TIMER, self.timer_callback)

        try:
            self.init_gyroscope()
        except:
            pass


    # Method to setup the config for the gyroscope
    def init_gyroscope(self):
        log_msg = String()
        log_msg.data = f"[][info][gyroscope] Trying to find offsets for the gyroscope sensor!"

        self.log_publisher.publish(log_msg)

        if self.mul_enable:
            self.select_mul_channel(self.mul_channel)

        # Set accelerometer in low power mode
        i2c.write_byte_data(self.gyro_address, BMI160Registers.COMMAND_REGISTER.value, 0x12)

        time.sleep(0.004)   # Sleep for 4 miliseconds

        # Set gyroscope in normal mode
        i2c.write_byte_data(self.gyro_address, BMI160Registers.COMMAND_REGISTER.value, 0x15)

        time.sleep(0.080)   # Sleep for 80 miliseconds

        # Set magneto meter into suspend mode
        i2c.write_byte_data(self.gyro_address, BMI160Registers.COMMAND_REGISTER.value, 0x1B)

        time.sleep(0.001) # Sleep for 1 milisecond

        # set the range
        self.set_gyro_range(GyroAngularRange.GYRO_RANGE_2000)

        # Set the output rate
        self.set_gyro_output_rate(GyroOutputRate.GYRO_RATE_1600)

        self.calibration_process()

        print("Finished initializing gyroscope!")


    # Method for calibrating the data from the gyroscope
    # This process always happen on boot, but it can also
    # be done later in the process
    def calibration_process(self):
        # collect offset samples and determine the bias
        for i in range(0, MAX_OFFSET_SAMPLES):
            data = self.read_data_from_gyro(enable_offset=False)

            self.gyro_offset_x += data.x
            self.gyro_offset_y += data.y
            self.gyro_offset_z += data.z

            with open("training_data.txt", "a") as f:
                f.write(f"{data.x}, {data.y}, {data.z}\n")

            print(f"Training data: {data}")

        self.gyro_offset_x = self.gyro_offset_x / MAX_OFFSET_SAMPLES
        self.gyro_offset_y = self.gyro_offset_y / MAX_OFFSET_SAMPLES
        self.gyro_offset_z = self.gyro_offset_z / MAX_OFFSET_SAMPLES

        # TODO: Add code for scale correction


    # Timer callback that periodicly checks if there are values ready from the sensor
    def timer_callback(self):
        # If mul enabled, alwats first select the channel
        if self.mul_enable:
            self.select_mul_channel(self.mul_channel)

        # Take a reading from the gyro
        try:
            data = self.read_data_from_gyro()

            log_msg = String()
            log_msg.data = f"Gyro data: {data}"

            self.log_publisher.publish(log_msg)

            print(f"Gyro data: {data}")
            print(f"Offset x: {self.gyro_offset_x}, Offset y: {self.gyro_offset_y}, Offset z: {self.gyro_offset_z}")
        except:
            pass

        # Set the selected channel back to zero
        if self.mul_enable:
            self.reset_mul_channel()


    # Read data from the gyroscope sensor
    def read_data_from_gyro(self, enable_offset = True):
        # Trying to read 1 byte from the gyroscope and mask the value to make sure it´s 8bit
        x_lsb = i2c.read_byte_data(self.gyro_address, BMI160Registers.GYROSCOPE_X_LSB.value) & BIT_8_MASK
        x_msb = i2c.read_byte_data(self.gyro_address, BMI160Registers.GYROSCOPE_X_MSB.value) & BIT_8_MASK
        y_lsb = i2c.read_byte_data(self.gyro_address, BMI160Registers.GYROSCOPE_Y_LSB.value) & BIT_8_MASK
        y_msb = i2c.read_byte_data(self.gyro_address, BMI160Registers.GYROSCOPE_Y_MSB.value) & BIT_8_MASK
        z_lsb = i2c.read_byte_data(self.gyro_address, BMI160Registers.GYROSCOPE_Z_LSB.value) & BIT_8_MASK
        z_msb = i2c.read_byte_data(self.gyro_address, BMI160Registers.GYROSCOPE_Z_MSB.value) & BIT_8_MASK

        return self.construct_gyro_data(x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb, enable_offset)


    # Method to construct the gyro data into a more readable format
    def construct_gyro_data(self, x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb, enable_offset):
        gd = GyroData(0, 0, 0)

        # Resolution of the gyro is 16 bit, make sure with the mask that it is 16 bit
        gd.x = ((x_msb<<7) | x_lsb) & BIT_16_MASK
        gd.y = ((y_msb<<7) | y_lsb) & BIT_16_MASK 
        gd.z = ((z_msb<<7) | z_lsb) & BIT_16_MASK

        if enable_offset:
            gd.x -= self.gyro_offset_x
            gd.y -= self.gyro_offset_y
            gd.z -= self.gyro_offset_z

        return gd


    # Method to configure the channel for the multiplexer, choose between channel 1-8
    def select_mul_channel(self, channel: int):
        # Make sure channel is always 7 or smaller
        if channel > 7:
            channel = 7

        # Set the channel
        i2c.write_byte_data(self.mul_address, 0x00, (1 << (channel)-1) & BIT_8_MASK)


    # Method to reset the selected channel on the multiplexer
    def reset_mul_channel(self):
        i2c.write_byte_data(self.mul_address, 0x00, 0x00)


    # Method to set the output rate of the gyroscope
    def set_gyro_output_rate(self, rate: GyroOutputRate):
        i2c.write_byte_data(self.gyro_address, BMI160Registers.GYROSCOPE_CONF.value, rate.value)


    # Method to set the range in which the gyro samples
    def set_gyro_range(self, range: GyroAngularRange):
        i2c.write_byte_data(self.gyro_address, BMI160Registers.GYROSCOPE_RANGE.value, range.value)


def main(args=None):
    rclpy.init(args=args)
    gn = GyroscopeNode()

    print('Gyroscope Package has booted!')

    try:
        rclpy.spin(gn)
    except KeyboardInterrupt:
        pass
    finally:
        gn.destroy_node()
        # rclpy.shutdown()


if __name__ == '__main__':
    main()
