import rclpy
from rclpy.node import Node
import lgpio
import smbus
import time
from enum import Enum
from dataclasses import dataclass


LED                 = 4
GYROSCOPE_ADDR      = 0x69
I2C_TIMER           = 0.01
BIT_8_MASK          = 0xFF
BIT_16_MASK         = 0xFFFF
MAX_OFFSET_SAMPLES  = 1000
SCALE_CORRECTION    = 0


h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, LED)

i2c = smbus.SMBus(1)

# Registers of the Gyroscope
# 0x0C - X
# 0x0D - X
# 0x0E - Y
# 0x0F - Y
# 0x10 - Z
# 0x11 - Z


@dataclass
class GyroData:
    x: int
    y: int
    z: int


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


class GyroscopeNode(Node):
    def __init__(self):
        super().__init__('gyroscope_topic')

        self.state = 0

        self.gyro_offset_x = 0
        self.gyro_offset_y = 0
        self.gyro_offset_z = 0

        self.timer = self.create_timer(I2C_TIMER, self.timer_callback)

        self.init_gyroscope()


    # Method to setup the config for the gyroscope
    def init_gyroscope(self):
        # Set accelerometer in low power mode
        i2c.write_byte_data(GYROSCOPE_ADDR, BMI160Registers.COMMAND_REGISTER.value, 0x12)

        time.sleep(0.004)   # Sleep for 4 miliseconds

        # Set gyroscope in normal mode
        i2c.write_byte_data(GYROSCOPE_ADDR, BMI160Registers.COMMAND_REGISTER.value, 0x15)

        time.sleep(0.080)   # Sleep for 80 miliseconds

        # Set magneto meter into suspend mode
        i2c.write_byte_data(GYROSCOPE_ADDR, BMI160Registers.COMMAND_REGISTER.value, 0x1B)

        time.sleep(0.001) # Sleep for 1 milisecond

        self.calibration_process()

        print("Finished initializing gyroscope!")


    def calibration_process(self):
        # collect offset samples and determine the bias
        for i in range(0, MAX_OFFSET_SAMPLES):
            data = self.read_data_from_gyro(enable_offset=False)

            self.gyro_offset_x += data.x
            self.gyro_offset_y += data.y
            self.gyro_offset_z += data.z

        self.gyro_offset_x = self.gyro_offset_x / MAX_OFFSET_SAMPLES
        self.gyro_offset_y = self.gyro_offset_y / MAX_OFFSET_SAMPLES
        self.gyro_offset_z = self.gyro_offset_z / MAX_OFFSET_SAMPLES

        # TODO: Add code for scale correction


    # Timer callback that periodicly checks if there are values ready from the sensor
    def timer_callback(self):
        # lgpio.gpio_write(h, LED, self.state)
        # self.state = not self.state

        data = self.read_data_from_gyro()

        print(f"Gyro data: {data}")
        print(f"Offset x: {self.gyro_offset_x}, Offset y: {self.gyro_offset_y}, Offset z: {self.gyro_offset_z}")


    def read_data_from_gyro(self, enable_offset = True):
        # Trying to read 1 byte from the gyroscope and mask the value to make sure it´s 8bit
        x_lsb = i2c.read_byte_data(GYROSCOPE_ADDR, BMI160Registers.GYROSCOPE_X_LSB.value) & BIT_8_MASK
        x_msb = i2c.read_byte_data(GYROSCOPE_ADDR, BMI160Registers.GYROSCOPE_X_MSB.value) & BIT_8_MASK
        y_lsb = i2c.read_byte_data(GYROSCOPE_ADDR, BMI160Registers.GYROSCOPE_Y_LSB.value) & BIT_8_MASK
        y_msb = i2c.read_byte_data(GYROSCOPE_ADDR, BMI160Registers.GYROSCOPE_Y_MSB.value) & BIT_8_MASK
        z_lsb = i2c.read_byte_data(GYROSCOPE_ADDR, BMI160Registers.GYROSCOPE_Z_LSB.value) & BIT_8_MASK
        z_msb = i2c.read_byte_data(GYROSCOPE_ADDR, BMI160Registers.GYROSCOPE_Z_MSB.value) & BIT_8_MASK

        return self.construct_gyro_data(x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb, enable_offset)


    # Method to construct the gyro data into a more readable format
    def construct_gyro_data(self, x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb, enable_offset):
        gd = GyroData(0, 0, 0)

        # Resolution of the gyro is 16 bit, make sure with the mask that it is 16 bit
        gd.x = ((x_msb<<8) | x_lsb) & BIT_16_MASK
        gd.y = ((y_msb<<8) | y_lsb) & BIT_16_MASK 
        gd.z = ((z_msb<<8) | z_lsb) & BIT_16_MASK

        if enable_offset:
            gd.x -= self.gyro_offset_x
            gd.y -= self.gyro_offset_y
            gd.z -= self.gyro_offset_z

        return gd


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
