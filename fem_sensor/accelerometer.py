import smbus
import math

IMU_ADDR = 0x69


class IMU:
    def __init__(self, bus_num=1, imu_addr=IMU_ADDR):
        self.bus = smbus.SMBus(bus_num)
        self.imu_addr = imu_addr
        self.initialize_imu()

    def initialize_imu(self):
        """
        Initializing the IMU on the FEM by configuring the necessary registers.

        Registers
        ----------
        0x6b: power management
        0x1b: gyro config          | default range +/- 250 deg/s
        0x1c: accelerometer config | default range +/- 2g (?)

        note: check motion.py or blocks.py
        """
        try:
            self.bus.write_byte_data(self.imu_addr, 0x6B, 0x00)
            self.bus.write_byte_data(self.imu_addr, 0x1C, 0x00)
            self.bus.write_byte_data(self.imu_addr, 0x1B, 0x00)
            print("Initialization successful.")
        except Exception as e:
            print(f"Error initializing {e}")

    def read_accel(self):
        """
        Reads the accelerometer data from the IMU (MPU-9250).

        Returns
        --------
        tuple of int
            x, y, z
        """
        try:
            data = self.bus.read.i2c_block_data(self.imu_addr, 0x3B, 6)
            x = data[0] << 8 | data[1]
            y = data[2] << 8 | data[3]
            z = data[4] << 8 | data[5]

            # converting to signed values.
            x = x - 65536 if x > 32767 else x
            y = y - 65536 if y > 32767 else y
            z = z - 65536 if z > 32767 else z
            return x, y, z
        except Exception as e:
            print(f"Error reading accelerometer: {e}")
            return None, None, None

    def calc_position(self, x, y, z):
        """
        Calculates the roll (theta) and pitch (phi) angles based on
        the accelerometer data.

        Parameters
        -----------
        x, y, z : int
            accelerometer values

        Returns
        --------
        tuple of float
            theta and phi (degrees)
        """
        # assuming the accelerometer is set to +/- 2g, where +/- 1g translates
        # to 16384 LSB/g (least significant bits per g).
        # 16384 = 2^14 because the accelerometer data for each axis is 16-bit
        # (2 bytes, 6 bytes total).
        ax = x / 16384.0
        ay = y / 16384.0
        az = z / 16384.0
        theta = math.atan2(ay, az) * 180.0 / math.pi
        phi = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi

        return theta, phi
