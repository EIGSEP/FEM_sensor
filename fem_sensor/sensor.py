import math
import redis
import smbus

ERROR_VALUE = -1  # default value for status reports if comms are broken
ERROR_STRING = "UNKNOWN"  # default string for status reports if comms broken

REDIS_HOST = "localhost"
REDIS_PORT = 6379


class Fem:

    # i2c addresses
    IMU_ADDR = 0x69  # accelerometer
    SW_ADDR = 0x20  # switch gpio
    # i2c registsers for switch
    CONTROL_REG = 2
    TRANSMIT_REG = 3
    RECEIVE_REG = 3
    STATUS_REG = 4
    COMMAND_REG = 4
    # write/read bit for switch
    WRITE_BIT = 0
    READ_BIT = 1
    # i2c command
    CMD_START = 1 << 7
    CMD_STOP = 1 << 6
    # switch modes
    SWMODE = {"load": 0b000, "antenna": 0b110, "noise": 0b001}

    def __init__(self, bus_num=1):
        self.bus = smbus.SMBus(bus_num)
        self.initialize_imu()
        # switch
        self.SWMODE_REV = {v: k for k, v in self.SWMODE.items()}
        # redis
        self.redis = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)

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
        self.bus.write_byte_data(self.IMU_ADDR, 0x6B, 0x00)
        self.bus.write_byte_data(self.IMU_ADDR, 0x1C, 0x00)
        self.bus.write_byte_data(self.IMU_ADDR, 0x1B, 0x00)
        print("Initialization of IMU successful.")

    def get_status(self):
        """Return dict of config status."""
        rv = {}
        try:
            switch, east, north = self.switch()
            rv["switch"] = switch
            # rv["lna_power_e"] = east
            # rv["lna_power_n"] = north
            # rv["temp"] = self.temperature()
            # rv["voltage"] = self.shunt("u")
            # rv["current"] = self.shunt("i")
            # rv["id"] = self.id()
            theta, phi = self.imu()  # XXX
            rv["imu_theta"] = theta
            rv["imu_phi"] = phi
            # rv["pressure"] = self.pressure()
            # rv["humidity"] = self.humidity()
        except (RuntimeError, IOError):
            rv["switch"] = ERROR_STRING
            # rv["lna_power_e"] = ERROR_VALUE
            # rv["lna_power_n"] = ERROR_VALUE
            # rv["temp"] = ERROR_VALUE
            # rv["voltage"] = ERROR_VALUE
            # rv["current"] = ERROR_VALUE
            # rv["id"] = ERROR_STRING
            rv["imu_theta"] = ERROR_VALUE
            rv["imu_phi"] = ERROR_VALUE
            # rv["pressure"] = ERROR_VALUE
            # rv["humidity"] = ERROR_VALUE
        return rv

    def read_accel(self):
        """
        Reads the accelerometer data from the IMU (MPU-9250).

        Returns
        --------
        tuple of int
            x, y, z
        """
        data = self.bus.read_i2c_block_data(self.IMU_ADDR, 0x3B, 6)
        x = data[0] << 8 | data[1]
        y = data[2] << 8 | data[3]
        z = data[4] << 8 | data[5]

        # converting to signed values.
        x = x - 65536 if x > 32767 else x
        y = y - 65536 if y > 32767 else y
        z = z - 65536 if z > 32767 else z
        return x, y, z

    def calc_position(x, y, z):  # XXX theta/phi mixed up
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

    def switch(self, mode=None, east=None, north=None, verify=False):
        """Switch between antenna, noise and load mode

        Example:
        switch()    # Get mode&status in (mode, east, north)
                    # eg, ('antenna', True, True)
                    # if the switch is set to antenna and both LNAs are on.
        switch(mode='antenna')  # Switch into antenna mode
        switch(mode='noise')    # Switch into noise mode
        switch(mode='load')     # Switch into load mode
        switch(east=True)       # Switch on east pole
        switch(north=False)     # Switch off north pole
        """
        try:
            val = self._switch_read(length=1)
        except Exception:
            raise RuntimeError("I2C RF switch read failure")
        cur_e = bool(val & 0b00001000)
        cur_n = bool(val & 0b00010000)
        cur_mode = self.SWMODE_REV.get(val & 0b00000111, "Unknown")
        if mode is None and east is None and north is None:
            return cur_mode, cur_e, cur_n
        if east is None:
            east = east or cur_e
        if north is None:
            north = cur_n
        if mode is None:
            mode = cur_mode
        new_val = 0b00000000
        if east:
            new_val |= 0b00001000
        if north:
            new_val |= 0b00010000
        new_val |= self.SWMODE.get(mode, val & 0b00000111)
        self._switch_write(new_val)
        if verify:
            assert new_val == self._switch_read()  # XXX

    def _switch_read(self, length=1):
        """
        I2C reads arbitary number of bytes from a slave device, It carries out
        the following steps:
        1. Send start
        2. Send address and R/W bit low (expecting an ack from the slave)
        3. Receive one or more bytes (expecting an ack after sending each byte)
        4. Receive the last byte
        4. Send stop

        length : number of bytes to read
        """
        data = []
        self.bus.write_byte_data(
            self.SW_ADDR,
            self.TRANSMIT_REG,
            (self.SW_ADDR << 1 | self.WRITE_BIT),
        )
        self.bus.write_byte_data(
            self.SW_ADDR, self.COMMAND_REG, self.CMD_START | self.CMD_WRITE
        )
        for i in range(length - 1):
            # the command below also gives an ACK signal from master to slave
            # because CMD_ACK is actually 0
            self.bus.write_byte_data(
                self.SW_ADDR, self.COMMAND_REG, self.CMD_WRITE
            )
            ret = self.bus.read_byte_data(self.SW_ADDR, self.RECEIVE_REG)
            data.append(ret)
        # the last read ends with a NACK signal and a stop signal from master
        # to slave
        self.bus.write_byte_data(
            self.SW_ADDR,
            self.COMMAND_REG,
            self.CMD_READ | self.CMD_NACK | self.CMD_STOP,
        )
        data.append(self.bus.read_byte_data(self.SW_ADDR, self.RECEIVE_REG))
        if length == 1:
            return data[0]
        return data

    def _switch_write(self, new_val):
        """
        I2C writes arbitary number of bytes to a slave device, It carries out
        the following steps:
        1. Send start
        2. Send address and R/W bit low (expecting an ack from the slave)
        3. Send one or more bytes (expecting an ack after sending each byte)
        4. Send stop

        new_val : the new switch value to be written
        """
        self.bus.write_byte_data(
            self.SW_ADDR,
            self.TRANSMIT_REG,
            (self.SW_ADDR << 1 | self.WRITE_BIT),
        )
        self.bus.write_byte_data(
            self.SW_ADDR, self.COMMAND_REG, self.CMD_START | self.CMD_WRITE
        )
        self.bus.write_byte_data(self.SW_ADDR, self.TRANSMIT_REG, new_val)
        self.bus.write_byte_data(
            self.SW_ADDR, self.COMMAND_REG, self.CMD_WRITE
        )
        self.bus.write_byte_data(self.SW_ADDR, self.COMMAND_REG, self.CMD_STOP)

    def update_redis(self, theta, phi):
        """
        Updates the Redis database with the accelerometer data.
        """
        self.redis.set("theta", theta)
        self.redis.set("phi", phi)
