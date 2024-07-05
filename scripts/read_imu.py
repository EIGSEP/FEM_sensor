import argparse
import time
from fem_sensor.accelerometer import IMU

parser = argparse.ArgumentParser(
    description="Read IMU data.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument(
    "-r",
    dest="update_redis",
    action="store_true",
    default=False,
    help="Update redis with the sensor data.",
)
args = parser.parse_args()

imu = IMU()
print("Press Ctrl+C to stop reading.")
try:
    while True:
        x, y, z = imu.read_accel()
        if x is not None:
            theta, phi = imu.calc_position(x, y, z)
            print(f"x:{x} y:{y} z:{z} Theta: {theta:.2f} Phi: {phi:.2f}")
            if args.update_redis:
                imu.update_redis(theta, phi)
        time.sleep(2)  # 2 second delay.
except KeyboardInterrupt:
    print("Stopped reading position. Self-destruct in 5...4...3...2...1")
