#!/usr/bin/env python3
"""
Phidget Spatial → ROS 2 driver (Python, rclpy)

Publishes:
  - /imu/data_raw : sensor_msgs/Imu  (orientation unset; angular_velocity in rad/s; linear_acceleration in m/s^2)
  - /imu/mag      : sensor_msgs/MagneticField  (optional, disabled by default)

Notes:
- Assumes Phidget22 + udev rules already installed (no sudo needed).
- By default converts accel from g→m/s^2 and gyro from deg/s→rad/s.
- Calibrates a simple gyro bias on startup while the device is still.
"""

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

from Phidget22.Devices.Spatial import Spatial
from Phidget22.PhidgetException import PhidgetException


G = 9.80665  # m/s^2 per g
DEG2RAD = math.pi / 180.0
UT_TO_T = 1e-6
GAUSS_TO_T = 1e-4


class PhidgetSpatialDriver(Node):
    def __init__(self):
        super().__init__('phidget_spatial_driver')

        # ---------- Parameters ----------
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate_hz', 100.0)          # desired ROS publish rate
        self.declare_parameter('accel_in_g', True)                 # True: convert g→m/s^2
        self.declare_parameter('gyro_in_deg', True)                # True: convert deg/s→rad/s
        self.declare_parameter('publish_mag', False)               # publish /imu/mag
        self.declare_parameter('mag_units', 'uT')                  # 'uT', 'G', or 'T'
        self.declare_parameter('calibrate_gyro_bias', True)        # estimate zero-rate bias at start
        self.declare_parameter('calib_duration_sec', 3.0)          # seconds (keep IMU still)
        self.declare_parameter('accel_cov', 0.0)                   # diag entries
        self.declare_parameter('gyro_cov', 0.0)                    # diag entries

        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.accel_in_g = bool(self.get_parameter('accel_in_g').value)
        self.gyro_in_deg = bool(self.get_parameter('gyro_in_deg').value)
        self.publish_mag = bool(self.get_parameter('publish_mag').value)
        self.mag_units = str(self.get_parameter('mag_units').value).lower()
        self.calib_gyro = bool(self.get_parameter('calibrate_gyro_bias').value)
        self.calib_dur = float(self.get_parameter('calib_duration_sec').value)
        self.accel_cov = float(self.get_parameter('accel_cov').value)
        self.gyro_cov = float(self.get_parameter('gyro_cov').value)

        # ---------- Publishers ----------
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 50)
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', 10) if self.publish_mag else None

        # ---------- Device + State ----------
        self.spatial = Spatial()
        self.spatial.setOnSpatialDataHandler(self._on_spatial_data)

        self._lock = threading.Lock()
        self._have_data = False
        self._accel = (0.0, 0.0, 0.0)     # raw from device (g or m/s^2)
        self._gyro  = (0.0, 0.0, 0.0)     # raw from device (deg/s or rad/s)
        self._mag: Optional[tuple] = None # raw from device (uT/G/T depending on device/config)

        # Gyro bias estimation
        self._bias_active = self.calib_gyro
        self._bias_samples = []
        self._gyro_bias = (0.0, 0.0, 0.0)

        # ---------- Open device ----------
        try:
            self.spatial.openWaitForAttachment(5000)
        except PhidgetException as e:
            self.get_logger().error(f'Failed to attach Phidget Spatial: {e}')
            raise

        # Set device data interval to as fast as possible, publish at desired rate
        try:
            min_dt = self.spatial.getMinDataInterval()
            max_dt = self.spatial.getMaxDataInterval()
            # Aim for device callback faster or equal to publish rate; cap to range
            target_dt = max(min_dt, min(max_dt, int(1000.0 / max(self.rate_hz, 1.0))))
            self.spatial.setDataInterval(target_dt)
            self.get_logger().info(f'Phidget Spatial attached. DataInterval={target_dt} ms')
        except Exception as e:
            self.get_logger().warn(f'Could not set DataInterval: {e}')

        # Start bias calibration timer if requested
        if self._bias_active:
            self.get_logger().info(f'Calibrating gyro bias for {self.calib_dur:.1f}s. Keep IMU still…')
            self.create_timer(self.calib_dur, self._finish_bias_calibration)

        # ROS publish timer
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self._publish)

    # --------- Phidget callback ---------
    def _on_spatial_data(self, dev, acceleration, angularRate, magneticField, timestamp):
      with self._lock:
          self._accel = (float(acceleration[0]), float(acceleration[1]), float(acceleration[2]))
          self._gyro  = (float(angularRate[0]),  float(angularRate[1]),  float(angularRate[2]))
          self._mag   = (float(magneticField[0]), float(magneticField[1]), float(magneticField[2])) if magneticField is not None else None
          self._have_data = True
          if self._bias_active:
              self._bias_samples.append(self._gyro)

    def _finish_bias_calibration(self):
        with self._lock:
            if not self._bias_active:
                return
            n = len(self._bias_samples)
            if n >= 10:
                bx = sum(g[0] for g in self._bias_samples) / n
                by = sum(g[1] for g in self._bias_samples) / n
                bz = sum(g[2] for g in self._bias_samples) / n
                self._gyro_bias = (bx, by, bz)
                self.get_logger().info(f'Gyro bias (device units): {self._gyro_bias}')
            else:
                self.get_logger().warn('Not enough samples for gyro bias; skipping.')
            self._bias_active = False
            self._bias_samples = []

    # --------- ROS publish ---------
    def _publish(self):
        if not self._have_data:
            return

        with self._lock:
            ax, ay, az = self._accel
            gx, gy, gz = self._gyro
            if self._bias_active is False:
                gx -= self._gyro_bias[0]
                gy -= self._gyro_bias[1]
                gz -= self._gyro_bias[2]
            mag = self._mag

        # Unit conversions
        if self.accel_in_g:
            ax, ay, az = ax * G, ay * G, az * G
        if self.gyro_in_deg:
            gx, gy, gz = gx * DEG2RAD, gy * DEG2RAD, gz * DEG2RAD

        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.frame_id

        # Imu message (orientation unknown)
        imu_msg = Imu()
        imu_msg.header = hdr
        imu_msg.orientation_covariance[0] = -1.0  # orientation not provided
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.angular_velocity_covariance[0] = self.gyro_cov
        imu_msg.angular_velocity_covariance[4] = self.gyro_cov
        imu_msg.angular_velocity_covariance[8] = self.gyro_cov
        imu_msg.linear_acceleration_covariance[0] = self.accel_cov
        imu_msg.linear_acceleration_covariance[4] = self.accel_cov
        imu_msg.linear_acceleration_covariance[8] = self.accel_cov

        self.pub_imu.publish(imu_msg)

        # Optional magnetometer
        if self.publish_mag and self.pub_mag and mag is not None:
            mx, my, mz = mag
            if self.mag_units == 'ut':
                mx, my, mz = mx * UT_TO_T, my * UT_TO_T, mz * UT_TO_T
            elif self.mag_units == 'g':
                mx, my, mz = mx * GAUSS_TO_T, my * GAUSS_TO_T, mz * GAUSS_TO_T
            # else assume already Tesla
            mag_msg = MagneticField()
            mag_msg.header = hdr
            mag_msg.magnetic_field.x = mx
            mag_msg.magnetic_field.y = my
            mag_msg.magnetic_field.z = mz
            # Leave covariance zero (unknown)
            self.pub_mag.publish(mag_msg)

    def destroy_node(self):
        try:
            self.spatial.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PhidgetSpatialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
