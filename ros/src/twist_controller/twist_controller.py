GAS_DENSITY = 2.858
ONE_MPH = 0.44704
THROTTLE_MAX = 0.75  # lets not drive very aggressively

import rospy
import math

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.pid_controller = PID(1.0, 0.005, 0.0)
        self.brake_lpf = LowPassFilter(0.2, 0.1)

        self.yaw_controller = YawController(
            wheel_base=kwargs['wheel_base'],
            steer_ratio=kwargs['steer_ratio'],
            min_speed=kwargs['min_speed'],
            max_lat_accel=kwargs['max_lat_accel'],
            max_steer_angle=kwargs['max_steer_angle']
        )

        self.prev_time = None
        self.prev_throttle = 0
        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']

        total_vehicle_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity'] * GAS_DENSITY
        self.max_brake_torque = total_vehicle_mass * abs(kwargs['decel_limit']) * kwargs['wheel_radius']

    def control(self, twist_cmd, current_velocity, dbw_enabled):
        """
        :return: (throttle, brake, steer)
        """

        throttle = 0.0
        brake = 0.0
        steering = 0.0

        if not dbw_enabled:
            self.pid_controller.reset()
            return throttle, brake, steering

        if not all((twist_cmd, current_velocity)):
            return throttle, brake, steering

        desired_linear_velocity = twist_cmd.twist.linear.x
        desired_angular_velocity = twist_cmd.twist.angular.z

        current_linear_velocity = current_velocity.twist.linear.x

        steering = self.yaw_controller.get_steering(
            linear_velocity=desired_linear_velocity,
            angular_velocity=desired_angular_velocity,
            current_velocity=current_linear_velocity)

        if not self.prev_time:
            self.prev_time = rospy.get_time()
            return throttle, brake, steering

        delta_v = desired_linear_velocity - current_linear_velocity
        delta_v = max(self.decel_limit, delta_v)
        delta_v = min(delta_v, self.accel_limit)
        delta_t = float(rospy.get_time() - self.prev_time)
        self.prev_time = rospy.get_time()

        control = self.pid_controller.step(
            error=delta_v,
            sample_time=delta_t
        )
        # throttle : [0 .. 1]
        # brake : torque (N*m) = F(acceleration, weight, wheel radius)

        if desired_linear_velocity == 0:
            control = -1

        if control > 0:
            throttle = math.tanh(control)
            throttle = max(0.0, min(THROTTLE_MAX, throttle))
            brake = 0.0
        else:
            throttle = 0.0

            brake = 0.4*self.max_brake_torque*math.tanh(math.fabs(delta_v))
            if desired_linear_velocity <= ONE_MPH * 1.0:
                brake = 0.4*self.max_brake_torque
                if current_linear_velocity <= ONE_MPH * 5.0:
                    brake = self.max_brake_torque
            brake = min(brake, self.max_brake_torque)
            brake = max(1.0, brake)

            brake = self.brake_lpf.filt(brake)

        self.prev_throttle = throttle

        return throttle, brake, steering
