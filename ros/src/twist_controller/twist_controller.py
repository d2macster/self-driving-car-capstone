GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import rospy
import math

from pid import PID
from yaw_controller import YawController


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.pid_controller = PID(1.0, 0.2, 4)
        self.yaw_controller = YawController(
            wheel_base=kwargs['wheel_base'],
            steer_ratio=kwargs['steer_ratio'],
            min_speed=kwargs['min_speed'],
            max_lat_accel=kwargs['max_lat_accel'],
            max_steer_angle=kwargs['max_steer_angle']
        )

        self.prev_time = None
        self.prev_throttle = 0

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
        delta_t = float(rospy.get_time() - self.prev_time)
        self.prev_time = rospy.get_time()

        control = self.pid_controller.step(
            error=delta_v,
            sample_time=delta_t
        )
        # throttle : [0 .. 1]
        # brake : torque (N*m) = F(acceleration, weight, wheel radius)


        if control > 0:
            throttle = 2 * math.tanh(control)
            throttle = max(0.0, min(1.0, throttle))
            if throttle - self.prev_throttle > 0.1:
                throttle = self.prev_throttle + 0.1
            brake = 0.0
        elif control >= -1:
            throttle = 0.0
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(self.max_brake_torque, -control + 1)

        self.prev_throttle = throttle

        return throttle, brake, steering
