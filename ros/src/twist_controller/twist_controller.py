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
        # usually we cannot change brake or throttle instantaneously
        # thus adding lpf to mimic real life human behavior
        # a = 0.14, b = 0.86
        self.brake_lpf = LowPassFilter(6, 1)
        self.throttle_lpf = LowPassFilter(6, 1)


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

    def control(self, twist_cmd, current_velocity, dbw_enabled, driving_mode):
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

        # if current_linear_velocity == 0:
        #     self.pid_controller.reset()

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

        action = 0

        # lets prevent ping-pong between throttle and brake
        # when we try to maintain requested speed

        if driving_mode == 1:
            # path planner is accelerating the car
            if delta_v > 0:
                # activate throttle right away if we are below desired speed limit
                action = 1
            if delta_v <= -0.2:
                # activate brake only if the speed difference is bigger that
                action = -1

        if driving_mode == -1:
            # path planner is slowing the car
            if delta_v < -0.1:
                # let the car slow down a bit on its own
                # if this is not enough - the activate brake
                action = -1
            if delta_v >= 0.2:
                # we slowed down too much - lets speed up a bit
                action = 1

        if desired_linear_velocity == 0:
            action = -1

        if action > 0:
            throttle = math.tanh(control)
            throttle = max(0.0, min(THROTTLE_MAX, throttle))
            throttle = self.throttle_lpf.filt(throttle)

            self.brake_lpf.filt(0.0)

        if action < 0:
            brake = 0.2*self.max_brake_torque*math.tanh(math.fabs(control))

            if desired_linear_velocity <= ONE_MPH * 1.0:
                brake = 0.4*self.max_brake_torque
                if current_linear_velocity <= ONE_MPH * 1.0:
                    brake = self.max_brake_torque
            brake = min(brake, self.max_brake_torque)
            brake = max(1.0, brake)

            brake = self.brake_lpf.filt(brake)
            self.throttle_lpf.filt(0.0)

        self.prev_throttle = throttle

        return throttle, brake, steering
