
import time
from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, max_accel, max_steering, wheel_base, steer_ratio, min_speed, max_lat_accel, decel_limit, vehicle_mass, fuel_capacity, wheel_radius, brake_deadband, *args, **kwargs):
        # TODO: Implement
        self.pid_speed_controller = PID(2., 0., 0.)
        self.pid_accel_controller = PID(0.4, 0.1, 0., -1., 1.)
        self.lpf_accel = LowPassFilter(tau = 0.5, ts = 0.02)
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steering)

        self.last_visit = 0
        self.control_rate = 50. # dbw_node.py is currently set up to publish steering, throttle, and brake commands at 50hz
        self.control_period = 1.0 / self.control_rate;
        self.past_velocity_linear = None
        self.steer_kp = 0
        self.max_accel = max_accel
        self.max_steering = max_steering
        self.max_decel = decel_limit
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        pass

    def reset(self):
      self.pid_accel_controller.reset()
      self.pid_speed_controller.reset()

    def control(self, twist_cmd, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        throttle = 0. 
        brake = 0.
        steering = 0.


        # return default values if twist cmd or current velocity is null
        if (twist_cmd is  None) or (current_velocity is  None):
          return  throttle, brake, steering

        # return default values if dbw is disengaged
        if not dbw_enabled:
          return  throttle, brake, steering

        # do not run controller if this is the first time after init.
        if self.last_visit == 0:
          self.last_visit = time.time()
          self.past_velocity_linear = current_velocity.linear
          # return  throttle, brake, steering

        accel = self.control_period * (current_velocity.linear.x - self.past_velocity_linear.x)
        self.lpf_accel.filt(accel)

        
        linear_velocity_error = twist_cmd.linear.x - current_velocity.linear.x
        current_ts = time.time()
        dt =  current_ts - self.last_visit
        self.last_visit = current_ts
        acceleration = self.pid_speed_controller.step(linear_velocity_error, self.control_period)
        # acceleration = min(acceleration, self.max_accel)

        if(acceleration > 2*ONE_MPH):
          throttle = self.pid_accel_controller.step(acceleration - self.lpf_accel.get(), self.control_period)
          # if abs(throttle) <= 0.02:
          #   throttle = 0
        # elif (acceleration < -self.brake_deadband):
        elif (acceleration < 0.):
          acceleration = max(self.max_decel, acceleration)
          brake = -acceleration * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius


        steering = self.yaw_controller.get_steering(twist_cmd.linear.x, twist_cmd.angular.z, current_velocity.linear.x) + self.steer_kp * (twist_cmd.angular.z - current_velocity.angular.z);

        # take care of both extremes in steering Right or left
        steering = min(steering, self.max_steering)
        steering = max(steering, -self.max_steering)

        return  throttle, brake, steering
