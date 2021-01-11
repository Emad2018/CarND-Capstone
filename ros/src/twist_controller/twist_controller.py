import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base,
                 steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        kp = .3
        ki = .001
        kd = 2
        mn = 0
        mx = .2
        self.throttle_control = PID(kp, ki, kd, mn, mx)

        tau = .5
        ts = .02
        self.vel_lo_pass_fillter = LowPassFilter(tau, ts)

        self.veh_mass_ = vehicle_mass
        self.wheel_radius_ = wheel_radius
        self.decel_limit_ = decel_limit
        self.last_vel = None
        self.prev_time = rospy.get_time()

        self.yaw_control = YawController(
            wheel_base, steer_ratio, .1, max_lat_accel, max_steer_angle)

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_status):

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_status:
            self.throttle_control.reset()
            return 0, 0, 0
        current_velocity = self.vel_lo_pass_fillter.filt(current_velocity)
        steering = self.yaw_control.get_steering(
            linear_velocity, angular_velocity, current_velocity)

        vel_error = linear_velocity-current_velocity
        self.last_vel = current_velocity

        current_time = rospy.get_time()
        sample_time = current_time-self.prev_time
        self.prev_time = current_time

        throttle = self.throttle_control.step(vel_error, sample_time)
        brake = 0

        if ((not linear_velocity) and (current_velocity < .1)):
            throttle = 0
            brake = 400
        elif ((throttle < .1) and (vel_error < 0)):

            throttle = 0
            brake = abs(max(vel_error, self.decel_limit_)) * \
                self.veh_mass_ * self.wheel_radius_

        return throttle, brake, steering
