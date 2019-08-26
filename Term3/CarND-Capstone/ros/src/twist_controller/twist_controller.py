import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# An Object instance of this 'Controller Class' is initialised in dbw_node.py:  DBWNode--> def __init__ constructor: self.controller = Controller(......)
# The actual control function is called @rate 50Hz             in dbw_node.py:  DBWNode--> def loop        function: self.controller.control(......)
class Controller(object):
    def __init__(self,
                 vehicle_mass,
                 fuel_capacity ,
                 brake_deadband ,
                 decel_limit ,
                 accel_limit ,
                 wheel_radius ,
                 wheel_base ,
                 steer_ratio ,
                 max_lat_accel ,
                 max_steer_angle):

        self.vehicle_mass    = vehicle_mass
        self.fuel_capacity   = fuel_capacity
        self.brake_deadband  = brake_deadband
        self.decel_limit     = decel_limit   # these come out as ros params in db-node.py
        self.accel_limit     = accel_limit
        self.wheel_radius    = wheel_radius

        # TODO: Implement
        # Controller 1: Steering Yaw controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        # Controller 2: PID Controller
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0.  #Minimum throttle value
        mx = 0.2 #Maximum throttle value
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau          = 0.5  # 1/(2pi*tau) = cutoff frequency
        ts           = 0.02 # Sample Time
        # The low pass filter is used because the velocity that is coming over the messages is noisy
        # The low pass filter filters out the high frequency noise in the velocity
        self.vel_lpf = LowPassFilter(tau, ts)



        self.last_time       = rospy.get_time()

    # The actual control function is called @rate 50Hz in DBWNode--> def loop function: self.controller.control(......)
    def control(self, current_vel, dbw_enabled, linear_vel,angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # If dbw is off we dont want to accumulate the error ( in the integral term of the pid controller. The throttle controller is a PID controller)
        # So if you accumulate all this error when the dbw is off, and then turn on the dbw, this error might cause the car to do something erratic
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0., 0.

        current_vel = self.vel_lpf.filt(current_vel)

        # rospy.logwarn("Angular Velocity: {0}".format(angular_vel))
        # rospy.logwarn("Target Velocity: {0}".format(linear_vel))
        # rospy.logwarn("Target Angular Velocity: {0}".format(angular_vel))
        # rospy.logwarn("Current Velocity: {0}".format(current_vel))
        # rospy.logwarn("Filtered Velocity: {0}".format(self.vel_lpf.get()))

        steering  = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error     = linear_vel - current_vel
        self.last_vel = current_vel

        current_time   = rospy.get_time()
        sample_time    = current_time - self.last_time
        self.last_time = current_time

        #Physics for the throttle part
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake    = 0

        if linear_vel == 0. and  current_vel < 0.1:
            throttle = 0
            brake    = 400 #N*m - to hold the car in place if we are stopped at a light. Acceleration - 1m/s^2

        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Torque N*m

        return throttle, brake, steering
