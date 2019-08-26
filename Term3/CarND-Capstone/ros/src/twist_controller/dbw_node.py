#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass    = rospy.get_param('~vehicle_mass', 1736.35) #Need this for brake torque (Nm)
        fuel_capacity   = rospy.get_param('~fuel_capacity', 13.5) #this is not required for the simulator lol
        brake_deadband  = rospy.get_param('~brake_deadband', .1)
        decel_limit     = rospy.get_param('~decel_limit', -5)
        accel_limit     = rospy.get_param('~accel_limit', 1.)
        wheel_radius    = rospy.get_param('~wheel_radius', 0.2413) #Need this for brake torque (Nm)
        wheel_base      = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio     = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel   = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)



        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool,         self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd',           TwistStamped, self.twist_cb)
        rospy.Subscriber('/current_velocity',    TwistStamped, self.velocity_cb)

        #The following are initialised to None here: But these will be set later in the Subscriber Callbacks

        # So if you want your car to just drive in a straight line you can set Throttle to 1, Steering and Break to 0 . Publish throttle
        # Unlike Break, throttle is in proportion from zero to one
        # so instead of using the controller, you could also hard code these values and publish them
        self.dbw_enabled  = None
        self.linear_vel   = None
        self.angular_vel  = None
        self.current_vel  = None
        self.curr_ang_vel = None
        self.throttle     = self.steering = self.brake = 0

        #Declare Publishers
        self.steer_pub    = rospy.Publisher('/vehicle/steering_cmd',  SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub    = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object: You are only initialising/instantiating the Controller object and using a constructor to initialise the values
        # The actual controller is called in the loop (since this is the init function, you initialise the controller too)
        self.controller = Controller(vehicle_mass    = vehicle_mass,
                                     fuel_capacity   = fuel_capacity,
                                     brake_deadband  = brake_deadband,
                                     decel_limit     = decel_limit,
                                     accel_limit     = accel_limit,
                                     wheel_radius    = wheel_radius,
                                     wheel_base      = wheel_base,
                                     steer_ratio     = steer_ratio,
                                     max_lat_accel   = max_lat_accel,
                                     max_steer_angle = max_steer_angle)

        self.loop()

    #3 subscribers , 3 callbacks
    def dbw_enabled_cb(self,msg):
        self.dbw_enabled = msg

    def twist_cb(self, msg):
        self.linear_vel  = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z

    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x

    def loop(self):
        # 50Hz : extremely important to be at 50Hz.Drive by wire needs inputs @rate 50Hz. If it is below 20Hz  so it will shut off (safety feature)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            if not None in (self.current_vel, self.linear_vel, self.angular_vel):
                # We import the controller from twist_controller.py and use that to obtain throttle, brake and steering
                # The controller will provide with throttle, brake and steering , if you provide it with dbw_enabled (or not), current_vel,  linear and angular velocities
                # dbw_enabled (or not), current_vel,  linear and angular velocities are initialised to None in def __init__ but the values come from the callback functions
                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,
                                                                                   self.dbw_enabled,
                                                                                   self.linear_vel,
                                                                                   self.angular_vel)
            if self.dbw_enabled:
                self.publish(self.throttle, self.brake, self.steering)
            rate.sleep()


    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
