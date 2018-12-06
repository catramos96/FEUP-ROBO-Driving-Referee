#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist


class Velocity(object):

    def __init__(self, min_velocity, max_velocity, num_steps):
        assert min_velocity > 0 and max_velocity > 0 and num_steps > 0
        self._min = min_velocity
        self._max = max_velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (max_velocity - min_velocity) / (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step):
        """
        Takes a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """
        if step == 0:
            return 0

        assert step > 0 and step <= self._num_steps
        max_value = self._min + self._step_incr * (step - 1)
        return value * max_value



class EllipseMovement():

    def __init__(self):
        self._pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._hz = rospy.get_param('~hz', 50)
        self._forward_rate = rospy.get_param('~forward_rate', 0.3)
        self._backward_rate = rospy.get_param('~backward_rate', 0.25)
        self._rotation_rate = rospy.get_param('~rotation_rate', 0.25)
        self._angular = 0
        self._linear = 0
        
    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            self._set_velocity()
            self._publish()
            rate.sleep()

    def _set_velocity(self):
        self._angular = 0.2
        self._linear = 0.5
        
    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _publish(self):
        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)
        


def main():
    rospy.init_node('ellipse_movement')
    app = EllipseMovement()
    app.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass