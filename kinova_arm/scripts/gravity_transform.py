#!/usr/bin/python3

import rospy 
import tf
import math
import numpy
from std_msgs.msg import Float64MultiArray

REFERENCE_FRAME = 'j2n6s300_link_base'
END_EFFECTOR_FRAME = 'j2n6s300_link_6'

PUBLISHER_TOPIC = '/j2n6s300_driver/hand_gravity_vector'

class Transformer(object):
    def __init__(self, rate = 50):
        try:
            rospy.init_node('transform_listener')
        except:
            pass

        self.listener = tf.TransformListener()
        self.Rate = rospy.Rate(rate)

    def record_and_publish(self):
        message = Float64MultiArray()
        message.data = []

        self.pub = rospy.Publisher(PUBLISHER_TOPIC, Float64MultiArray, queue_size=1)

        while not rospy.is_shutdown():
            try:
                (translation, rotation) = self.listener.lookupTransform(REFERENCE_FRAME, END_EFFECTOR_FRAME, rospy.Time(0))

                rotation_rpy = list(tf.transformations.euler_from_quaternion(rotation))
                rotation_rpy[0] = rotation_rpy[0] + math.pi
                rotation_rpy[2] = rotation_rpy[2] + (math.pi/2)
                rotation_matrix = tf.transformations.euler_matrix(rotation_rpy[0], rotation_rpy[1], rotation_rpy[2])

                gravity_vector = [0, 0, -9.8, 0]
                rotated_gravity_vector = numpy.matmul(gravity_vector, rotation_matrix)

                g_x = rotated_gravity_vector[1]
                g_y = -rotated_gravity_vector[0]
                g_z = rotated_gravity_vector[2]

                message.data = [g_x, g_y, g_z]
                self.pub.publish(message)
                self.Rate.sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            
if __name__ == '__main__':
    t = Transformer()
    
    print("Publishing the dynamic gravity vector for the end effector!")
    t.record_and_publish()
