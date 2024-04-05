#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry##

        #self.fig = plt.figure()
xs = []
ys = []
rads = []
x_num = []
xs_num =0

actual_pose = Pose()


_actual_pose = rospy.Subscriber("/base_pose_ground_truth", Odometry,_actual_pose_callback,queue_size=1)
_estimated_pose = rospy.Subscriber("/amcl_pose", Odometry,amcl_pose_callback,queue_size=1)

def _actual_pose_callback(message):
    acutal_pose = message.pose.pose



def amcl_pose_callback(message):


    _x_off= abs(1 - actual_pose.position.x/(message.pose.pose.position.x))*100
    _y_off= abs(1 - actual_pose.position.y/(message.pose.pose.position.y))*100
    _rad_off = abs(1-(getHeading(actual_pose.orientation)/getHeading(message.pose.pose.orientation)))*100

    if(xs_num < 350):
        xs_num = xs_num +1
        x_num.append(xs_num)
        xs.append(_y_off)
        ys.append(_x_off)
        rads.append(_rad_off)
        rospy.loginfo(xs_num)

    elif(xs_num > 150):

        rospy.loginfo("plotting")
        plt.plot(x_num,xs, label = "x percent off")
        plt.plot(x_num,ys, label = "y percent off")
        plt.ylim(0,100)
        plt.show()
        plt.plot(x_num,rads, label = "rad percent off")
        plt.ylim(0,100)
        plt.show()
    #rospy.loginfo(self.actual_pose)
    rospy.loginfo("x percent off: " + str(_x_off) + "; y percent off: " + str(_y_off) + ";randian percent off: " + str(_rad_off))



if __name__ == "__main__":
    rospy.init_node("test_publisher")
    rospy.loginfo("leech")
    rospy.spin()
