#!/usr/bin/env python
# Author : Esraa Magdy

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point ,PoseArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospkg
from ford_msgs.msg import Clusters
class AgentsStates:
    def __init__(self):
        self.node_name = rospy.get_name()
        # Re-publishing Robot's states in the format needed for the network
        self.odom_sub  = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pose_pub  = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10) 
        self.vel_pub = rospy.Publisher('/robot_velocity',Vector3, queue_size=10)
        # Re-publishing Pedestrians states in the format needed for the network
        self.ped_sub = rospy.Subscriber(
            '/spencer/perception_internal/people_detection/rgbd_front_top/upper_body_detector/bounding_box_centres',
            PoseArray,self.peds_callback)
        self.clusters_pub = rospy.Publisher('cluster/output/clusters',Clusters,queue_size=1)
        #publish goal 
        self.goal_pub = rospy.Publisher('/JA01/move_base_simple/goal',PoseStamped,queue_size=1)
    def odom_callback(self,msg):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = msg.pose.pose.position.x
        pose.pose.position.y = msg.pose.pose.position.y
        pose.pose.position.z = msg.pose.pose.position.z
        pose.pose.orientation.x = msg.pose.pose.orientation.x
        pose.pose.orientation.y = msg.pose.pose.orientation.y
        pose.pose.orientation.z = msg.pose.pose.orientation.z
        pose.pose.orientation.w = msg.pose.pose.orientation.w 
        self.pose_pub.publish(pose)
        
        vel = Vector3()
        vel.x = msg.twist.twist.linear.x
        vel.y = msg.twist.twist.linear.y
        vel.z = 0
        self.vel_pub.publish(vel)

    def peds_callback(self,peds_msg):
        peds_cluster = Clusters()
        peds_cluster.header.stamp = rospy.Time.now()
        peds_cluster.header.frame_id = peds_msg.header.frame_id
        ped_vel = Vector3()
        ped_vel.x = 0.4
        ped_vel.y = 0.4
        ped_vel.z = 0.0
       
        print(len(peds_msg.poses))
        
        if len(peds_msg.poses) > 0 :

            for i in range(len(peds_msg.poses)):
                peds_cluster.mean_points = peds_msg.poses[i].position
                peds_cluster.velocities.append(ped_vel)
                peds_cluster.labels.append(i)

        self.clusters_pub.publish(peds_cluster)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = 5.0
        goal.pose.position.y = 4.0
        self.goal_pub.publish(goal) 
       
if __name__ == '__main__':
    rospy.init_node('agents_states', anonymous=False)
    robot = AgentsStates()
    rospy.spin()