#!/usr/bin/env python
# Author : Esraa Magdy

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point ,PoseArray , TransformStamped, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospkg
from ford_msgs.msg import Clusters
from spencer_tracking_msgs.msg import DetectedPersons
from tf import TransformListener 
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32, ColorRGBA, Int32
class AgentsStates:
    def __init__(self):
        self.node_name = rospy.get_name()
        self.tf_listener = TransformListener()
        # Re-publishing Robot's states in the format needed for the network
        self.odom_sub  = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pose_pub  = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10) 
        self.vel_pub = rospy.Publisher('/robot_velocity',Vector3, queue_size=10)
        # Re-publishing Pedestrians states in the format needed for the network
        self.ped_sub = rospy.Subscriber(
            '/spencer/perception_internal/detected_person_association/lasers_upper_body_fused',
            DetectedPersons,self.peds_callback)
        self.clusters_pub = rospy.Publisher('cluster/output/clusters',Clusters,queue_size=1)
        #self.pub_agent_markers = rospy.Publisher('~agent_markers',MarkerArray,queue_size=1)
        #publish goal 
        self.goal_pub = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=1)
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
        peds_cluster.header.frame_id = "map"
        peds_cluster.labels =[]
        peds_cluster.counts =[]
        peds_cluster.mean_points=[]
        peds_cluster.max_points=[]
        peds_cluster.min_points=[]
        peds_cluster.pointclouds=[]
        peds_cluster.velocities=[]
        peds_cluster.num.data = int(len(peds_msg.detections))
        
        #markers = MarkerArray()
        
        #print(len(peds_msg.detections))
        
        if len(peds_msg.detections) > 0 :
            for i in range(len(peds_msg.detections)):
                
                t = self.tf_listener.getLatestCommonTime("/map", "/base_footprint")
                pose_in_baseframe = PointStamped()
                pose_in_baseframe.header.frame_id = "base_footprint"
                pose_in_baseframe.header.stamp = rospy.Time.now()
                pose_in_baseframe.point = peds_msg.detections[i].pose.pose.position
                pose_in_map = PointStamped()
                pose_in_map.header.frame_id = "map"
                pose_in_map.header.stamp = rospy.Time.now()


                pose_in_map = self.tf_listener.transformPoint("/map", pose_in_baseframe)
                peds_cluster.mean_points.append(pose_in_map.point)
                ped_vel = Vector3()
                ped_vel.x = 0.4
                ped_vel.y = 0.4
                ped_vel.z = 0.0
                peds_cluster.velocities.append(ped_vel)
                peds_cluster.labels.append(i)
                '''
                marker = Marker()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = 'map'
                marker.ns = 'other_agent'
                marker.id = i
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.pose.position.x = pose_in_map.point.x
                marker.pose.position.y = pose_in_map.point.y
                # marker.pose.orientation = orientation
                marker.scale = Vector3(x=0.2,y=0.2,z=0.2)
                marker.color = ColorRGBA(r=1.0,g=0.4,a=1.0)
                marker.lifetime = rospy.Duration(0.1)
                markers.markers.append(marker)
                '''

            self.clusters_pub.publish(peds_cluster)
            #self.pub_agent_markers.publish(markers)

        
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = 5.0
        goal.pose.position.y = 0.0
        self.goal_pub.publish(goal) 
        
       
if __name__ == '__main__':
    rospy.init_node('agents_states', anonymous=False)
    robot = AgentsStates()
    rospy.spin()