import roslib
roslib.load_manifest('mapping_tools')
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
from nav_msgs.msg import Odometry
import tf



#def amcl_callback(data):
 #   global amcl_pub
  #  publish(data, amcl_pub)
  #   pose = PoseStamped()
  #   pose.pose = data.pose.pose
  #   pose.header = data.header
  #   pub.publish(pose)

#def ekf_callback(data):
 #   global


# def slam_callback(data):
#     global slam_pub
#     #data = PoseStamped()
#     odom = Odometry()
#     odom.header = data.header
#     odom.header.frame_id = "odom"
#     odom.pose.pose = data.pose
#     slam_pub.publish(odom)


rospy.init_node('pose_publisher', anonymous=False)


#rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback)
#rospy.Subscriber("/slam_out_pose", PoseStamped, slam_callback)

#amcl_pub = rospy.Publisher("~amcl_pose", PoseStamped, queue_size=10)
#ekf_pub = rospy.Publisher("~ekf_pose", PoseStamped, queue_size=10)
slam_pub = rospy.Publisher("~slam_odom", Odometry, queue_size=10)


listener = tf.TransformListener()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    try:
        trans, rot = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = trans[0]
        odom.pose.pose.position.y = trans[1]
        odom.pose.pose.position.z = trans[2]
        odom.pose.pose.orientation.x = rot[0]
        odom.pose.pose.orientation.y = rot[1]
        odom.pose.pose.orientation.z = rot[2]
        odom.pose.pose.orientation.w = rot[3]
        slam_pub.publish(odom)
    except:
        continue
    rate.sleep()