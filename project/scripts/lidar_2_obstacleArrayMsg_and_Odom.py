# """
# Prerequisite: install the costmap_converter package:

# - Clone the `costmap_converter` package into your workspace's `src` directory using the following command:
#    ```
#    git clone https://github.com/ros-agriculture/costmap_converter.git
#    ```
# - Install any dependencies required by the `costmap_converter` package. You can use the following command to install all dependencies:
#    ```
#    rosdep install --from-/via_points s src --ignore-src -r -y
#    ```
# - From catkin workspace root, build the package using the following command:
#    ```
#    catkin_make
#    ```
# - Source the `setup.bash` file in your workspace's `devel` directory:
#    ```
#    source devel/setup.bash
#    ```
# """


#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32, Polygon, PoseStamped
from novatel_gps_msgs.msg import Inspva
from nav_msgs.msg import Odometry, Path
from tf.transformations import quaternion_from_euler
import alvinxy.alvinxy as axy
import matplotlib.pyplot as plt
import random


# macros: lidar constraints
xb = [0,15] # x bounds
yb = [-6,6] # y bounds
zb = [-1.25,0.5] # z bounds

xmin = 0.6 # minimum x absolute value (only consider poins with abs(x) > xmin)
ymin = 0.6 # minimum y absolute value (only consider poins with abs(y) > ymin)
zmin = 0.6 # minimum z absolute value (only consider poins with abs(z) > zmin)


# def wps_to_local_xy(lon_wp, lat_wp, olat = 40.0928042, olon = -88.2356915):
def wps_to_local_xy(lon_wp, lat_wp, olat = 40.0928233, olon = -88.2355777):
    # convert GNSS waypoints into local fixed frame reprented in x and y
    lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, olat, olon)
    return lon_wp_x, lat_wp_y  


def point_cloud_callback(msg):
    # Process point cloud data and create ObstacleArrayMsg
    obstacles = ObstacleArrayMsg()
    obstacles.header = msg.header

    obstacle_xs = []
    obstacle_ys = []

    threshold = 1
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point

        # filter groud points
        if xb[0]<x<xb[1] and yb[0]<y<yb[1] and zb[0]<z<zb[1]:
            if abs(x) > xmin and abs(y) > ymin and abs(z) > zmin:
                
                if random.randint(0,10) < threshold:
                    continue

                # Create an ObstacleMsg
                obstacle = ObstacleMsg()
                
                obstacle.radius = 1

                obstacle.header = msg.header

                # Create a polygon with a single point (x, y)
                polygon = Polygon()
                # polygon.header = msg.header
                # polygon.polygon = Polygon()
                polygon.points = [Point32(x, y, 0)]

                # Add the polygon to the obstacle
                obstacle.polygon = polygon
                obstacles.obstacles.append(obstacle)

                # for plotting
                obstacle_xs.append(x)
                obstacle_ys.append(y)

    plt.xlim(-2,16)
    plt.ylim(-5,5)
    plt.scatter(obstacle_xs, obstacle_ys, c="black", s=50)
    plt.savefig("obstacles.png")
    plt.clf()


    # Publish the obstacle array
    obstacle_pub.publish(obstacles)
    print("len(obstacles):", len(obstacles.obstacles))

def inspva_callback(msg):
    # Convert the Inspva message to an Odometry message
    odom = Odometry()
    odom.header.stamp = msg.header.stamp
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    # Set position (latitude, longitude, altitude)
    local_x_curr, local_y_curr = wps_to_local_xy(msg.longitude, msg.latitude)
    # print("local_x_curr, local_y_curr:",local_x_curr, local_y_curr)
    odom.pose.pose.position.x = local_x_curr
    odom.pose.pose.position.y = local_y_curr
    
    # odom.pose.pose.position.x = msg.latitude
    # odom.pose.pose.position.y = msg.longitude
    # odom.pose.pose.position.z = msg.height


    # Set orientation (roll, pitch, yaw)
    roll, pitch, yaw = msg.roll * 0.0174533, msg.pitch * 0.0174533, msg.azimuth * 0.0174533
    q = quaternion_from_euler(roll, pitch, yaw)
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]
    # print("yaw",yaw)

    # Publish the Odometry message
    odom_pub.publish(odom)

def _publish_via_points(start_x=0, start_y=0, goal_x=-15, goal_y=3):

    via_path = Path()
    via_path.header.stamp = rospy.Time.now()
    via_path.header.frame_id = "odom" # CHANGE HERE: odom/map
 
    via_path.poses = [PoseStamped(), PoseStamped()]
    via_path.poses[0].pose.position.x = start_x
    via_path.poses[0].pose.position.y = start_y
    via_path.poses[0].pose.position.z = 0

    # via_path.poses[0].pose.orientation.x = 0
    # via_path.poses[0].pose.orientation.y = 0
    # via_path.poses[0].pose.orientation.z = 0
    # via_path.poses[0].pose.orientation.w = 1.0

    via_path.poses[1].pose.position.x = goal_x
    via_path.poses[1].pose.position.y = goal_y
    via_path.poses[1].pose.position.z = 0

    # via_path.poses[1].pose.orientation.x = 0
    # via_path.poses[1].pose.orientation.y = 0
    # via_path.poses[1].pose.orientation.z = 0
    # via_path.poses[1].pose.orientation.w = 1.0

    via_point_pub.publish(via_path)

def publish_via_points_callback(msg):
    _publish_via_points(start_x=0, start_y=0, goal_x=15, goal_y=3)


if __name__ == "__main__":
    rospy.init_node("velodyne_to_teb_obstacles")

    # Subscribe to Velodyne point cloud data
    rospy.Subscriber("/lidar1/velodyne_points", PointCloud2, point_cloud_callback)

    # Create a publisher for the obstacles topic
    obstacle_pub = rospy.Publisher("/test_optim_node/obstacles", ObstacleArrayMsg, queue_size=1)

    # Create a publisher for the via points topic
    via_point_pub = rospy.Publisher("/test_optim_node/via_points", Path, queue_size=1)

    # Subscribe to INS position and velocity data
    rospy.Subscriber('/novatel/inspva', Inspva, inspva_callback)

    # Create publisher for the Odometry message
    odom_pub = rospy.Publisher('/test_optim_node/odom', Odometry, queue_size=10)

    # publish the via_points
    rospy.Timer(rospy.Duration(1/10.0), publish_via_points_callback)

    rospy.spin()