# """
# Prerequisite: install the costmap_converter package:

# - Clone the `costmap_converter` package into your workspace's `src` directory using the following command:
#    ```
#    git clone https://github.com/ros-agriculture/costmap_converter.git
#    ```
# - Install any dependencies required by the `costmap_converter` package. You can use the following command to install all dependencies:
#    ```
#    rosdep install --from-paths src --ignore-src -r -y
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


#!/usr/bin/env python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
# from geometry_msgs.msg import PolygonStamped, Point32, Polygon
from geometry_msgs.msg import Point32, Polygon

def point_cloud_callback(msg):
    # Process point cloud data and create ObstacleArrayMsg
    obstacles = ObstacleArrayMsg()
    obstacles.header = msg.header

    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        x, y, _ = point

        # Create an ObstacleMsg
        obstacle = ObstacleMsg()
        obstacle.header = msg.header

        # Create a polygon with a single point (x, y)
        polygon = Polygon()
        # polygon.header = msg.header
        # polygon.polygon = Polygon()
        polygon.points = [Point32(x, y, 0)]

        # Add the polygon to the obstacle
        obstacle.polygon = polygon
        obstacles.obstacles.append(obstacle)

    # Publish the obstacle array
    obstacle_pub.publish(obstacles)
   #  print(obstacles)

if __name__ == "__main__":
    rospy.init_node("velodyne_to_teb_obstacles")

    # Subscribe to Velodyne point cloud data
    rospy.Subscriber("/lidar1/velodyne_points", PointCloud2, point_cloud_callback)

    # Create a publisher for the obstacles topic
    obstacle_pub = rospy.Publisher("/obstacles", ObstacleArrayMsg, queue_size=1)

    rospy.spin()
