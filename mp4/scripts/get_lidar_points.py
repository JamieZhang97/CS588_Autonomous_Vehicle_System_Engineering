#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointCloud, ChannelFloat32
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from planner import get_trajectory_datapoints

### === k-means === ###

def initialize_centroids(data, k):
    return data[np.random.choice(data.shape[0], k, replace=False)]

def assign_clusters(data, centroids):
    distances = np.linalg.norm(data - centroids[:, np.newaxis], axis=2)
    return np.argmin(distances, axis=0)

def update_centroids(data, clusters, k):
    new_centroids = np.array([data[clusters == i].mean(axis=0) for i in range(k)])
    return new_centroids

def kmeans(data, k, max_iterations=100, tol=1e-4):
    centroids = initialize_centroids(data, k)
    prev_centroids = centroids.copy()
    
    for _ in range(max_iterations):
        clusters = assign_clusters(data, centroids)
        centroids = update_centroids(data, clusters, k)
        
        if np.linalg.norm(centroids - prev_centroids) < tol:
            break
            
        prev_centroids = centroids.copy()
        
    return centroids, clusters




### === main function === ###
def get_waypoints_main(as_main = True):

    ### === configs === ###
    k = 2 # number of clusters

    xb = [0,15] # x bounds
    yb = [-5,7] # y bounds
    zb = [-1.2,0.5] # z bounds

    xmin = 0.6 # minimum x absolute value (only consider poins with abs(x) > xmin)
    ymin = 0.6 # minimum y absolute value (only consider poins with abs(y) > ymin)
    zmin = 0.6 # minimum z absolute value (only consider poins with abs(z) > zmin)

    if_plot = False # whether to plot the points
    if_keep_publishing = False # whether to keep publishing the centroids
    
    sufficient_time_steps = 20 # avg over first 20 time steps

    ### === plotting === ###
    if if_plot:
        fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        ax = fig.add_subplot(111)
        
        # Set the axes limits
        ax.set_xlim([xb[0], xb[1]])
        ax.set_ylim([yb[0], yb[1]])
        # ax.set_zlim([zb[0], zb[1]])

        # set the axes labels
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        # ax.set_zlabel('z')

    ### === callback === ###
    def lidar_callback(msg):
        nonlocal condition_met
        xs = []
        ys = []
        zs = []
        for point in pc2.read_points(msg, skip_nans=True):
            x, y, z = point[:3]
            if xb[0]<x<xb[1] and yb[0]<y<yb[1] and zb[0]<z<zb[1]:
                if abs(x) > xmin and abs(y) > ymin and abs(z) > zmin: 
                    xs.append(x)
                    ys.append(y)
                    zs.append(z)
        points = np.array(list(zip(xs, ys, zs)))
        centroids, clusters = kmeans(points, k)
        # print("centroids", centroids)
        # print("clusters", clusters)
        
        if if_keep_publishing:
            # Create a PointCloud message to publish the centroids
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'lidar_frame'
            pcl_centroids = PointCloud()
            pcl_centroids.header = header
            pcl_centroids.points = [Point32(x=c[0], y=c[1], z=c[2]) for c in centroids]
            pcl_centroids.channels = []

            # Create a PointCloud message to publish the raw points
            pcl_points = PointCloud()
            pcl_points.header = header
            pcl_points.points = [Point32(x=p[0], y=p[1], z=p[2]) for p in points]
            pcl_points.channels = [ChannelFloat32(name='clusters', values=clusters)]
            assert len(points) == len(clusters)

            # Publish the PointCloud message
            pub_centroids.publish(pcl_centroids)
            pub_points.publish(pcl_points)
        else:
            # print(centroids)
            centroids_all_time_steps.append(centroids)
            # print(len(centroids_all_time_steps), "centroids_all_time_steps", centroids_all_time_steps)
            if len(centroids_all_time_steps) >= sufficient_time_steps:
                condition_met = True

        # plot the centroids in the figure
        if if_plot:
            # ax.scatter(centroids[:,0], centroids[:,1], centroids[:,2], s=10, c='r', marker='o')
            ax.scatter(centroids[:,0], centroids[:,1], s=10, c='r', marker='o')

            # ax.scatter(xs, ys, zs, s=0.1, c='b', marker='.')
            ax.scatter(xs, ys, s=0.1, c='b', marker='.')            
            plt.savefig('lidar_points.png')

    # initialize the list for storing centroids from all time steps
    centroids_all_time_steps = []

    ### init nodes ###
    if as_main:
        rospy.init_node('lidar_node')
    # publish the centroids x,y,z coordinates (.points)
    pub_centroids = rospy.Publisher('/lidar_processed/centroids', PointCloud, queue_size=10) 
    # publish the raw points x,y,z coordinates (.points) and their cluster assignments (.cahnnels['clusters'])
        # Note: the cluster assignment is converted to float due to the requirement of the ChannelFloat32 message type
        #      you can convert it back to int to indicate the cluster assignment
    pub_points = rospy.Publisher('/lidar_processed/points', PointCloud, queue_size=10) 
    sub = rospy.Subscriber('/lidar1/velodyne_points', PointCloud2, lidar_callback)
    
    # early stop for getting the two clusters 
    condition_met = False

    ## using custom loop to stop spin##
    print("start spinning...")
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown() and not condition_met:
        rate.sleep()
    print(f"out of spin loop after receiving centroids for {sufficient_time_steps} time steps")
    # get the average centroids from ten time steps
    print(centroids_all_time_steps)
    # avg_centroids = np.mean(centroids_all_time_steps, axis=0)
    avg_centroids = centroids_all_time_steps[0] #TODO: fix this
    assert len(avg_centroids) == k


    # # get the way points
    c1, c2 = avg_centroids[0][:2], avg_centroids[1][:2]
    print("avg_centroids:", avg_centroids)
    print("c1, c2:", c1, c2)
    way_points = get_trajectory_datapoints(c1, c2)
    return avg_centroids, way_points

if __name__ == '__main__':

    def plot_waypoints(way_points, c1, c2):
        xs_merged, ys_merged = way_points['x'].values, way_points['y'].values
        # seg_1_x, seg_1_y = way_points['seg_1_x'].values, way_points['seg_1_y'].values
        # circle_x, circle_y = way_points['circle_x'].values, way_points['circle_y'].values
        # seg_2_x, seg_2_y = way_points['seg_2_x'].values, way_points['seg_2_y'].values
        plt.plot([c1[0], c2[0]], [c1[1], c2[1]])
        plt.axis('equal')
        plt.scatter(xs_merged, ys_merged, s=0.5, c='b', marker='.')
        # plt.plot(seg_1_x, seg_1_y)
        # plt.plot(circle_x, circle_y)
        # plt.plot(seg_2_x, seg_2_y)
        plt.savefig('visualize_waypoints.png')

    ### usage example ###
    condition_met = False
    avg_centroids, way_points = get_waypoints_main(as_main=True) # returns avg_centroids and a pandas dataframe containing way points and tengents
    # To get the way points x and y coordinates, you can use the following code:
        # xs_merged, ys_merged = way_points['x'].values, way_points['y'].values

    print("avg_centroids:", avg_centroids)
    print(way_points.head(), way_points.shape)


    # plot_waypoints(way_points, avg_centroids[0][:2], avg_centroids[1][:2])