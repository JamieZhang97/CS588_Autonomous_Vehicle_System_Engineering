import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# note: all the headings are in degrees rather than radian

def get_middle_point(c1, c2):
    '''
    get the middle point of line segment c1 to c2
    '''
    dest = [(c1[0] + c2[0]) / 2, (c1[1] + c2[1]) / 2]
    return dest

def get_radius(x, y, m, n):
    # get the radius of the circle
    return np.sqrt((x - m)**2 + (y - n)**2)

def get_trajactory_first_segment(centroid1, centroid2, min_margin=None):
    '''
    params:
    centroid1, centroid2: two centroids of the detected objects from lidar god's code
    function to compute the trajactory
    return: pandas dataframe (5cm space on x between datapoint)
    '''
    # the destination point is the middle of two centorids
    m, n = get_middle_point(centroid1, centroid2)

    # tangent k1 of destination point
    y_delta = (centroid1[1] - centroid2[1])
    x_delta = (centroid1[0] - centroid2[0])

    y_delta = y_delta + 1e-8 if abs(y_delta) < 1e-8 else y_delta # handle smoothing
    x_delta = x_delta + 1e-8 if abs(x_delta) < 1e-8 else x_delta # handle smoothing

    k0 = y_delta/ x_delta
    k = - 1 / k0

    # plot the perpendicular bisector
    # plt.plot(m, n, 'bo')
    # x = np.arange(1000, 2000)
    # y = lambda x: k * (x - m) + n
    # plt.plot(x, y(x))

    # f(x) = ax^3 + bx^2 + cx + d since original point has tangent = 0 and f(0) = 0, c, d = 0
    # f(x) = ax^3 + bx^2, a = (km - 2n) / m^3, b = (3n - km) / m^2
    a, b = (k * m - 2 * n )/ m**3, (3 * n - k * m) / m**2
    f = lambda x: a * x**3 + b * x**2
    tangent = lambda x: 3 * a * x**2 + 2 * b * x
    return f, tangent

def get_circle(x0, y0, m0, n0, theta0=0, clockwise=True, sec_seg_num_points=120):
    '''
    get the circle trajectory
    x0, y0: start x, y location
    m0, n0: object location (center of the circle)
    '''
    # get the radius of the cardbox
    r = get_radius(x0, y0, m0, n0)

    theta0 = np.rad2deg(np.arctan2((y0 - n0), (x0 - m0)))
    thetas = -(1 + np.arange(sec_seg_num_points)).astype('float64')
    # if the car needs to surround the object in counter clockwise direction, we reverse delta theta
    if not clockwise:
        thetas = -thetas
    thetas *= 360 / sec_seg_num_points
    thetas += theta0
    thetas_rad = np.radians(thetas)
    xs = m0 + r * np.cos(thetas_rad)
    ys = n0 + r * np.sin(thetas_rad)
    tangent = thetas - 90 if clockwise else thetas + 90
    return xs, ys, tangent

def get_trajectory_datapoints(c1, c2, offset_degree=270):
    # config #
    # first_seg_num_points = 20
    # sec_seg_num_points = 20
    # third_seg_num_points = 20

    # first_seg_num_points = 100
    # sec_seg_num_points = 120
    # third_seg_num_points = 120

    total_points = 2000

    first_seg_num_points = int(0.25 * total_points)
    sec_seg_num_points = int(0.5 * total_points)
    third_seg_num_points = int(0.25 * total_points)

    # input centroids here get middle point of the two objects
    m, n = get_middle_point(c1, c2)
    third_seg_length = m
    
    first_seg_step_xs = m / first_seg_num_points

    # get x coordinates for the first curve, add first_seg_step_xs / 2 to force (m, n) to be the end point
    # otherwise the x coordinate of the endpoint would be m - first_seg_step_xs
    first_seg_xs = np.arange(0, m + first_seg_step_xs / 2, first_seg_step_xs)
    
    # get lambda functions of y and k
    first_seg_ys, tangent0 = get_trajactory_first_segment(c1, c2)

    # get actual value of ys and ks
    first_seg_ys = first_seg_ys(first_seg_xs)
    first_seg_headings = np.rad2deg(np.arctan(tangent0(first_seg_xs))) # we use degrees for headings
    first_seg_end_heading = first_seg_headings[-1] # get the end heading degree of first segment

    # plot circle
    r1, r2 = c1[0]**2 + c1[1]**2, c2[0]**2 + c2[1]**2
    if r1 >= r2:
        circle_x, circle_y, circle_headings = get_circle(m, n, c1[0], c1[1], first_seg_end_heading, c1[1] < n, sec_seg_num_points)
    else:
        circle_x, circle_y, circle_headings = get_circle(m, n, c2[0], c2[1], first_seg_end_heading, c2[1] < n, sec_seg_num_points)

    # plot heading direction
    # plt.plot(m, n, 'bo')
    third_seg_step_x = third_seg_length/third_seg_num_points
    third_seg_xs = np.arange(m, m + (third_seg_length * np.cos(np.radians(first_seg_end_heading))), third_seg_step_x)
    third_seg_ys = lambda x: np.tan(np.radians(first_seg_end_heading)) * (x - m) + n
    third_seg_ys = third_seg_ys(third_seg_xs)
    third_seg_headings = (np.ones(len(third_seg_xs)) * first_seg_end_heading)

    xs_merged = np.hstack((first_seg_xs, circle_x, third_seg_xs))
    ys_merged = np.hstack((first_seg_ys, circle_y, third_seg_ys))
    headings_merged = -np.hstack((first_seg_headings, circle_headings, third_seg_headings))
    # correct the initial degree offset
    headings_merged = (headings_merged + offset_degree) % 360

    columns = {
        'x': xs_merged, 
        'y': ys_merged, 
        'heading': headings_merged,
        # 'seg_1_x': xs,
        # 'seg_1_y': ys,
        # 'circle_x': circle_x,
        # 'circle_y': circle_y,
        # 'seg_2_x': x,
        # 'seg_2_y': y,
    }  
    df = pd.DataFrame(columns)

    # ## plot the trajectory ##
    # plt.plot([c1[0], c2[0]], [c1[1], c2[1]])
    # plt.axis('equal')
    # plt.scatter(xs_merged, ys_merged)
    # plt.scatter(circle_x[:20], circle_y[:20])
    # plt.savefig('test.png')

    # debug test place
    # plt.clf()
    # step = 10 / 10
    # xs = np.arange(0, 10.2, 1)
    # ys = xs
    # plt.scatter(xs, ys)
    # plt.savefig('test.png')

    return df

if __name__ == '__main__':
    from get_lidar_points import get_waypoints_main
    ## example usage
    # c1, c2 = [2000, 800], [1000, 600]
    # c1, c2 = [4.439,1.031], [4.498, 0.296]
    # c1, c2 = [3.63240455, 3.044457], [8.72167307, -1.19047339]
    # c1, c2 = [3.63240455,-3.044457], [8.72167307, 1.19047339]

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


    avg_centroids, way_points = get_waypoints_main(as_main=True)
    xs_merged, ys_merged, heading = way_points['x'].values, way_points['y'].values, way_points['heading'].values

    c1, c2 = avg_centroids[0][:2], avg_centroids[1][:2]
    plot_waypoints(way_points, c1, c2)

    df = get_trajectory_datapoints(c1, c2)
    df.to_csv("waypoints.csv")
    print(df)
    # print(df.iloc[:20], df.shape)
    # print(df.iloc[50:101], df.shape)
    # print(df.iloc[40:80])
    # print(df)
