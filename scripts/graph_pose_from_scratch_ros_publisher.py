# ref: https://github.com/goldbattle/simple_2d_slam/
import g2o_file_reader
from load_2d_g2o import load_2d_g2o
import os
import math
import numpy as np
import sys
import rospy
from nav_msgs.msg import Odometry
import tf

# Initialize ROS node
rospy.init_node('pose_graph_publisher')

odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()
map_broadcaster = tf.TransformBroadcaster()

np.set_printoptions(suppress=True, formatter={'float_kind': '{:0.4f}'.format}, threshold=sys.maxsize)

def getNode(nodes, id):
    for node in nodes:
        if node["id"] == id:
            return node

def wrap2pi(theta):
    while theta > math.pi:
        theta = theta - 2*math.pi
    while theta < -math.pi:
        theta = theta + 2*math.pi
    return theta

def rot2(theta):
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return R

def update_node(node, dx):
    node["state"][0] += dx[0]
    node["state"][1] += dx[1]
    node["state"][2] = wrap2pi(node["state"][2] + dx[2])
    return node

# Load g2o file with pose graph
g2o_file = os.path.abspath("/home/behnam/workspace/robotic_notes/data/slam/input_MITb_g2o.g2o")
nodes, edges = load_2d_g2o(filename=g2o_file)

# Initial pose graph plotting (before optimization)
x_cords = [node["state"][0] for node in nodes]
y_cords = [node["state"][1] for node in nodes]

# Helper function to publish odometry for consecutive nodes
def publish_odometry(node1, node2):
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    
    # Set pose from the optimized result of node2 relative to node1
    odom_msg.pose.pose.position.x = node2["state"][0]
    odom_msg.pose.pose.position.y = node2["state"][1]
    odom_msg.pose.pose.orientation.z = node2["state"][2]
    
    # Publish odometry
    odom_pub.publish(odom_msg)
    
    # Publish transform from odom to base_link
    odom_broadcaster.sendTransform(
        (node2["state"][0], node2["state"][1], 0),
        tf.transformations.quaternion_from_euler(0, 0, node2["state"][2]),
        rospy.Time.now(),
        "base_link",  # The robot's body frame
        "odom"        # Odom frame
    )

# Helper function to publish map transform for optimized nodes
def publish_map_transform(node):
    map_broadcaster.sendTransform(
        (node["state"][0], node["state"][1], 0),
        tf.transformations.quaternion_from_euler(0, 0, node["state"][2]),
        rospy.Time.now(),
        "odom",  # The odom frame should be linked to the map
        "map"    # The global map frame
    )

# Publish transform between map and odom frames
def publish_map_to_odom_transform():
    map_broadcaster.sendTransform(
        (0, 0, 0),
        tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time.now(),
        "odom",  # Child frame (odom)
        "map"    # Parent frame (map)
    )

# Optimization loop
dx_norm = np.inf
iteration = 0
while dx_norm > 1e-2:
    b = np.zeros([3*len(x_cords), 1])
    H = np.zeros([3*len(x_cords), 3*len(x_cords)])

    for edge in edges:
        id1 = edge["id1"]
        id2 = edge["id2"]

        node1 = getNode(nodes, id1)
        node2 = getNode(nodes, id2)

        x1 = node1["state"][0]
        y1 = node1["state"][1]
        theta1 = node1["state"][2]

        x2 = node2["state"][0]
        y2 = node2["state"][1]
        theta2 = node2["state"][2]

        theta2_in_1 = wrap2pi(theta2 - theta1)

        err_theta = wrap2pi(wrap2pi(edge["meas"][2]) - theta2_in_1)

        R_1toG = rot2(theta1)
        p_1inG = np.array([[x1], [y1]])
        p_2inG = np.array([[x2], [y2]])

        p_2in1 = R_1toG.T @ (p_2inG - p_1inG)
        p_2in1_from_measurement = np.array([[edge["meas"][0]], [edge["meas"][1]]])

        err_pos_x, err_pos_y = p_2in1_from_measurement - p_2in1
        err_pos_x = err_pos_x.squeeze()
        err_pos_y = err_pos_y.squeeze()

        info = np.array(edge["info"])

        A_i_j = np.array([[-np.cos(theta1), -np.sin(theta1), -np.sin(theta1)*(x2-x1) + np.cos(theta1)*(y2-y1)],
                          [np.sin(theta1), -np.cos(theta1), -np.cos(theta1)*(x2-x1) - np.sin(theta1)*(y2-y1)],
                          [0, 0, -1]])

        B_i_j = np.array([[np.cos(theta1), np.sin(theta1), 0],
                          [-np.sin(theta1), np.cos(theta1), 0], [0, 0, 1]])

        H[3*id1:3*id1+3, 3*id1:3*id1+3] += A_i_j.T @ info @ A_i_j
        H[3*id1:3*id1+3, 3*id2:3*id2+3] += A_i_j.T @ info @ B_i_j
        H[3*id2:3*id2+3, 3*id1:3*id1+3] += B_i_j.T @ info @ A_i_j
        H[3*id2:3*id2+3, 3*id2:3*id2+3] += B_i_j.T @ info @ B_i_j

        b[3*id1:3*id1+3, 0] += A_i_j.T @ info @ np.array([[err_pos_x], [err_pos_y], [err_theta]]).squeeze()
        b[3*id2:3*id2+3, 0] += B_i_j.T @ info @ np.array([[err_pos_x], [err_pos_y], [err_theta]]).squeeze()

    H[0:3, 0:3] += 1e6 * np.eye(3)
    x = np.linalg.solve(H, b)

    dx_norm = np.linalg.norm(x)
    iteration += 1
    print(f'Iteration {iteration} with delta = {dx_norm}')

    for index, node in enumerate(nodes):
        node = update_node(node, x[3*index:3*index+3, 0])

    # Publish odometry during the optimization process
    for i in range(len(nodes) - 1):
        publish_odometry(nodes[i], nodes[i+1])

# After optimization, publish the final map transform
for node in nodes:
    publish_map_transform(node)

# Continuously broadcast the map to odom transform
rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    publish_map_to_odom_transform()
    rate.sleep()

