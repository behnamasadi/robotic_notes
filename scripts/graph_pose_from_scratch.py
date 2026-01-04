# ref: https://github.com/goldbattle/simple_2d_slam/


from load_2d_g2o import load_2d_g2o
import os
import matplotlib.pyplot as plt
import math
import numpy as np
import sys
from pathlib import Path

# np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True, formatter={
                    'float_kind': '{:0.4f}'.format}, threshold=sys.maxsize)


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
    R = np.zeros([2, 2])

    R = np.array([[np.cos(theta), -np.sin(theta)],
                 [np.sin(theta), np.cos(theta)]])
    return R


def update_node(node, dx):

    # position is just an addition
    node["state"][0] = node["state"][0] + dx[0]
    node["state"][1] = node["state"][1] + dx[1]

    # add theta, then enforce it is within -pi to pi
    node["state"][2] = wrap2pi(node["state"][2] + dx[2])

    return node


# Get script directory and project root
script_dir = Path(__file__).resolve().parent
project_root = script_dir.parent
data_dir = project_root / "data" / "slam"

# Default g2o file (can be overridden with command-line argument if needed)
# Available files: input_INTEL_g2o.g2o, input_M3500_g2o.g2o, input_MITb_g2o.g2o
g2o_file = data_dir / "input_MITb_g2o.g2o"

# Verify the file exists
if not g2o_file.exists():
    print(f"Error: G2O file not found at: {g2o_file}")
    print(f"Script directory: {script_dir}")
    print(f"Project root: {project_root}")
    print(f"Data directory: {data_dir}")
    print(f"Data directory exists: {data_dir.exists()}")
    if data_dir.exists():
        available_files = list(data_dir.glob("*.g2o"))
        if available_files:
            print(f"Available .g2o files: {[f.name for f in available_files]}")
    sys.exit(1)


nodes, edges = load_2d_g2o(filename=g2o_file)


x_cords = [node["state"][0] for node in nodes]
y_cords = [node["state"][1] for node in nodes]
theta = [node["state"][2] for node in nodes]

plt.plot(x_cords, y_cords, color="blue", label="Before optimization")
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Nodes from g2o file')
plt.grid(True)


# ensure our orientations are bounded between -pi and pi


dx_norm = np.inf
iteration = 0

A_i_j = np.zeros([3, 3])
B_i_j = np.zeros([3, 3])

while dx_norm > 1e-2:
    # for i in [1]:
    # our matrices for Hx=-b
    b = np.zeros([3*len(x_cords), 1])
    H = np.zeros([3*len(x_cords), 3*len(x_cords)])

    for edge in edges:

        # idout
        id1 = edge["id1"]
        # idin
        id2 = edge["id2"]

        node1 = getNode(nodes, id1)
        node2 = getNode(nodes, id2)

        x1 = node1["state"][0]
        y1 = node1["state"][1]
        theta1 = node1["state"][2]

        x2 = node2["state"][0]
        y2 = node2["state"][1]
        theta2 = node2["state"][2]

        theta2_in_1 = wrap2pi(theta2-theta1)

        # orientation error
        err_theta = wrap2pi(wrap2pi(edge["meas"][2]) - theta2_in_1)

        # position error
        R_1toG = rot2(theta1)

        p_1inG = np.array([[x1], [y1]])
        p_2inG = np.array([[x2], [y2]])

        p_2in1 = R_1toG.T @ (p_2inG - p_1inG)

        p_2in1_from_measurement = np.array(
            [[edge["meas"][0]], [edge["meas"][1]]])

        err_pos_x, err_pos_y = p_2in1_from_measurement - p_2in1

        err_pos_x = err_pos_x.squeeze()
        err_pos_y = err_pos_y.squeeze()

        # 3x3d data on the file is I11 I12 I13 I22 I23 I33 but the matrix is symmetric so
        info = np.array(edge["info"])

        # Jacobian of current relative in respect to NODE 1

        A_i_j = np.array([[-np.cos(theta1), -np.sin(theta1), -np.sin(theta1)*(x2-x1) + np.cos(theta1)*(y2-y1)],
                          [np.sin(theta1), -np.cos(theta1), -np.cos(theta1) *
                           (x2-x1) - np.sin(theta1)*(y2-y1)],
                          [0, 0, -1]])

        # print("A_i_j:", A_i_j)

        # Jacobian of current relative in respect to NODE 2

        B_i_j = np.array([[np.cos(theta1), np.sin(theta1), 0],
                          [-np.sin(theta1), np.cos(theta1), 0], [0, 0, 1]])

        # print("B_i_j:", B_i_j)

        # update our information

        # H_i_i
        H[3*id1:3*id1+3, 3*id1:3*id1+3] = H[3*id1:3 *
                                            id1+3, 3*id1:3*id1+3] + A_i_j.T @ info@A_i_j

        # print("H_i_i:", H[3*id1:3*id1+3, 3*id1:3*id1+3])

        # H_i_j
        H[3*id1:3*id1+3, 3*id2:3*id2+3] = H[3*id1:3 *
                                            id1+3, 3*id2:3*id2+3] + A_i_j.T @ info @ B_i_j

        # print("H_i_j:", H[3*id1:3*id1+3, 3*id2:3*id2+3])

        # H_j_i
        H[3*id2:3*id2+3, 3*id1: 3*id1+3] = H[3*id2:3 *
                                             id2+3, 3*id1: 3*id1+3]+B_i_j.T@info@A_i_j

        # print("H_j_i:", H[3*id2:3*id2+3, 3*id1: 3*id1+3])

        # H_j_j
        H[3*id2:3*id2+3, 3*id2: 3*id2+3] = H[3*id2:3 *
                                             id2+3, 3*id2: 3*id2+3]+B_i_j.T@info@B_i_j

        # print("H_j_j:", H[3*id2:3*id2+3, 3*id2: 3*id2+3])

        # update our error terms
        result = A_i_j.T @ info @ np.array([[err_pos_x],
                                            [err_pos_y], [err_theta]])

        # print(result)
        # print("A_i_j:\n", A_i_j)

        result = result.squeeze()

        b[3*id1:3*id1+3, 0] = b[3*id1:3*id1+3, 0] + result

        # print("b1", b[3*id1:3*id1+3, 0])

        result = B_i_j.T @ info @ np.array([[err_pos_x],
                                           [err_pos_y], [err_theta]])

        # print("B_i_j:\n", B_i_j)

        result = result.squeeze()

 #       print(result)

        b[3*id2:3*id2+3, 0] = b[3*id2:3*id2+3, 0] + result

        # print("b2", b[3*id2:3*id2+3, 0])
        # print("err_pos_x,err_pos_y,err_theta", err_pos_x, err_pos_y, err_theta)


#  fix the first node to be known
    H[0:3, 0:3] = H[0:3, 0:3] + 1e6*np.eye(3)
    # solve the linear system
    # x = H\b
    x = np.linalg.solve(H, b)

    # print(x)
    # print(x.shape)

    dx_norm = np.linalg.norm(x)
    iteration = iteration + 1
    print(f'iter {iteration} with delta = {dx_norm}')

    # update our nodes
    for index, node in enumerate(nodes):
        node = update_node(node, x[3*index:3*index+3, 0])


x_cords = [node["state"][0] for node in nodes]
y_cords = [node["state"][1] for node in nodes]
theta = [node["state"][2] for node in nodes]

plt.plot(x_cords, y_cords, color="red", label="After optimization")
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Nodes from g2o file')
plt.grid(True)
plt.legend()
plt.show()
