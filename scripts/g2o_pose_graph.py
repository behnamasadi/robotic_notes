from graphslam.graph import Graph
import os

import numpy as np
import gtsam
import pyg2o


"""
    helper functions 
"""


def vector6(x, y, z, a, b, c):
    """Create 6d double numpy array."""
    return np.array([x, y, z, a, b, c], dtype=float)


def info2mat(info):
    mat = np.zeros((6, 6))
    ix = 0
    for i in range(mat.shape[0]):
        mat[i, i:] = info[ix:ix+(6-i)]
        mat[i:, i] = info[ix:ix+(6-i)]
        ix += (6-i)

    return mat


def read_g2o(fn):
    verticies, edges = [], []
    with open(fn) as f:
        for line in f:
            line = line.split()
            if line[0] == 'VERTEX_SE3:QUAT':
                v = int(line[1])
                pose = np.array(line[2:], dtype=np.float32)
                verticies.append([v, pose])

            elif line[0] == 'EDGE_SE3:QUAT':
                u = int(line[1])
                v = int(line[2])
                pose = np.array(line[3:10], dtype=np.float32)
                info = np.array(line[10:], dtype=np.float32)

                info = info2mat(info)
                edges.append([u, v, pose, info, line])

    return verticies, edges


def write_g2o(pose_graph, fn):
    import csv
    verticies, edges = pose_graph
    with open(fn, 'w') as f:
        writer = csv.writer(f, delimiter=' ')
        for (v, pose) in verticies:
            row = ['VERTEX_SE3:QUAT', v] + pose.tolist()
            writer.writerow(row)
        for edge in edges:
            writer.writerow(edge[-1])


def write_xyz(pose_graph, fn):
    import csv
    verticies, edges = pose_graph
    with open(fn, 'w') as f:
        writer = csv.writer(f, delimiter=' ')
        for (v, pose) in verticies:
            row = pose.tolist()
            writer.writerow(row)


g = Graph.from_g2o("/home/behnam/sphere_bignoise_vertex3.g2o")
g.plot(vertex_markersize=1)
