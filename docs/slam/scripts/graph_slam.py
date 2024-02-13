# tutorials
# https://github.com/chinhsuanwu/awesome-visual-odometry
# https://github.com/gisbi-kim/modern-slam-tutorial-python
# https://github.com/RainerKuemmerle/g2o
# https://github.com/miquelmassot/g2o-python
# https://github.com/niconielsen32/VisualSLAM/blob/main/bundle_adjustment_solution.py
# https://github.com/niconielsen32/VisualSLAM/tree/main
# https://github.com/niconielsen32/VisualSLAM/blob/main/demo.py
# https://avisingh599.github.io/vision/monocular-vo/
# https://avisingh599.github.io/vision/visual-odometry-full/
# https://github.com/goldbattle/simple_2d_slam
# https://github.com/uzh-rpg/rpg_svo_pro_open
# https://github.com/FoamoftheSea/KITTI_visual_odometry/blob/main/KITTI_visual_odometry.ipynb
# https://github.com/uzh-rpg/rpg_svo_pro_open/blob/master/doc/frontend/visual_frontend.md
# https://www.youtube.com/watch?v=yXWkNC_A_YE
# https://python-graphslam.readthedocs.io/en/stable/
# https://python-graphslam.readthedocs.io/en/stable/graphslam.vertex.html
# https://lucacarlone.mit.edu/datasets/
# https://www.ipb.uni-bonn.de/datasets/
# https://github.com/RainerKuemmerle/g2o/wiki/File-Format-SLAM-2D
# https://github.com/RainerKuemmerle/g2o/tree/pymem/python/examples
# https://github.com/RainerKuemmerle/g2o/blob/pymem/python/examples/simple_optimize.py
# https://github.com/RainerKuemmerle/g2o/tree/pymem
# https://github.com/RainerKuemmerle/g2o/blob/pymem/python/examples/ba_demo.py
# https://github.com/RainerKuemmerle/g2o/blob/pymem/python/examples/notebook_slam2d.ipynb
# https://www.youtube.com/watch?v=I3K3gIvsUpY


# import numpy as np
# import g2o as g2opy
# # import g2opy
# import os
# import argparse


# import g2opy


# file = "/home/behnam/workspace/simple_2d_slam/data/input_INTEL_g2o.g2o"

# # parser = argparse.ArgumentParser()

# # parser.add_argument(
# #     "-i", "--input", type=str, default=file, help="input file"
# # )

# # args = parser.parse_args()

# # solver = g2opy.BlockSolverX(g2opy.LinearSolverEigenX())
# # solver = g2opy.OptimizationAlgorithmLevenberg(solver)
# # optimizer = g2opy.SparseOptimizer()
# # optimizer.set_verbose(True)
# # optimizer.set_algorithm(solver)

# # # optimizer.load(args.input)
# # optimizer.load(file)

# # print("num vertices:", len(optimizer.vertices()))
# # print("num edges:", len(optimizer.edges()), end="\n\n")


# def create_optimizer():
#     optimizer = g2opy.SparseOptimizer()
#     solver = g2opy.BlockSolverX(g2opy.LinearSolverEigenX())
#     solver = g2opy.OptimizationAlgorithmLevenberg(solver)
#     optimizer.set_algorithm(solver)
#     return optimizer


# optimizer = create_optimizer()

# pip install graphslam
# https://python-graphslam.readthedocs.io/en/stable/

from graphslam.load import load_g2o_se2
import numpy as np

file = "/home/behnam/workspace/simple_2d_slam/data/input_INTEL_g2o.g2o"

g = load_g2o_se2(file)
# g.plot(vertex_markersize=1)
# for vertices in g._vertices:
#     print(vertices.index)

for edge in g._edges:
    print("edge.vertex_ids: ", edge.vertex_ids)
    print("edge.information: ", edge.information)
    print("edge.estimate: ", edge.estimate)
    for v in edge.vertices:
        print("v.id: ", v.id)
        print("v.pose:", v.pose)
        print("v.index:", v.index)
        print("v.fixed:", v.fixed)
    print("**********************")


delta_x_norm = np.inf
epsilon = 0.2


print("------------------------------")


number_of_vertices = len(g._vertices)
number_of_edges = len(g._edges)

print("number of vertices: ", number_of_vertices)
print("number of edges: ", number_of_edges)

# while delta_x_norm > epsilon:
#     H = np.zeros([3*number_of_vertices, 3*number_of_vertices])
#     b = np.zeros([3*number_of_vertices, 1])
#     for edge in g._edges:
#         print("edge.vertex_ids: ", edge.vertex_ids)
#         print("edge.information: ", edge.information)
#         print("edge.estimate: ", edge.estimate)
#         for v in edge.vertices:
#             print("v.id: ", v.id)
#             print("v.pose:", v.pose)
#             print("v.index:", v.index)
#             print("v.fixed:", v.fixed)
