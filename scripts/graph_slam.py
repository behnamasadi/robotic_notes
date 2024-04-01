

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
