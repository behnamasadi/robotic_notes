import numpy as np
import g2o
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import g2o
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class PoseGraphOptimization(g2o.SparseOptimizer):
    def __init__(self):
        super().__init__()
        self.edges = []  # Add this line to store edges manually
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        super().set_algorithm(solver)

    def optimize(self, max_iterations=20):
        super().initialize_optimization()
        super().optimize(max_iterations)

    def add_vertex(self, id, pose, fixed=False):
        v_se3 = g2o.VertexSE3()
        v_se3.set_id(id)
        v_se3.set_estimate(pose)
        v_se3.set_fixed(fixed)
        super().add_vertex(v_se3)

    def add_edge(self, vertices, measurement, information=np.identity(6), robust_kernel=None):
        edge = g2o.EdgeSE3()
        for i, v in enumerate(vertices):
            if isinstance(v, int):
                v = self.vertex(v)
            edge.set_vertex(i, v)

        edge.set_measurement(measurement)
        edge.set_information(information)
        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        super().add_edge(edge)
        self.edges.append((vertices, edge))  # Store edge for manual iteration


    def get_pose(self, id):
        return self.vertex(id).estimate()

# Adjust visualization to use the manually stored edges
def visualize_pose_graph(optimizer, title='Pose Graph'):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)

    for vertex_id in optimizer.vertices():
        pose = optimizer.get_pose(vertex_id).matrix()
        x, y, z = pose[0:3, 3]
        ax.scatter(x, y, z, marker='o')

    for vertices, edge in optimizer.edges:  # Use the manually stored edges
        xs, ys, zs = [], [], []
        for vertex_id in vertices:
            pose = optimizer.get_pose(vertex_id).matrix()[0:3, 3]
            xs.append(pose[0])
            ys.append(pose[1])
            zs.append(pose[2])
        ax.plot(xs, ys, zs, color='b')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

def information_matrix_from_upper_triangular(vector):
    """Reconstruct the full 6x6 information matrix from the upper triangular part."""
    assert len(vector) == 21, "The upper triangular part must have 21 elements."
    matrix = np.zeros((6, 6))
    indices = np.triu_indices_from(matrix)
    matrix[indices] = vector
    matrix[(indices[1], indices[0])] = vector  # Mirror the upper triangular part to the lower part
    return matrix

def load_g2o_file(optimizer, filename):
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if not parts:
                continue  # Skip empty lines

            if parts[0] == 'VERTEX_SE3:QUAT':
                # Extract vertex data
                id = int(parts[1])
                x, y, z = map(float, parts[2:5])
                qx, qy, qz, qw = map(float, parts[5:9])
                pose = g2o.Isometry3d(g2o.Quaternion(qw, qx, qy, qz), [x, y, z])
                optimizer.add_vertex(id, pose, id == 0)  # Assuming the first vertex is fixed

            elif parts[0] == 'EDGE_SE3:QUAT':
                # Extract edge data
                id_from, id_to = map(int, parts[1:3])
                x, y, z = map(float, parts[3:6])
                qx, qy, qz, qw = map(float, parts[6:10])
                measurement = g2o.Isometry3d(g2o.Quaternion(qw, qx, qy, qz), [x, y, z])

                information_vector = np.array(parts[10:]).astype(float)
                information = information_matrix_from_upper_triangular(information_vector)

                optimizer.add_edge([id_from, id_to], measurement, information)


def main():
    # optimizer = PoseGraphOptimization()
    # Load your g2o file here and add vertices and edges to the optimizer
    # This part is left as an exercise to the reader, as it involves parsing a g2o file
    # and using optimizer.add_vertex and optimizer.add_edge accordingly.

    # Example of adding vertices and edges manually
    # optimizer.add_vertex(id=0, pose=g2o.Isometry3d(), fixed=True)
    # optimizer.add_vertex(id=1, pose=g2o.Isometry3d())
    # optimizer.add_edge(vertices=[0, 1], measurement=g2o.Isometry3d())

    optimizer = PoseGraphOptimization()
    
    # Set the path to your g2o file
    #g2o_file_path = 'path/to/your/file.g2o'
    g2o_file_path = '/home/behnam/workspace/robotic_notes/data/slam/sphere_bignoise_vertex3.g2o'

    load_g2o_file(optimizer, g2o_file_path)
    
    # Optimizing the pose graph
    optimizer.optimize()

    # Visualize the pose graph
    visualize_pose_graph(optimizer, 'Optimized Pose Graph')

if __name__ == '__main__':
    main()
