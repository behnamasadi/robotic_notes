import argparse
import os


def read_g2o_file(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    vertices = {}
    edges = []

    for line in lines:
        items = line.strip().split()

        if items[0] == 'VERTEX_SE3:QUAT':
            # Each vertex has an id and 7 parameters (x, y, z, qx, qy, qz, qw)
            id = int(items[1])
            parameters = list(map(float, items[2:]))
            vertices[id] = parameters

        elif items[0] == 'VERTEX_SE2':
            # Each vertex has an id and 3 parameters (x, y, theta)
            id = int(items[1])
            parameters = list(map(float, items[2:]))
            vertices[id] = parameters

        elif items[0] == 'EDGE_SE3:QUAT':
            # Each edge has two vertex ids and an information matrix (for the error function)
            id1, id2 = map(int, items[1:3])
            measurement = list(map(float, items[3:10]))
            info_matrix = list(map(float, items[10:]))
            edges.append((id1, id2, measurement, info_matrix))

        elif items[0] == 'EDGE_SE2':
            # Each edge has two vertex ids and an information matrix (for the error function)
            id1, id2 = map(int, items[1:3])
            measurement = list(map(float, items[3:6]))
            info_matrix = list(map(float, items[6:]))
            edges.append((id1, id2, measurement, info_matrix))

    return vertices, edges


def read_g2o_file_SE2(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    vertices = {}
    edges = []

    for line in lines:
        items = line.strip().split()

        if items[0] == 'VERTEX_SE2':
            # Each vertex has an id and 3 parameters (x, y, theta)
            id = int(items[1])
            parameters = list(map(float, items[2:]))
            vertices[id] = parameters

        elif items[0] == 'EDGE_SE2':
            # Each edge has two vertex ids and an information matrix (for the error function)
            id1, id2 = map(int, items[1:3])
            measurement = list(map(float, items[3:6]))
            info_matrix = list(map(float, items[6:]))
            edges.append((id1, id2, measurement, info_matrix))

    return vertices, edges


def read_g2o_file_SE3(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    vertices = {}
    edges = []

    for line in lines:
        items = line.strip().split()

        if items[0] == 'VERTEX_SE3:QUAT':
            # Each vertex has an id and 7 parameters (x, y, z, qx, qy, qz, qw)
            id = int(items[1])
            parameters = list(map(float, items[2:]))
            vertices[id] = parameters

        elif items[0] == 'EDGE_SE3:QUAT':
            # Each edge has two vertex ids and an information matrix (for the error function)
            id1, id2 = map(int, items[1:3])
            measurement = list(map(float, items[3:10]))
            info_matrix = list(map(float, items[10:]))
            edges.append((id1, id2, measurement, info_matrix))

    return vertices, edges


def main():
    os.system("clear")

    parser = argparse.ArgumentParser(
        add_help="specify the g2o file -i <input-dir>")

    parser.add_argument("-i", type=str,
                        required=True,  help="input file")

    args = parser.parse_args()
    if args.i:
        print("reading input file is: ", args.i)

    vertices, edges = read_g2o_file_SE2(args.i)

    print(f"Number of vertices: {len(vertices)}")
    print(f"Number of edges {len(edges)}")


if __name__ == "__main__":
    main()
