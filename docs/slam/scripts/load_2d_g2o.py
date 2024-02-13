def load_2d_g2o(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()

    nodes = []
    edges = []

    for line in lines:
        line = line.strip()  # Remove leading and trailing whitespaces
        if line.startswith('VERTEX_SE2'):
            parts = line.split()
            node = {}
            node['id'] = int(parts[1])
            node['state'] = [float(parts[2]), float(
                parts[3]), float(parts[4])]  # x, y, theta
            nodes.append(node)
        elif line.startswith('EDGE_SE2'):
            parts = line.split()
            edge = {}
            edge['id1'] = int(parts[1])  # idout
            edge['id2'] = int(parts[2])  # idin
            edge['meas'] = [float(parts[3]), float(
                parts[4]), float(parts[5])]  # dx, dy, dtheta
            # I11 I12 I13 I22 I23 I33
            edge['info'] = [[float(parts[6]), float(parts[7]), float(parts[8])],
                            [float(parts[7]), float(
                                parts[9]), float(parts[10])],
                            [float(parts[8]), float(parts[10]), float(parts[11])]]
            edges.append(edge)

    return nodes, edges

# Usage
# nodes, edges = load_2d_g2o('your_filename.g2o')


def main(filename):
    load_2d_g2o(filename)


if (__name__ == "__main__"):
    main("foo")
