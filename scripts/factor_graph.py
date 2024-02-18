# conda install -c anaconda networkx
# pip install --upgrade networkx matplotlib
import matplotlib.pyplot as plt
import networkx as nx

# Create the graph
G = nx.Graph()

# Add variable nodes
variables = ["x", "y", "z"]
G.add_nodes_from(variables, color="lightblue")

# Add factor nodes
factors = ["f1", "f2"]
G.add_nodes_from(factors, color="lightcoral")

# Add edges between factors and their corresponding variables
G.add_edges_from([("f1", "x"), ("f1", "y"), ("f2", "y"), ("f2", "z")])

# Draw the graph
colors = ["lightblue" if n in variables else "lightcoral" for n in G.nodes()]
pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True, node_color=colors, node_size=2000)
plt.title("Factor Graph")
plt.show()
