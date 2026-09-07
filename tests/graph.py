# %%
from pathlib import Path

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import pandas as pd

data_dir = Path("tests/data")
files = sorted(data_dir.glob("*graph.csv"))


def tree_layout(graph, root=None, vertical_gap=1.0, horizontal_gap=1.0):
    if root is None:
        roots = [node for node in graph if graph.in_degree(node) == 0]
        if len(roots) != 1:
            raise ValueError("Expected exactly one root; pass root explicitly")
        root = roots[0]

    layers = {}
    queue = [(root, 0)]
    while queue:
        node, depth = queue.pop(0)
        if node in layers:
            continue
        layers[node] = depth
        queue.extend((child, depth + 1) for child in graph.successors(node))

    positions = {}
    for depth in sorted(set(layers.values())):
        nodes = [node for node, node_depth in layers.items() if node_depth == depth]
        offset = (len(nodes) - 1) * horizontal_gap / 2
        positions.update(
            {
                node: (index * horizontal_gap - offset, -depth * vertical_gap)
                for index, node in enumerate(nodes)
            }
        )
    return positions


f = files[0]
num_vertices, num_edges = pd.read_csv(f, nrows=1).values[0]
vertices_df = pd.read_csv(
    f, skiprows=2, nrows=num_vertices, index_col=0, names=["area"]
)
edges_df = pd.read_csv(
    f, skiprows=2 + num_vertices, nrows=num_edges, names=["parent", "child"]
)

graph = nx.from_pandas_edgelist(
    edges_df,
    source="parent",
    target="child",
    create_using=nx.DiGraph,
)

root = next(node for node, degree in graph.in_degree() if degree == 0)
pos = tree_layout(graph, root=root)

fig, ax = plt.subplots(figsize=(9, 5))
# node_values = vertices_df.loc[list(graph.nodes), "area"]
nx.draw_networkx_edges(graph, pos, ax=ax, arrows=True, arrowsize=18)
nx.draw_networkx_nodes(
    graph,
    pos,
    ax=ax,
    # node_color=node_values,
    # cmap="viridis",
    node_size=1200,
)
nx.draw_networkx_labels(graph, pos, ax=ax, font_color="white")
edge_labels = {
    (parent, child): f"{vertices_df.loc[child, 'area']:,}"
    for parent, child in graph.edges
}
nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels, ax=ax)
ax.set_title("Graph (node labels = IDs, edge labels = child area)")
ax.axis("off")
fig.tight_layout()
plt.show()
