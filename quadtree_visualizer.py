import matplotlib.pyplot as plt
import networkx as nx

def parse_quadtree(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    nodes = []
    for line in lines:
        depth = line.count('_')
        data = line.split(':')[-1].strip().split()
        x, y = float(data[0].replace(',', '.')), float(data[1].replace(',', '.'))
        nodes.append((depth, (x, y)))
    return nodes

def build_tree(nodes):
    G = nx.DiGraph()
    root = "root"
    G.add_node(root)

    def count_descendants(node, G):
        return sum(count_descendants(child, G) for child in G.successors(node)) + 1

    stack = [(root, -1)]  # (node_name, depth)
    node_counts = {}
    for depth, (x, y) in nodes:
        node_name = f"Box({x:.3f}, {y:.3f})"
        while stack and stack[-1][1] >= depth:
            stack.pop()
        parent_name, _ = stack[-1]
        G.add_node(node_name)
        G.add_edge(parent_name, node_name)
        stack.append((node_name, depth))

    for node in G.nodes:
        if node != root:
            node_counts[node] = count_descendants(node, G) - 1

    for node in list(G.nodes):
        if node != root:
            new_name = f"{node} [{node_counts[node]}]"
            G = nx.relabel_nodes(G, {node: new_name})

    return G

def visualize_tree(G):
    plt.figure(figsize=(12, 8))
    pos = nx.drawing.nx_agraph.graphviz_layout(G, prog="dot")
    nx.draw(G, pos, with_labels=True, arrows=False, node_size=2000, node_color="lightblue", font_size=8, font_weight="bold")
    plt.title("Quadtree as a Tree Diagram")
    plt.show()

if __name__ == "__main__":
    file_path = "implementation_and_examples/quadtree_output_ pipuck1.txt"  # Replace with your filename
    quadtree_nodes = parse_quadtree(file_path)
    quadtree_graph = build_tree(quadtree_nodes)
    visualize_tree(quadtree_graph)
