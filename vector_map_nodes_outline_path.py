import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
from scipy.spatial import distance
import math

# Implemented based on https://hal.science/hal-02150278/document

# Set parameters
initial_line_dist = 1  # Distance threshold to connect red cells
shortcut_dist = 1
close_ends_dist = 1.5

# cls = [( 5.5 , 7.5 ),( 6.5 , 7.5 ),( 6.5 , 6.5 ),( 6.5 , 5.5 ),( 7.5 , 5.5 ),( 7.5 , 6.5 ),( 7.5 , 4.5 ),( 6.5 , 4.5 ),( 6.5 , 3.5 ),( 7.5 , 3.5 ),( 7.5 , 2.5 ),( 6.5 , 2.5 ),( 6.5 , 1.5 ),( 5.5 , 1.5 ),( 4.5 , 1.5 ),( 3.5 , 1.5 ),( 3.5 , 2.5 ),( 2.5 , 2.5 ),( 2.5 , 3.5 ),( 1.5 , 3.5 ),( 1.5 , 4.5 ),( 2.5 , 4.5 ),( 2.5 , 5.5 ),( 3.5 , 5.5 ),( 3.5 , 6.5 ),( 3.5 , 0.5 ),( 3.5 , 7.5 ),( 3.5 , 8.5 ),( 3.5 , 9.5 ),( 4.5 , 7.5 ),( 6.5 , 0.5 ),( 8.5 , 3.5 ),( 2.5 , 7.5 ),( 8.5 , 6.5 )]
# cls = [( 4.5 , 7.5 ),( 3.5 , 6.5 ),( 3.5 , 7.5 ),( 2.5 , 6.5 ),( 2.5 , 5.5 ),( 3.5 , 5.5 ),( 3.5 , 4.5 ),( 2.5 , 4.5 ),( 2.5 , 3.5 ),( 3.5 , 3.5 ),( 2.5 , 2.5 ),( 3.5 , 2.5 ),( 4.5 , 2.5 ),( 4.5 , 1.5 ),( 6.5 , 1.5 ),( 5.5 , 1.5 ),( 6.5 , 0.5 ),( 7.5 , 1.5 ),( 8.5 , 2.5 ),( 7.5 , 2.5 ),( 6.5 , 2.5 ),( 6.5 , 3.5 ),( 8.5 , 1.5 ),( 7.5 , 4.5 ),( 6.5 , 4.5 ),( 6.5 , 5.5 ),( 7.5 , 5.5 ),( 7.5 , 6.5 ),( 5.5 , 6.5 ),( 6.5 , 7.5 ),( 5.5 , 7.5 ),( 6.5 , 6.5 )]
# cls = [( 2.5 , 6.5 ),( 2.5 , 5.5 ),( 3.5 , 5.5 ),( 3.5 , 6.5 ),( 3.5 , 4.5 ),( 3.5 , 3.5 ),( 4.5 , 3.5 ),( 5.5 , 3.5 ),( 5.5 , 4.5 ),( 6.5 , 4.5 ),( 4.5 , 6.5 ),( 5.5 , 6.5 ),( 6.5 , 6.5 ),( 6.5 , 7.5 ),( 7.5 , 7.5 ),( 7.5 , 6.5 ),( 8.5 , 6.5 ),( 8.5 , 5.5 ),( 8.5 , 4.5 ),( 7.5 , 4.5 )]
cls = [( 3.5 , 15.5 ),( 4.5 , 15.5 ),( 4.5 , 14.5 ),( 3.5 , 14.5 ),( 3.5 , 13.5 ),( 4.5 , 13.5 ),( 5.5 , 13.5 ),( 4.5 , 12.5 ),( 5.5 , 12.5 ),( 5.5 , 11.5 ),( 4.5 , 11.5 ),( 5.5 , 10.5 ),( 6.5 , 10.5 ),( 6.5 , 11.5 ),( 7.5 , 11.5 ),( 7.5 , 12.5 ),( 8.5 , 13.5 ),( 8.5 , 14.5 ),( 8.5 , 15.5 ),( 8.5 , 12.5 ),( 7.5 , 15.5 ),( 6.5 , 15.5 ),( 6.5 , 16.5 ),( 4.5 , 16.5 ),( 5.5 , 16.5 ),( 12.5 , 13.5 ),( 12.5 , 14.5 ),( 12.5 , 15.5 ),( 13.5 , 15.5 ),( 13.5 , 14.5 ),( 13.5 , 13.5 ),( 15.5 , 17.5 ),( 15.5 , 16.5 ),( 15.5 , 15.5 ),( 15.5 , 14.5 ),( 15.5 , 13.5 ),( 15.5 , 12.5 ),( 14.5 , 12.5 ),( 16.5 , 14.5 ),( 15.5 , 11.5 ),( 16.5 , 10.5 ),( 15.5 , 10.5 ),( 17.5 , 12.5 ),( 17.5 , 10.5 ),( 17.5 , 11.5 ),( 18.5 , 12.5 ),( 18.5 , 13.5 ),( 18.5 , 14.5 ),( 19.5 , 15.5 ),( 18.5 , 15.5 ),( 18.5 , 16.5 ),( 17.5 , 16.5 ),( 17.5 , 17.5 )]
# ncls = [( 5.5 , 15.5 ),( 5.5 , 14.5 ),( 6.5 , 14.5 ),( 7.5 , 14.5 ),( 7.5 , 13.5 ),( 6.5 , 13.5 ),( 6.5 , 12.5 )]
# ncls = [( 2.5 , 15.5 ),( 1.5 , 15.5 ),( 0.5 , 15.5 ),( 0.5 , 14.5 ),( 1.5 , 14.5 ),( 2.5 , 14.5 ),( 2.5 , 13.5 ),( 1.5 , 13.5 ),( 0.5 , 13.5 ),( 0.5 , 12.5 ),( 1.5 , 12.5 ),( 2.5 , 12.5 ),( 3.5 , 12.5 ),( 3.5 , 11.5 ),( 2.5 , 11.5 ),( 0.5 , 11.5 ),( 1.5 , 11.5 ),( 0.5 , 10.5 ),( 1.5 , 10.5 ),( 1.5 , 9.5 ),( 0.5 , 9.5 ),( 2.5 , 9.5 ),( 2.5 , 10.5 ),( 1.5 , 8.5 ),( 0.5 , 8.5 ),( 0.5 , 7.5 ),( 5.5 , 15.5 ),( 5.5 , 14.5 ),( 6.5 , 14.5 ),( 7.5 , 14.5 ),( 7.5 , 13.5 ),( 6.5 , 13.5 ),( 6.5 , 12.5 )]
# ac = ( 19.641577060931898 , 17.3173835125448 )
# tc = ( 0.5376344086021505 , 10.632795698924733 )
tc = ( 1.505376344086021 , 10.363978494623657 )
ac = ( 19.516129032258064 , 14.521684587813622 )

# cls = [( 4.5 , 14.5 ),( 5.5 , 14.5 ),( 5.5 , 13.5 ),( 4.5 , 12.5 ),( 4.5 , 13.5 ),( 5.5 , 12.5 ),( 5.5 , 11.5 ),( 4.5 , 11.5 ),( 4.5 , 10.5 ),( 5.5 , 10.5 ),( 5.5 , 9.5 ),( 4.5 , 9.5 ),( 4.5 , 8.5 ),( 5.5 , 8.5 ),( 4.5 , 7.5 ),( 5.5 , 7.5 ),( 6.5 , 7.5 ),( 7.5 , 8.5 ),( 7.5 , 7.5 ),( 8.5 , 7.5 ),( 8.5 , 8.5 ),( 8.5 , 9.5 ),( 9.5 , 9.5 ),( 9.5 , 10.5 ),( 8.5 , 10.5 ),( 8.5 , 11.5 ),( 9.5 , 11.5 ),( 10.5 , 11.5 ),( 10.5 , 12.5 ),( 9.5 , 12.5 ),( 11.5 , 12.5 ),( 11.5 , 13.5 ),( 10.5 , 13.5 ),( 10.5 , 14.5 ),( 9.5 , 14.5 ),( 9.5 , 15.5 ),( 8.5 , 15.5 ),( 8.5 , 14.5 ),( 7.5 , 15.5 ),( 7.5 , 16.5 ),( 6.5 , 16.5 ),( 6.5 , 15.5 ),( 5.5 , 15.5 ),( 14.5 , 14.5 ),( 14.5 , 13.5 ),( 15.5 , 14.5 ),( 15.5 , 13.5 ),( 15.5 , 12.5 ),( 14.5 , 12.5 ),( 14.5 , 11.5 ),( 13.5 , 11.5 ),( 13.5 , 10.5 ),( 12.5 , 10.5 ),( 12.5 , 9.5 ),( 13.5 , 9.5 ),( 14.5 , 10.5 ),( 13.5 , 8.5 ),( 13.5 , 7.5 ),( 14.5 , 7.5 ),( 14.5 , 8.5 ),( 15.5 , 8.5 ),( 15.5 , 7.5 ),( 16.5 , 7.5 ),( 16.5 , 8.5 ),( 16.5 , 9.5 ),( 16.5 , 10.5 ),( 17.5 , 10.5 ),( 17.5 , 9.5 ),( 17.5 , 11.5 ),( 17.5 , 12.5 ),( 18.5 , 13.5 ),( 17.5 , 13.5 ),( 16.5 , 14.5 ),( 17.5 , 15.5 ),( 17.5 , 14.5 )]
# ncls = [( 6.5 , 13.5 ),( 6.5 , 14.5 ),( 7.5 , 13.5 ),( 8.5 , 13.5 ),( 7.5 , 12.5 ),( 6.5 , 11.5 ),( 6.5 , 10.5 ),( 7.5 , 9.5 ),( 7.5 , 10.5 ),( 6.5 , 9.5 ),( 15.5 , 9.5 ),( 14.5 , 9.5 ),( 15.5 , 10.5 ),( 15.5 , 11.5 ),( 16.5 , 11.5 ),( 16.5 , 12.5 ),( 16.5 , 13.5 ),( 7.5 , 14.5 ),( 9.5 , 13.5 ),( 8.5 , 12.5 ),( 6.5 , 12.5 ),( 7.5 , 11.5 ),( 6.5 , 8.5 )]


# cls = [( 3.5 , 7.5 ),( 3.5 , 9.5 ),( 3.5 , 8.5 ),( 3.5 , 10.5 ),( 3.5 , 11.5 ),( 3.5 , 12.5 ),( 3.5 , 13.5 ),( 4.5 , 13.5 ),( 5.5 , 13.5 ),( 6.5 , 13.5 ),( 7.5 , 13.5 ),( 8.5 , 13.5 ),( 9.5 , 13.5 ),( 9.5 , 12.5 ),( 9.5 , 11.5 ),( 9.5 , 10.5 ),( 9.5 , 9.5 ),( 9.5 , 7.5 ),( 9.5 , 8.5 ),( 10.5 , 13.5 ),( 11.5 , 13.5 ),( 12.5 , 13.5 ),( 13.5 , 13.5 ),( 14.5 , 13.5 ),( 15.5 , 13.5 ),( 15.5 , 12.5 ),( 15.5 , 11.5 ),( 15.5 , 10.5 ),( 15.5 , 9.5 ),( 15.5 , 8.5 ),( 15.5 , 7.5 )]
# cls = [( 3.5 , 7.5 ),( 3.5 , 9.5 ),( 3.5 , 8.5 ),( 3.5 , 10.5 ),( 3.5 , 11.5 ),( 3.5 , 12.5 ),( 3.5 , 13.5 ),( 4.5 , 13.5 ),( 5.5 , 13.5 ),( 6.5 , 13.5 ),( 7.5 , 13.5 ),( 8.5 , 13.5 ),( 9.5 , 13.5 ),( 9.5 , 12.5 ),( 9.5 , 11.5 ),( 9.5 , 10.5 ),( 9.5 , 9.5 ),( 9.5 , 7.5 ),( 9.5 , 8.5 ),( 10.5 , 13.5 ),( 11.5 , 13.5 ),( 12.5 , 13.5 ),( 13.5 , 13.5 ),( 14.5 , 13.5 ),( 15.5 , 13.5 ),( 15.5 , 12.5 ),( 15.5 , 11.5 ),( 15.5 , 10.5 ),( 15.5 , 9.5 ),( 15.5 , 8.5 ),( 15.5 , 7.5 ),( 3.5 , 6.5 ),( 2.5 , 6.5 ),( 1.5 , 6.5 ),( 1.5 , 7.5 ),( 1.5 , 9.5 ),( 1.5 , 8.5 ),( 1.5 , 10.5 ),( 1.5 , 11.5 ),( 1.5 , 12.5 ),( 1.5 , 13.5 ),( 1.5 , 14.5 ),( 1.5 , 15.5 ),( 2.5 , 15.5 ),( 3.5 , 15.5 ),( 4.5 , 15.5 ),( 5.5 , 15.5 ),( 6.5 , 15.5 ),( 7.5 , 15.5 ),( 8.5 , 15.5 ),( 9.5 , 15.5 ),( 11.5 , 15.5 ),( 10.5 , 15.5 ),( 12.5 , 15.5 ),( 13.5 , 15.5 ),( 14.5 , 15.5 ),( 15.5 , 15.5 ),( 16.5 , 15.5 ),( 17.5 , 15.5 ),( 17.5 , 14.5 ),( 17.5 , 13.5 ),( 17.5 , 12.5 ),( 17.5 , 11.5 ),( 17.5 , 10.5 ),( 17.5 , 9.5 ),( 17.5 , 8.5 ),( 17.5 , 7.5 ),( 17.5 , 6.5 ),( 16.5 , 6.5 ),( 15.5 , 6.5 ),( 11.5 , 12.5 ),( 11.5 , 11.5 ),( 11.5 , 10.5 ),( 11.5 , 9.5 ),( 11.5 , 8.5 ),( 11.5 , 7.5 ),( 11.5 , 6.5 ),( 10.5 , 6.5 ),( 9.5 , 6.5 )]
# ncls = [( 2.5 , 7.5 ),( 2.5 , 8.5 ),( 2.5 , 9.5 ),( 2.5 , 10.5 ),( 2.5 , 11.5 ),( 2.5 , 12.5 ),( 2.5 , 13.5 ),( 2.5 , 14.5 ),( 3.5 , 14.5 ),( 4.5 , 14.5 ),( 5.5 , 14.5 ),( 6.5 , 14.5 ),( 8.5 , 14.5 ),( 7.5 , 14.5 ),( 9.5 , 14.5 ),( 11.5 , 14.5 ),( 10.5 , 14.5 ),( 12.5 , 14.5 ),( 13.5 , 14.5 ),( 14.5 , 14.5 ),( 15.5 , 14.5 ),( 16.5 , 14.5 ),( 16.5 , 13.5 ),( 16.5 , 12.5 ),( 16.5 , 11.5 ),( 16.5 , 10.5 ),( 16.5 , 9.5 ),( 16.5 , 8.5 ),( 16.5 , 7.5 ),( 10.5 , 7.5 ),( 10.5 , 8.5 ),( 10.5 , 9.5 ),( 10.5 , 10.5 ),( 10.5 , 11.5 ),( 10.5 , 12.5 )]
# ncls = []

# ac = ( 4.247311827956989 , 4.880107526881721 )
# tc = ( 13.207885304659495 , 16.49301075268817 )
# ac = ( 7.70779220779221 , 8.194805194805195 )
# tc = ( 1.4870129870129873 , 5.480519480519481 )
# ac = ( 1.655844155844156 , 4.532467532467534 )
# tc = ( 5.422077922077923 , 0.5064935064935068 )
# ac = ( 3.88961038961039 , 1.5324675324675328 )
# tc = ( 1.2012987012987013 , 4.818181818181818 )

# ac = ( 1.666666666666667 , 15.328136200716846 )
# tc = ( 19.623655913978492 , 11.636379928315412 )
# ac = ( 2.4910394265232974 , 15.435663082437278 )
# tc = ( 16.50537634408602 , 6.277956989247313 )

# ac=( 6.397849462365592 , 2.460887096774194 )
# tc=( 8.145161290322582 , 15.95551075268817 )

#very crowded
# cls = [( 2.5 , 8.5 ),( 2.5 , 9.5 ),( 2.5 , 10.5 ),( 2.5 , 11.5 ),( 2.5 , 12.5 ),( 3.5 , 12.5 ),( 4.5 , 13.5 ),( 3.5 , 13.5 ),( 2.5 , 13.5 ),( 4.5 , 12.5 ),( 4.5 , 11.5 ),( 5.5 , 11.5 ),( 4.5 , 10.5 ),( 5.5 , 10.5 ),( 5.5 , 9.5 ),( 6.5 , 10.5 ),( 6.5 , 9.5 ),( 6.5 , 8.5 ),( 8.5 , 8.5 ),( 8.5 , 10.5 ),( 8.5 , 12.5 ),( 9.5 , 13.5 ),( 9.5 , 12.5 ),( 9.5 , 11.5 ),( 8.5 , 9.5 ),( 8.5 , 11.5 ),( 8.5 , 13.5 ),( 10.5 , 13.5 ),( 11.5 , 13.5 ),( 11.5 , 12.5 ),( 11.5 , 11.5 ),( 11.5 , 10.5 ),( 11.5 , 8.5 ),( 10.5 , 8.5 ),( 10.5 , 9.5 ),( 11.5 , 9.5 ),( 17.5 , 14.5 ),( 17.5 , 15.5 ),( 15.5 , 15.5 ),( 15.5 , 12.5 ),( 15.5 , 9.5 ),( 14.5 , 8.5 ),( 15.5 , 8.5 ),( 16.5 , 8.5 ),( 11.5 , 15.5 ),( 13.5 , 15.5 ),( 14.5 , 15.5 ),( 12.5 , 15.5 ),( 16.5 , 15.5 ),( 16.5 , 14.5 ),( 16.5 , 13.5 ),( 15.5 , 13.5 ),( 14.5 , 11.5 ),( 14.5 , 10.5 ),( 14.5 , 12.5 ),( 14.5 , 9.5 ),( 15.5 , 2.5 ),( 10.5 , 2.5 ),( 9.5 , 4.5 ),( 4.5 , 5.5 ),( 4.5 , 4.5 ),( 3.5 , 4.5 ),( 2.5 , 3.5 ),( 3.5 , 3.5 ),( 3.5 , 2.5 ),( 5.5 , 5.5 ),( 6.5 , 5.5 ),( 6.5 , 4.5 ),( 6.5 , 3.5 ),( 6.5 , 2.5 ),( 5.5 , 1.5 ),( 4.5 , 1.5 ),( 3.5 , 1.5 ),( 6.5 , 1.5 ),( 9.5 , 2.5 ),( 9.5 , 3.5 ),( 9.5 , 5.5 ),( 10.5 , 5.5 ),( 11.5 , 4.5 ),( 10.5 , 4.5 ),( 10.5 , 3.5 ),( 11.5 , 2.5 ),( 15.5 , 4.5 ),( 15.5 , 5.5 ),( 13.5 , 2.5 ),( 14.5 , 5.5 ),( 16.5 , 4.5 ),( 17.5 , 2.5 ),( 16.5 , 9.5 ),( 17.5 , 9.5 ),( 17.5 , 10.5 )]

class Node:
    def __init__(self, coordinate):
        self.coordinate = coordinate
        self.vectorNodes = [None, None, None, None]
        self.left = coordinate[0]-0.5
        self.right = coordinate[0]+0.5
        self.top = coordinate[1]+0.5
        self.bottom = coordinate[1]-0.5

    def __eq__(self, other):
        if isinstance(other, Node):
            return self.coordinate == other.coordinate
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

    def contains(self, c):
        result = self.left <= c[0] and self.right >= c[0] and self.top >= c[1] and self.bottom <= c[1]
        return result


# Store the positions of red cells and lines
red_cells = []
free_cells = []
free_cells_plot = []
lines = []
# linecells = []
to_be_deleted = []
to_be_added = []

potential_shortcuts = []

setting_agent = False
setting_target = False
removing_free_cells = False

agent = None
agentplot = None
target = None
targetplot = None

gridsize = 20

# Create a grid
fig, ax = plt.subplots(figsize=(gridsize,gridsize))
ax.set_xlim(0, gridsize)
ax.set_ylim(0, gridsize)
ax.set_xticks(np.arange(0, gridsize+1, 1))
ax.set_yticks(np.arange(0, gridsize+1, 1))
ax.grid(True)

bbox = ax.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
width, height = bbox.width, bbox.height
cellplotsize = width/gridsize * fig.dpi /3

#time complexity: O(1)
def connected_edge(node, node2):
    if node.left == node2.right: # node2 is to the left of node
        return 0
    elif node.right == node2.left: # node2 is to the right of node
        return 2
    elif node.top == node2.bottom: # node2 is above node
        return 1
    elif node.bottom == node2.top: # node2 is below node
        return 3

#time complexity: O(1)
def draw_line(node, other_node):
    """Draw a line between two red cells c1 and c2."""
    line, = ax.plot([node.coordinate[0], other_node.coordinate[0]], [node.coordinate[1], other_node.coordinate[1]], 'b-')
    #if line doesn't exist yet, add it to each node
    if (other_node not in node.vectorNodes):
        c_edge = connected_edge(node, other_node) #O(1)
        other_edge = c_edge + 2 if c_edge < 2 else c_edge - 2
        node.vectorNodes[c_edge] = other_node
        other_node.vectorNodes[other_edge] = node
        lines.append((node, other_node, line))

#time complexity: O(1)
def draw_target_agent_line(node, node2, edge=None):
    """Draw a line between two red cells c1 and c2."""
    if edge:
        (start_edge, end_edge) = get_start_end_edge(node2, edge) #O(1)
        halfway_pt = ( (start_edge[0] + end_edge[0])/2, (start_edge[1] + end_edge[1])/2)
        line, = ax.plot([node.coordinate[0], halfway_pt[0]], [node.coordinate[1], halfway_pt[1]], 'g-')
    else:
        line, = ax.plot([node.coordinate[0], node2.coordinate[0]], [node.coordinate[1], node2.coordinate[1]], 'g-')


#time complexity: O(1)
#returns the two intersections between a line (x1, y1) too (x2, y2) and a box (x_left, y_bottom, x_right, y_top)
def liang_barsky(x_left, y_bottom, x_right, y_top, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    p = [-dx, dx, -dy, dy]
    q = [x1 - x_left, x_right - x1, y1 - y_bottom, y_top - y1]
    t_enter = 0.0
    t_exit = 1.0

    for i in range(4):
        if p[i] == 0:  # Check if line is parallel to the clipping boundary
            if q[i] < 0:
                return None  # Line is outside and parallel, so completely discarded
        else:
            t = q[i] / p[i]
            if p[i] < 0:
                if t > t_enter:
                    t_enter = t
            else:
                if t < t_exit:
                    t_exit = t

    if t_enter > t_exit:
        return None  # Line is completely outside

    x1_clip = x1 + t_enter * dx if t_enter != 0 else None
    y1_clip = y1 + t_enter * dy if t_enter != 0 else None
    x2_clip = x1 + t_exit * dx if t_exit != 0 else None
    y2_clip = y1 + t_exit * dy if t_exit != 0  else None

    return x1_clip, y1_clip, x2_clip, y2_clip


#time complexity: O(ns)
#where ns is the number of occupied cells in the search space
def ray_trace_occupied_intersection(node, node2):
    agent_x = node.coordinate[0]
    agent_y = node.coordinate[1]
    dx = node2.coordinate[0]- agent_x
    dy = node2.coordinate[1] - agent_y
    d = math.sqrt(dx * dx + dy * dy)
    stepSize = 0.1
    nSteps = math.ceil(d / stepSize)
    stepX = dx / nSteps
    stepY = dy / nSteps

    agent_x += stepX
    agent_y += stepY

    for s in range(nSteps-1):
        for cell in red_cells:
            if cell != node:
                if cell.contains((agent_x, agent_y)):
                    #find intersection edge
                    dist=math.inf
                    edge = None
                    
                    #get first and last intersections with the bos
                    fst_x, fst_y, snd_x, snd_y = liang_barsky(cell.left, cell.bottom, cell.right, cell.top, node.coordinate[0], node.coordinate[1], node2.coordinate[0], node2.coordinate[1])

                    (int_x, int_y) = (fst_x, fst_y) if fst_x and fst_y else (snd_x, snd_y)
                    #get the edge that fst_x, fst_y is on (with margin)
                    if abs(int_x - cell.left) < 0.01: #left
                        edge = 0
                    elif abs(int_x - cell.right) < 0.01: #right
                        edge = 2
                    elif abs(int_y - cell.top) < 0.01: #top
                        edge = 1
                    elif abs(int_y - cell.bottom) < 0.01: #bottom
                        edge = 3

                    dist = distance.euclidean((node.coordinate[0], node.coordinate[1]), (int_x, int_y))

                    # if edge:
                    return ((cell, edge), dist)

        agent_x += stepX
        agent_y += stepY
    return (None, None)


def outline_route(start_node, target_node, node, edge, route):
    find_outline_segment(start_node, target_node, node, edge, route)


#edge: 0=left, 1=top, 2=right, 3=bottom
#time complexity: O(ns)
#ns is the number of occupied cells in the search space
def direction_to_target_free(node, edge, target_node, dist, route):
    line_x = node.coordinate[0]
    line_y = node.coordinate[1]

    if edge == 0:
        line_x = node.left
    elif edge == 1:
        line_y = node.top
    elif edge == 2:
        line_x = node.right
    elif edge == 3:
        line_y = node.bottom

    dist_calc_node = Node((line_x, line_y))
    intersection_edge, c_dist = ray_trace_occupied_intersection(dist_calc_node, target_node) #O(ns)  time complexity

    #if intersection edge is in route, return false
    if intersection_edge:
        (intersection_cell, intersection_edge) = intersection_edge
        (start_edge, end_edge) = get_start_end_edge(intersection_cell, intersection_edge) #c time complexity
        if (start_edge, end_edge) in route: #can't end up at the same edge twice
            return False
        if intersection_cell == node: #if we intersect the same node, we intersect another edge of the node, so direction not free
            return False
        if c_dist > 0.5: #if the intersection is far enough away, return true
            return True
    return True;    

#time complexity: O(1)
def get_start_end_edge(node, edge):
    node_coords = node.coordinate
    if edge == 0:
        start_edge = (node_coords[0]-0.5, node_coords[1]-0.5)
        end_edge = (node_coords[0]-0.5, node_coords[1]+0.5)
    elif edge == 1:
        start_edge = (node_coords[0]-0.5, node_coords[1]+0.5)
        end_edge = (node_coords[0]+0.5, node_coords[1]+0.5)
    elif edge == 2:
        start_edge = (node_coords[0]+0.5, node_coords[1]+0.5)
        end_edge = (node_coords[0]+0.5, node_coords[1]-0.5)
    elif edge == 3:
        start_edge = (node_coords[0]+0.5, node_coords[1]-0.5)
        end_edge = (node_coords[0]-0.5, node_coords[1]-0.5)
    else:
        raise ValueError("Invalid edge")
    return (start_edge, end_edge)

#time complexity: O(1)
def get_edge_midpoint(node, edge):
    n_coords = node.coordinate
    if edge == 0:
        return (node.left, n_coords[1])
    elif edge == 1:
        return (n_coords[0], node.top)
    elif edge == 2:
        return (node.right, n_coords[1])
    elif edge == 3:
        return (n_coords[0], node.bottom)
    
def draw_line_and_intersection(node, node2, route):
    # draw_target_agent_line(node, node2)
    intersection_edge, c_dist = ray_trace_occupied_intersection(node, node2) #O(ns) time complexity
    if intersection_edge:
        (intersection, edge) = intersection_edge
        start_edge, end_edge = get_start_end_edge(intersection, edge) #c time complexity
        halfway_pt = ( (start_edge[0] + end_edge[0])/2, (start_edge[1] + end_edge[1])/2)
        draw_target_agent_line(node, Node(halfway_pt), edge) #c time complexity
        # ax.plot(halfway_pt[0], halfway_pt[1], "go", markersize=10)
        route.append((start_edge, end_edge))
        #draw initial line
        line, = ax.plot([start_edge[0], end_edge[0]], [start_edge[1], end_edge[1]], 'c-')
        plt.draw()
        outline_route(node, node2, intersection, edge, route)
    else:
        draw_target_agent_line(node, node2)




#time complexity: O(1)
def optional_remove(list, element):
    if element in list:
        list.remove(element)
        

#find all existing edges connected to the given edge of the node
#time complexity: O(1)
def elegible_edges(node, edge):
    edges = []
    opposite_edge = edge + 2 if edge < 2 else edge - 2

    #adding all other edges around node
    edges.append((node, (edge+1)%4))
    edges.append((node, (edge+3)%4))
    
    #removing edges that are already connected
    for i,n in enumerate(node.vectorNodes):
        if n:
            optional_remove(edges, (node, i))
            if i != edge and i != opposite_edge:
                if node.vectorNodes[i]:
                    if node.vectorNodes[i].vectorNodes[edge]:
                        opposite_i = i + 2 if i < 2 else i - 2
                        edges.append((node.vectorNodes[i].vectorNodes[edge], opposite_i))
                    else:
                        edges.append((node.vectorNodes[i], edge))
    
    return edges


#time complexity:
#O(ns*e) 
# where ns is the number of occupied cells in the search space 
# and e is the number of edges along the path (all outline edges of all nodes = 4n)
#so O(ns*n) so O(n^2)
#But in reality search space will be:
#ns = (FRONTIER_SEARCH_DIAMETER/2/smallest_box_size * FRONTIER_SEARCH_DIAMETER/2/smallest_box_size) 
# = (FRONTIER_SEARCH_DIAMETER^2)/(4*smallest_box_size^2)
# = (8^2)/(4*0.15^2) = 64/0.09 = 711
#where most cells will not be occupied
#and e will be much smaller than n
#so in reality it will be much faster 
# 
# Also it won't be O(ns) in our actual implementation because we are using an optimized quadtree  
#It's actually O(log2(ns)) where d is the depth of  the tree
#and log2(711) = 9.47 so d = 10
#so O(log2(n)*e) = O(n*log2(n)) where e << n
#
#Let's say 20% of the cells are occupied, so ns = 711*0.2 = 142
#Max lines per occupied cell is 2
#so e = 142*2 = 284
#so O(log2(n)*e) = O(142*log2(142)*2) = O(142*7*2) = O(1988)

#edge: 0=left, 1=top, 2=right, 3=bottom
def find_outline_segment(start_node, target_node, node, edge, route):
    if direction_to_target_free(node, edge, target_node, None, route): #O(ns)
        # draw_target_agent_line(node, target_node)
        (start_edge, end_edge) = get_start_end_edge(node, edge) #O(1)
        halfway_pt = ( (start_edge[0] + end_edge[0])/2, (start_edge[1] + end_edge[1])/2)

        draw_line_and_intersection(Node(halfway_pt), target_node, route) #Don't know yet
        return
    d_max = -math.inf
    d_min = math.inf
    bestNode = None
    bestEdge = None
    bestEdgecoord = None
    bestpt = None
    # (ptN, node_to_segment) = point_to_segment_distance(node.coordinate, agent.coordinate, target.coordinate)
    edges = elegible_edges(node, edge) #O(1)
    for (n, e) in edges: #O(c) where c is max 2 so O(1)
        start_edge, end_edge = get_start_end_edge(n, e) #O(1)
        halfway_pt = ( (start_edge[0] + end_edge[0])/2, (start_edge[1] + end_edge[1])/2)
        
        agent_to_hpt = distance.euclidean(agent.coordinate, halfway_pt) #O(1)
        s = agent_to_hpt
        if (start_edge, end_edge) not in route:
            if s > d_max:
                d_max = s
                bestEdgecoord = (start_edge, end_edge)
                bestNode = n
                bestEdge = e
                bestpt = halfway_pt
    #If no further option, make new line from this node to target
    if not bestNode:
        (start_edge, end_edge) = get_start_end_edge(node, edge) #O(1)
        halfway_pt = ( (start_edge[0] + end_edge[0])/2, (start_edge[1] + end_edge[1])/2)

        draw_line_and_intersection(Node(halfway_pt), target_node, route) #Don't know yet
        return
    
    #else we continue from the best node
    ax.plot(bestpt[0], bestpt[1], "co", markersize=10)
    route.append(bestEdgecoord)
    line, = ax.plot([bestEdgecoord[0][0], bestEdgecoord[1][0]], [bestEdgecoord[0][1], bestEdgecoord[1][1]], 'c-')

    find_outline_segment(start_node, target_node, bestNode, bestEdge, route)

def add_free_cell(coordinate):
    (x,y) = coordinate
    cell = Node((x,y))

    # If the cell is already free, do nothing
    if cell in free_cells:
        return
    
    if cell in [agent, target]:
        return
    
    free_cells.append(cell)
    cellplt = ax.plot(cell.coordinate[0], cell.coordinate[1], "gs", markersize=cellplotsize, alpha=0.5)
    free_cells_plot.append(cellplt)


def fill_grid():
    for i in range(gridsize):
        for j in range(gridsize):
            add_free_cell((i+0.5, j+0.5))
    plt.draw()


def add_cell(coordinate):
    (x,y) = coordinate
    cell = Node((x,y))
    # If the cell is already red, do nothing
    if cell in red_cells:
        return
    
    if cell in [agent, target]:
        return

    # Add the cell to the list of red cells and plot it
    red_cells.append(cell)

    

    ix = free_cells.index(cell)
    del free_cells[ix]
    free_cells_plot[ix][0].remove()
    del free_cells_plot[ix]
    ax.plot(cell.coordinate[0], cell.coordinate[1], "rs", markersize=cellplotsize, alpha=0.5)


    for other_cell in red_cells:
        if other_cell != cell and distance.euclidean(cell.coordinate, other_cell.coordinate) <= initial_line_dist:
            draw_line(cell, other_cell)
    
    print('(', x, ',', y, '),', end='')

    # Check for shortcut detection
    # check_shortcuts(cell)
    plt.draw()

def set_agent_coordinate(cell):
    global agent
    agent = cell
    global setting_agent
    setting_agent = False
    (x,y) = cell.coordinate
    global agentplot
    agentplot = ax.plot(x, y, "bs", markersize=cellplotsize)
    print('\nAgent: (', x, ',', y, '),', end='\n')

def set_target_coordinate(cell):
    global target
    target = cell
    global setting_target
    setting_target = False
    (x,y) = cell.coordinate
    global targetplot
    targetplot = ax.plot(x, y, "gs", markersize=cellplotsize)
    print('\nTarget: (', x, ',', y, '),', end='\n')


def rm_cell(coordinate):
    (x,y) = coordinate
    cell = Node((x,y))
    # If the cell is already red, do nothing
    if cell not in free_cells:
        return
    
    # Add the cell to the list of red cells and plot it
    ix = free_cells.index(cell)
    del free_cells[ix]
    free_cells_plot[ix][0].remove()
    del free_cells_plot[ix]

    # Connect the cell to nearby red cells within distance e
    for other_cell in free_cells:
        if other_cell != cell and distance.euclidean(cell.coordinate, other_cell.coordinate) <= initial_line_dist:
            for (an, bn, l) in lines:
                if (an,bn) == (cell, other_cell) or (an,bn) == (other_cell, cell):
                    lines.remove((an,bn,l))
                    ix_an = an.vectorNodes.index(bn)
                    an.vectorNodes[ix_an] = None
                    ix_bn = bn.vectorNodes.index(an)
                    bn.vectorNodes[ix_bn] = None
                    l.remove()

    print('(', x, ',', y, '),', end='')

    
    # Check for shortcut detection
    # check_shortcuts(cell)
    plt.draw()


def on_click(event):
    """Handle mouse click events to add red cells and connect lines."""
    if event.inaxes != ax:
        return

    # Get the coordinates of the click and round them to the nearest grid point
    x, y = event.xdata, event.ydata
    cell = Node((x, y))

    global setting_agent
    global setting_target
    if setting_agent:
        set_agent_coordinate(cell)
    elif setting_target:
        set_target_coordinate(cell)
    elif removing_free_cells:
        x, y = math.floor(event.xdata)+0.5, math.floor(event.ydata)+0.5
        rm_cell((x,y))
    else:
        x, y = math.floor(event.xdata)+0.5, math.floor(event.ydata)+0.5
        add_cell((x,y))
    plt.draw()

    if agent and target:
        # draw_target_agent_line(agent, target)
        draw_line_and_intersection(agent ,target, [])



def connect_close_ends(self):
    for cell in red_cells:
        for cell2 in red_cells:
            d = distance.euclidean(cell.coordinate, cell2.coordinate)
            neq = cell != cell2
            # if cell != cell2 and ((len(cell.vectorNodes) == 1 and len(cell2.vectorNodes) >0) or (len(cell2.vectorNodes) == 1 and len(cell.vectorNodes) > 0)) and d <= close_ends_dist:
            # if cell != cell2 and ((len(cell.vectorNodes) == 1 and len(cell2.vectorNodes) ==1)) and d <= close_ends_dist:
            a_term = len(cell.vectorNodes) in [0,1] and len(cell2.vectorNodes) in [0,1]
            if cell != cell2 and a_term and d <= close_ends_dist:
                draw_line(cell, cell2)
    plt.draw()

def remove_free_cells(self):
    global removing_free_cells
    removing_free_cells = not removing_free_cells
    print('removing free cells:', removing_free_cells)

def set_agent(self):
    global setting_agent
    setting_agent = True
    global setting_target
    setting_target = False

def set_target(self):
    global setting_target
    setting_target = True
    global setting_agent
    setting_agent = False

def preset_cells(self):
    for c in cls:
        add_cell(c)
    for c in ncls:
        rm_cell(c)

def preset_agent_target(self):
    set_agent_coordinate(Node(ac))
    set_target_coordinate(Node(tc))
    # draw_target_agent_line(agent, target)
    draw_line_and_intersection(agent ,target, [])


# Connect the click event to the handler
cid = fig.canvas.mpl_connect('button_press_event', on_click)
ax.set_aspect('equal', adjustable='box')

axrmfree = fig.add_axes([0, 0.8, 0.1, 0.075])
axpreset = fig.add_axes([0, 0.65, 0.1, 0.075])
axconnect = fig.add_axes([0, 0.50, 0.1, 0.075])
axtarget = fig.add_axes([0, 0.35, 0.1, 0.075])
axagent = fig.add_axes([0, 0.2, 0.1, 0.075])
axpreset_ta = fig.add_axes([0, 0.05, 0.1, 0.075])

brmfree = Button(axrmfree, 'Remove free')
brmfree.on_clicked(remove_free_cells)

bpreset = Button(axpreset, 'Preset cells')
bpreset.on_clicked(preset_cells)


bconnect = Button(axconnect, 'Connect loose')
bconnect.on_clicked(connect_close_ends)

bagent = Button(axagent, 'Set Agent')
bagent.on_clicked(set_agent)

btarget = Button(axtarget, 'Set Target')
btarget.on_clicked(set_target)

bpreset_ta = Button(axpreset_ta, 'Preset Agent and Target')
bpreset_ta.on_clicked(preset_agent_target)

fill_grid()

plt.show()
