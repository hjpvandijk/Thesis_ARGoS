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
# cls = [( 3.5 , 15.5 ),( 4.5 , 15.5 ),( 4.5 , 14.5 ),( 3.5 , 14.5 ),( 3.5 , 13.5 ),( 4.5 , 13.5 ),( 5.5 , 13.5 ),( 4.5 , 12.5 ),( 5.5 , 12.5 ),( 5.5 , 11.5 ),( 4.5 , 11.5 ),( 5.5 , 10.5 ),( 6.5 , 10.5 ),( 6.5 , 11.5 ),( 7.5 , 11.5 ),( 7.5 , 12.5 ),( 8.5 , 13.5 ),( 8.5 , 14.5 ),( 8.5 , 15.5 ),( 8.5 , 12.5 ),( 7.5 , 15.5 ),( 6.5 , 15.5 ),( 6.5 , 16.5 ),( 4.5 , 16.5 ),( 5.5 , 16.5 ),( 12.5 , 13.5 ),( 12.5 , 14.5 ),( 12.5 , 15.5 ),( 13.5 , 15.5 ),( 13.5 , 14.5 ),( 13.5 , 13.5 ),( 15.5 , 17.5 ),( 15.5 , 16.5 ),( 15.5 , 15.5 ),( 15.5 , 14.5 ),( 15.5 , 13.5 ),( 15.5 , 12.5 ),( 14.5 , 12.5 ),( 16.5 , 14.5 ),( 15.5 , 11.5 ),( 16.5 , 10.5 ),( 15.5 , 10.5 ),( 17.5 , 12.5 ),( 17.5 , 10.5 ),( 17.5 , 11.5 ),( 18.5 , 12.5 ),( 18.5 , 13.5 ),( 18.5 , 14.5 ),( 19.5 , 15.5 ),( 18.5 , 15.5 ),( 18.5 , 16.5 ),( 17.5 , 16.5 ),( 17.5 , 17.5 )]
# cls = [( 4.5 , 14.5 ),( 5.5 , 14.5 ),( 5.5 , 13.5 ),( 4.5 , 12.5 ),( 4.5 , 13.5 ),( 5.5 , 12.5 ),( 5.5 , 11.5 ),( 4.5 , 11.5 ),( 4.5 , 10.5 ),( 5.5 , 10.5 ),( 5.5 , 9.5 ),( 4.5 , 9.5 ),( 4.5 , 8.5 ),( 5.5 , 8.5 ),( 4.5 , 7.5 ),( 5.5 , 7.5 ),( 6.5 , 7.5 ),( 7.5 , 8.5 ),( 7.5 , 7.5 ),( 8.5 , 7.5 ),( 8.5 , 8.5 ),( 8.5 , 9.5 ),( 9.5 , 9.5 ),( 9.5 , 10.5 ),( 8.5 , 10.5 ),( 8.5 , 11.5 ),( 9.5 , 11.5 ),( 10.5 , 11.5 ),( 10.5 , 12.5 ),( 9.5 , 12.5 ),( 11.5 , 12.5 ),( 11.5 , 13.5 ),( 10.5 , 13.5 ),( 10.5 , 14.5 ),( 9.5 , 14.5 ),( 9.5 , 15.5 ),( 8.5 , 15.5 ),( 8.5 , 14.5 ),( 7.5 , 15.5 ),( 7.5 , 16.5 ),( 6.5 , 16.5 ),( 6.5 , 15.5 ),( 5.5 , 15.5 ),( 14.5 , 14.5 ),( 14.5 , 13.5 ),( 15.5 , 14.5 ),( 15.5 , 13.5 ),( 15.5 , 12.5 ),( 14.5 , 12.5 ),( 14.5 , 11.5 ),( 13.5 , 11.5 ),( 13.5 , 10.5 ),( 12.5 , 10.5 ),( 12.5 , 9.5 ),( 13.5 , 9.5 ),( 14.5 , 10.5 ),( 13.5 , 8.5 ),( 13.5 , 7.5 ),( 14.5 , 7.5 ),( 14.5 , 8.5 ),( 15.5 , 8.5 ),( 15.5 , 7.5 ),( 16.5 , 7.5 ),( 16.5 , 8.5 ),( 16.5 , 9.5 ),( 16.5 , 10.5 ),( 17.5 , 10.5 ),( 17.5 , 9.5 ),( 17.5 , 11.5 ),( 17.5 , 12.5 ),( 18.5 , 13.5 ),( 17.5 , 13.5 ),( 16.5 , 14.5 ),( 17.5 , 15.5 ),( 17.5 , 14.5 )]
cls = [( 3.5 , 7.5 ),( 3.5 , 9.5 ),( 3.5 , 8.5 ),( 3.5 , 10.5 ),( 3.5 , 11.5 ),( 3.5 , 12.5 ),( 3.5 , 13.5 ),( 4.5 , 13.5 ),( 5.5 , 13.5 ),( 6.5 , 13.5 ),( 7.5 , 13.5 ),( 8.5 , 13.5 ),( 9.5 , 13.5 ),( 9.5 , 12.5 ),( 9.5 , 11.5 ),( 9.5 , 10.5 ),( 9.5 , 9.5 ),( 9.5 , 7.5 ),( 9.5 , 8.5 ),( 10.5 , 13.5 ),( 11.5 , 13.5 ),( 12.5 , 13.5 ),( 13.5 , 13.5 ),( 14.5 , 13.5 ),( 15.5 , 13.5 ),( 15.5 , 12.5 ),( 15.5 , 11.5 ),( 15.5 , 10.5 ),( 15.5 , 9.5 ),( 15.5 , 8.5 ),( 15.5 , 7.5 )]

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

ac=( 6.397849462365592 , 2.460887096774194 )
tc=( 8.145161290322582 , 15.95551075268817 )

class Node:
    def __init__(self, coordinate):
        self.coordinate = coordinate
        self.vectorNodes = []
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
lines = []
# linecells = []
to_be_deleted = []
to_be_added = []

potential_shortcuts = []

setting_agent = False
setting_target = False

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

def draw_line(node, other_node):
    """Draw a line between two red cells c1 and c2."""
    line, = ax.plot([node.coordinate[0], other_node.coordinate[0]], [node.coordinate[1], other_node.coordinate[1]], 'b-')
    #if line doesn't exist yet, add it to each node
    if (other_node not in node.vectorNodes):
        node.vectorNodes.append(other_node)
        other_node.vectorNodes.append(node)
        lines.append((node, other_node, line))

def draw_target_agent_line(node, node2):
    """Draw a line between two red cells c1 and c2."""
    line, = ax.plot([node.coordinate[0], node2.coordinate[0]], [node.coordinate[1], node2.coordinate[1]], 'g-')

def find_agent_target_node_intersection(node, node2):
    agent_x = node.coordinate[0]
    agent_y = node.coordinate[1]
    dx = node2.coordinate[0]- agent_x
    dy = node2.coordinate[1] - agent_y
    d = math.sqrt(dx * dx + dy * dy)
    stepSize = 1
    nSteps = math.ceil(d / stepSize)
    stepX = dx / nSteps
    stepY = dy / nSteps

    for s in range(nSteps):
        for cell in free_cells:
            if cell != node:
                if cell.contains((agent_x, agent_y)):
                    return cell

        agent_x += stepX
        agent_y += stepY

def find_agent_target_coordinate_intersection(node, node2):
    agent_x = node.coordinate[0]
    agent_y = node.coordinate[1]
    dx = node2.coordinate[0]- agent_x
    dy = node2.coordinate[1] - agent_y
    d = math.sqrt(dx * dx + dy * dy)
    stepSize = 1
    nSteps = math.ceil(d / stepSize)
    stepX = dx / nSteps
    stepY = dy / nSteps

    for s in range(nSteps):
        for cell in red_cells:
            if cell != node:
                if cell.contains((agent_x, agent_y)):
                    return (agent_x,agent_y)

        agent_x += stepX
        agent_y += stepY
        
def point_to_segment_distance(p, a, b):
    """Returns the minimum distance from point p to the line segment ab."""
    # Vector from a to b
    ab = np.array(b) - np.array(a)
    ap = np.array(p) - np.array(a)
    
    # Project point p onto line segment ab and clamp to segment
    t = np.dot(ap, ab) / np.dot(ab, ab)
    t = max(0, min(1, t))  # Clamp t to the range [0, 1]
    
    # Find the closest point on the segment
    closest_point = np.array(a) + t * ab
    # Return the distance between p and this closest point
    return (closest_point, np.linalg.norm(np.array(p) - closest_point))


def outline_route(start_node, target_node, route):
    #choose node edge (not already part of route) closest to agent
    #continue until direction to target is free from node
    
    # route = [node]
    route.append(start_node)
    find_outline_segment(start_node, target_node, route)

def direction_to_target_free(node, target_node, dist):
    line_x = node.coordinate[0]
    line_y = node.coordinate[1]
    dx = target_node.coordinate[0]- line_x
    dy = target_node.coordinate[1] - line_y
    d = dist if dist else math.sqrt(dx * dx + dy * dy)
    stepSize = 1
    nSteps = math.ceil(d / stepSize)
    stepX = dx / nSteps
    stepY = dy / nSteps

    free = True

    for s in range(nSteps):
        for cell in red_cells:
            if cell != node and cell.contains((line_x, line_y)):
                free = False

        line_x += stepX
        line_y += stepY
    return free
    
def draw_line_and_intersection(node, node2, route):
    intersection = find_agent_target_node_intersection(node, node2)

    outline_route(intersection, node2, route)

def first_last_intersection_midpoint(node, node2):
    first_intersection = find_agent_target_coordinate_intersection(node, node2)
    last_intersection = find_agent_target_coordinate_intersection(node2, node)
    return ((first_intersection[0] + last_intersection[0])/2, (first_intersection[1] + last_intersection[1])/2)
    


# def find_outline_segment(agent, target, node, route):
#     if direction_to_target_free(node, target):
#         return True
#     d = math.inf
#     bestNode = None
#     bestpt = None
#     for vn in node.vectorNodes:
#         (ptA, agent_to_segment) = point_to_segment_distance(agent.coordinate, node.coordinate, vn.coordinate)
#         (ptT, target_to_segment) = point_to_segment_distance(target.coordinate, node.coordinate, vn.coordinate)
#         if vn not in route:
#             if agent_to_segment < target_to_segment:
#                 if agent_to_segment < d:
#                     d = agent_to_segment
#                     bestNode = vn
#                     bestpt = ptA
#             else:
#                 if target_to_segment < d:
#                     d = target_to_segment
#                     bestNode = vn
#                     bestpt = ptT
#     if not bestNode:
#         return False
#     ax.plot(bestpt[0], bestpt[1], "co", markersize=10)
#     route.append(bestNode)
#     for (an, bn, l) in lines:
#         if (an,bn) == (node, bestNode) or (an,bn) == (bestNode, node):
#             lines.remove((an,bn,l))
#             l.remove()
#             line, = ax.plot([an.coordinate[0], bn.coordinate[0]], [an.coordinate[1], bn.coordinate[1]], 'c-')
#             lines.append((an,bn,line))
#             plt.draw()
#     if not find_outline_segment(agent, target, bestNode, route):
#         find_outline_segment(agent, target, node, route)
#     else:
#         return True

# def find_outline_segment(start_node, target_node, node, route):
#     if direction_to_target_free(node, target_node):
#         return
#     d = math.inf
#     bestNode = None
#     bestpt = None
#     for vn in node.vectorNodes:
#         (ptA, start_node_to_segment) = point_to_segment_distance(start_node.coordinate, node.coordinate, vn.coordinate)
#         (ptT, target_node_to_segment) = point_to_segment_distance(target_node.coordinate, node.coordinate, vn.coordinate)
#         if vn not in route:
#             if start_node_to_segment < target_node_to_segment:
#                 if start_node_to_segment < d:
#                     d = start_node_to_segment
#                     bestNode = vn
#                     bestpt = ptA
#             else:
#                 if target_node_to_segment < d:
#                     d = target_node_to_segment
#                     bestNode = vn
#                     bestpt = ptT
#     #If no further option, make new line from this node to target
#     if not bestNode:
#         draw_line_and_intersection(node, target_node)
#         return
    
#     #else we continue from the best node
#     ax.plot(bestpt[0], bestpt[1], "co", markersize=10)
#     route.append(bestNode)
#     for (an, bn, l) in lines:
#         if (an,bn) == (node, bestNode) or (an,bn) == (bestNode, node):
#             lines.remove((an,bn,l))
#             l.remove()
#             line, = ax.plot([an.coordinate[0], bn.coordinate[0]], [an.coordinate[1], bn.coordinate[1]], 'c-')
#             lines.append((an,bn,line))
#             plt.draw()
#             break

#     find_outline_segment(start_node, target_node, bestNode, route)

def find_outline_segment(node, target_node, route):
    if direction_to_target_free(node, target_node, None):
        draw_target_agent_line(node, target_node)
        return
    
    d_max = -math.inf
    d_min = math.inf
    bestNode = None
    bestpt = None
    for vn in node.vectorNodes:
        halfway_pt = ( (node.coordinate[0] + vn.coordinate[0])/2, (node.coordinate[1] + vn.coordinate[1])/2)
        (ptH, edge_to_segment) = point_to_segment_distance(halfway_pt, agent.coordinate, target.coordinate)
        
        target_to_hpt = distance.euclidean(target.coordinate, halfway_pt)
        
        s = target_to_hpt 
        if vn not in route:
            # if node_to_segment <=1.5:
                # if agent_to_hpt < target_to_hpt:
                #     if target_to_hpt > d_max:
                #         d_max = target_to_hpt
                #         bestNode = vn
                #         bestpt = halfway_pt
                # else:
                #     if agent_to_hpt > d_max:
                #         d_max = agent_to_hpt
                #         bestNode = vn
                #         bestpt = halfway_pt
            if s < d_min:
                d_min = s
                bestNode = vn
                bestpt = halfway_pt
            # else:
            #     if edge_to_segment > d_max:
            #         d_max = edge_to_segment
            #         bestNode = vn
            #         bestpt = halfway_pt
            # if mid_to_segment > d_max:
            #     d_max = mid_to_segment
            #     bestNode = vn
            #     bestpt = halfway_pt
    #If no further option, make new line from this node to target
    if not bestNode:
        draw_line_and_intersection(node, target_node, route)
        return
    
    #else we continue from the best node
    ax.plot(bestpt[0], bestpt[1], "co", markersize=10)
    route.append(bestNode)
    for (an, bn, l) in lines:
        if (an,bn) == (node, bestNode) or (an,bn) == (bestNode, node):
            lines.remove((an,bn,l))
            l.remove()
            line, = ax.plot([an.coordinate[0], bn.coordinate[0]], [an.coordinate[1], bn.coordinate[1]], 'c-')
            lines.append((an,bn,line))
            plt.draw()
            break

    find_outline_segment(bestNode, target_node, route)

def add_free_cell(coordinate):
    (x,y) = coordinate
    cell = Node((x,y))

    # If the cell is already free, do nothing
    if cell in free_cells:
        return
    
    if cell in [agent, target]:
        return
    
    free_cells.append(cell)
    # Connect the cell to nearby red cells within distance e
    for other_cell in free_cells:
        if other_cell != cell and distance.euclidean(cell.coordinate, other_cell.coordinate) <= initial_line_dist:
            draw_line(cell, other_cell)

    # Check for shortcut detection
    # check_shortcuts(cell)

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
    free_cells.remove(cell)
    ax.plot(cell.coordinate[0], cell.coordinate[1], "rs", markersize=cellplotsize, alpha=0.5)

    # Connect the cell to nearby red cells within distance e
    for other_cell in free_cells:
        if other_cell != cell and distance.euclidean(cell.coordinate, other_cell.coordinate) <= initial_line_dist:
            for (an, bn, l) in lines:
                if (an,bn) == (cell, other_cell) or (an,bn) == (other_cell, cell):
                    lines.remove((an,bn,l))
                    an.vectorNodes.remove(bn)
                    bn.vectorNodes.remove(an)
                    l.remove()
    
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
    else:
        x, y = math.floor(event.xdata)+0.5, math.floor(event.ydata)+0.5
        add_cell((x,y))
    plt.draw()

    if agent and target:
        draw_target_agent_line(agent, target)
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

def preset_agent_target(self):
    set_agent_coordinate(Node(ac))
    set_target_coordinate(Node(tc))
    draw_target_agent_line(agent, target)
    draw_line_and_intersection(agent ,target, [])


# Connect the click event to the handler
cid = fig.canvas.mpl_connect('button_press_event', on_click)
ax.set_aspect('equal', adjustable='box')

axpreset = fig.add_axes([0, 0.65, 0.1, 0.075])
axconnect = fig.add_axes([0, 0.50, 0.1, 0.075])
axtarget = fig.add_axes([0, 0.35, 0.1, 0.075])
axagent = fig.add_axes([0, 0.2, 0.1, 0.075])
axpreset_ta = fig.add_axes([0, 0.05, 0.1, 0.075])

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
