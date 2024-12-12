import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
from scipy.spatial import distance
import math

# Implemented based on https://hal.science/hal-02150278/document

# Set parameters
initial_line_dist = 1.5  # Distance threshold to connect red cells
shortcut_dist = 1
close_ends_dist = 1.5

class Node:
    def __init__(self, coordinate):
        self.coordinate = coordinate
        self.vectorNodes = []

    def __eq__(self, other):
        if isinstance(other, Node):
            return self.coordinate == other.coordinate
        return False

    def __ne__(self, other):
        return not self.__eq__(other)

# Store the positions of red cells and lines
red_cells = []
lines = []
# linecells = []
to_be_deleted = []
to_be_added = []

potential_shortcuts = []

# Create a grid
fig, ax = plt.subplots(figsize=(10,10))
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_xticks(np.arange(0, 11, 1))
ax.set_yticks(np.arange(0, 11, 1))
ax.grid(True)

def draw_line(node, other_node):
    """Draw a line between two red cells c1 and c2."""
    line, = ax.plot([node.coordinate[0], other_node.coordinate[0]], [node.coordinate[1], other_node.coordinate[1]], 'b-')
    #if line doesn't exist yet, add it to each node
    if (other_node not in node.vectorNodes):
        node.vectorNodes.append(other_node)
        other_node.vectorNodes.append(node)
        lines.append((node, other_node, line))

def remove_cell(c):
    for (a, b, l) in lines[:]:
        if a==c or b==c:
            delete_line((a,b,l))
    # red_cells.remove(c)

def delete_line(line_data):
    """Delete a line."""
    node1, node2, line = line_data

    
    if ((node1, node2, line) in lines):
        lines.remove((node1, node2, line)) #both ways because we don't know which way was added
        node1.vectorNodes.remove(node2)
        node2.vectorNodes.remove(node1)
        line.remove()
    elif ((node2, node1, line) in lines):
        lines.remove((node2, node1, line))
        node1.vectorNodes.remove(node2)
        node2.vectorNodes.remove(node1)
        line.remove() 

def check_successive_vertices(a, b, c, l, l2):
    # if ((a,c) or (c,a)) in linecells:
    #     return False

    Aarray = np.asarray(a.coordinate)
    Barray = np.asarray(b.coordinate)
    Carray = np.asarray(c.coordinate)

    p1 = Carray - Aarray
    p2 = Aarray - Barray
    p3 = Carray - Aarray

    npc = np.cross(Carray - Aarray, Aarray - Barray)
    npn = np.linalg.norm(Carray - Aarray)
    n = np.linalg.norm(np.cross(Carray - Aarray, Aarray - Barray))/np.linalg.norm(Carray - Aarray)


    d = np.linalg.norm(np.cross(Carray - Aarray, Aarray - Barray))/np.linalg.norm(Carray - Aarray)
    if d < shortcut_dist:
        # delete_line((a, b, l))
        # delete_line((b,a,l))
        # delete_line((b, c, l2))
        # delete_line((c,b,l2))
        # remove_cell(b)
        # draw_line(a, c)  # Add the shortcut

        to_be_deleted.append((a,b,l))
        # to_be_deleted.append((b,a,l))
        to_be_deleted.append((b,c,l2))
        # to_be_deleted.append((c,b,l2))
        # to_be_deleted.append(b)
        if a not in c.vectorNodes:
            to_be_added.append((a,c))

            return True

    return False

def check_successive_vertices(a, b, c):
    # if ((a,c) or (c,a)) in linecells:
    #     return False

    Aarray = np.asarray(a.coordinate)
    Barray = np.asarray(b.coordinate)
    Carray = np.asarray(c.coordinate)

    p1 = Carray - Aarray
    p2 = Aarray - Barray
    p3 = Carray - Aarray

    npc = np.cross(Carray - Aarray, Aarray - Barray)
    npn = np.linalg.norm(Carray - Aarray)
    n = np.linalg.norm(np.cross(Carray - Aarray, Aarray - Barray))/np.linalg.norm(Carray - Aarray)


    d = np.linalg.norm(np.cross(Carray - Aarray, Aarray - Barray))/np.linalg.norm(Carray - Aarray)
    if d > 0 and d <= shortcut_dist:
        for (n1,n2,l) in lines:
            if (a,b) == (n1,n2) or (a,b) == (n2,n1):
                to_be_deleted.append((a,b,l))
            if (b,c) == (n1,n2) or (b,c) == (n2,n1):
                to_be_deleted.append((b,c,l))
        if a not in c.vectorNodes:
            to_be_added.append((a,c))
            return True

    return False

def find_other_route(a,c):
    for an in a.vectorNodes:
        for ann in an.vectorNodes:
            if ann == c:
                for (n1,n2,l) in lines:
                    if (a,an) == (n1,n2) or (a,an) == (n2,n1):
                        to_be_deleted.append((a,an,l))
                    if (an,ann) == (n1,n2) or (an,ann) == (n2,n1):
                        to_be_deleted.append((an,ann,l))




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

def iterate_all():
    for node in red_cells:
        if len(node.vectorNodes) in [1,2]:
            for vn in node.vectorNodes:
                # for vn2 in vn.vectorNodes:
                #     if node!=vn2: #vn = v2
                #         if check_successive_vertices(node,vn,vn2):
                #             return
                for vn3 in node.vectorNodes:
                    if vn != vn3: #node = v2
                        if check_successive_vertices(vn,node,vn3):
                            find_other_route(vn,vn3)
                            return

def iterate(node):
    for vn in node.vectorNodes:
        for vn2 in vn.vectorNodes:
            if node!=vn2: #vn = v2
                if check_successive_vertices(node,vn,vn2):
                    return
        for vn3 in node.vectorNodes:
            if vn != vn3: #node = v2
                if check_successive_vertices(vn,node,vn3):
                    return



def check_shortcuts_btn(self):
    """Check and remove shortcut lines."""
    iterate_all()
    for (ad,bd,ld) in to_be_deleted:
        delete_line((ad,bd,ld))
    # for cell in to_be_deleted:
    #     remove_cell(cell)
    to_be_deleted.clear()
    for (aa,ca) in to_be_added:
        draw_line(aa,ca)
    to_be_added.clear()
    plt.draw()

def check_shortcuts(node):
    """Check and remove shortcut lines."""
    iterate(node)
    for (ad,bd,ld) in to_be_deleted:
        delete_line((ad,bd,ld))
    # for cell in to_be_deleted:
    #     remove_cell(cell)
    to_be_deleted.clear()
    for (aa,ca) in to_be_added:
        draw_line(aa,ca)
    to_be_added.clear()
    plt.draw()


def on_click(event):
    """Handle mouse click events to add red cells and connect lines."""
    if event.inaxes != ax:
        return

    # Get the coordinates of the click and round them to the nearest grid point
    x, y = math.floor(event.xdata)+0.5, math.floor(event.ydata)+0.5
    cell = Node((x, y))

    # If the cell is already red, do nothing
    if cell in red_cells:
        return

    # Add the cell to the list of red cells and plot it
    red_cells.append(cell)
    ax.plot(x, y, "rs", markersize=55)

    # Connect the cell to nearby red cells within distance e
    for other_cell in red_cells:
        if other_cell != cell and distance.euclidean(cell.coordinate, other_cell.coordinate) <= initial_line_dist:
            draw_line(cell, other_cell)

    # Check for shortcut detection
    # check_shortcuts(cell)

    plt.draw()

# Connect the click event to the handler
cid = fig.canvas.mpl_connect('button_press_event', on_click)
ax.set_aspect('equal', adjustable='box')

axshortcut = fig.add_axes([0, 0.05, 0.1, 0.075])
axconnect = fig.add_axes([0, 0.2, 0.1, 0.075])

bshortcut = Button(axshortcut, 'Check Schortcuts')
bshortcut.on_clicked(check_shortcuts_btn)

bconnect = Button(axconnect, 'Connect Close Ends')
bconnect.on_clicked(connect_close_ends)

plt.show()
