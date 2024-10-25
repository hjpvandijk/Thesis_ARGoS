import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
from scipy.spatial import distance
import math

# Implemented based on https://hal.science/hal-02150278/document

# Set parameters
e = 1.1  # Distance threshold to connect red cells

# Store the positions of red cells and lines
red_cells = []
lines = []
linecells = []
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

def draw_line(c1, c2):
    """Draw a line between two red cells c1 and c2."""
    line, = ax.plot([c1[0], c2[0]], [c1[1], c2[1]], 'b-')
    if ((c1,c2) and (c2,c1)) not in linecells:
        lines.append((c1, c2, line))
        # lines.append((c2, c1, line))
        linecells.append((c1,c2))
        # linecells.append((c2,c1))

def remove_cell(c):
    for (a, b, l) in lines[:]:
        if a==c or b==c:
            delete_line((a,b,l))
    red_cells.remove(c)

def delete_line(line_data):
    """Delete a line."""
    c1, c2, line = line_data
    if line_data in lines:
        lines.remove(line_data)
        linecells.remove((c1,c2))
        line.remove()

def check_successive_vertices(a, b, c, l, l2):
    # if ((a,c) or (c,a)) in linecells:
    #     return False

    Aarray = np.asarray(a)
    Barray = np.asarray(b)
    Carray = np.asarray(c)

    p1 = Carray - Aarray
    p2 = Aarray - Barray
    p3 = Carray - Aarray

    npc = np.cross(Carray - Aarray, Aarray - Barray)
    npn = np.linalg.norm(Carray - Aarray)
    n = np.linalg.norm(np.cross(Carray - Aarray, Aarray - Barray))/np.linalg.norm(Carray - Aarray)


    d = np.linalg.norm(np.cross(Carray - Aarray, Aarray - Barray))/np.linalg.norm(Carray - Aarray)
    if d < e:
        # delete_line((a, b, l))
        # delete_line((b,a,l))
        # delete_line((b, c, l2))
        # delete_line((c,b,l2))
        # remove_cell(b)
        # draw_line(a, c)  # Add the shortcut

        to_be_deleted.append((a,b,l))
        to_be_deleted.append((b,a,l))
        to_be_deleted.append((b,c,l2))
        to_be_deleted.append((c,b,l2))
        # to_be_deleted.append(b)
        if (a,c) not in linecells:
            to_be_added.append((a,c))

            return True

    return False

def connect_close_ends(self):
    cell_counts = np.zeros(len(red_cells))
    for i, cell in enumerate(red_cells):
        for (a, b, l) in lines[:]:
            if a==cell or b==cell:
                cell_counts[i] += 1
    for i, cell in enumerate(red_cells):
        for j, cell2 in enumerate(red_cells):
            if cell != cell2 and (cell_counts[i] == 1 and cell_counts[j] > 0 or cell_counts[j] == 1 and cell_counts[i] >0) and distance.euclidean(cell, cell2) < e:
                draw_line(cell, cell2)
    plt.draw()

def iterate():
    for (a, b, l) in lines[:]:  # Loop over lines from c1 to c2
        for (c, d, l2) in lines[:]:
            if l != l2:
                if a==c and b!=d:
                    if check_successive_vertices(b,a,d,l,l2):
                        return
                elif b==d and a!=c:
                    if check_successive_vertices(a,b,c,l,l2):
                        return
                elif a==d and b!=c:
                    if check_successive_vertices(b,a,c,l,l2):
                        return
                elif b==c and a!=b:
                    if check_successive_vertices(a,b,d,l,l2):
                        return


def check_shortcuts(self):
    """Check and remove shortcut lines."""
    iterate()
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
    cell = (x, y)

    # If the cell is already red, do nothing
    if cell in red_cells:
        return

    # Add the cell to the list of red cells and plot it
    red_cells.append(cell)
    ax.plot(x, y, "rs", markersize=55)

    # Connect the cell to nearby red cells within distance e
    for other_cell in red_cells:
        if other_cell != cell and distance.euclidean(cell, other_cell) < e:
            draw_line(cell, other_cell)

    # Check for shortcut detection
    # check_shortcuts()

    plt.draw()

# Connect the click event to the handler
cid = fig.canvas.mpl_connect('button_press_event', on_click)
ax.set_aspect('equal', adjustable='box')

axshortcut = fig.add_axes([0, 0.05, 0.1, 0.075])
axconnect = fig.add_axes([0, 0.2, 0.1, 0.075])

bshortcut = Button(axshortcut, 'Check Schortcuts')
bshortcut.on_clicked(check_shortcuts)

bconnect = Button(axconnect, 'Connect Close Ends')
bconnect.on_clicked(connect_close_ends)

plt.show()
