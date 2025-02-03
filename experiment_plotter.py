import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def plot_arena(filename):
    tree = ET.parse(filename)
    root = tree.getroot()

    # Find the arena element
    arena = root.find('arena')

    # Create a plot
    fig, ax = plt.subplots()

    # Extract and plot each box
    for box in arena.findall('box'):
        size = box.get('size').split(',')
        position = box.find('body').get('position').split(',')
        orientation = box.find('body').get('orientation').split(',')

        height = float(size[0])
        width = float(size[1])
        y = float(position[0])#5
        x = -float(position[1])#0
        angle = float(orientation[2]) #+ 90  # Assuming rotation around the Z-axis

        # Create a rectangle
        rect = patches.Rectangle((x-width / 2, y-height / 2), width, height, linewidth=1, edgecolor='r', facecolor='gray', alpha=0.5, angle=angle)
        # Apply rotation
        # t = patches.transforms.Affine2D().rotate_deg(angle) + patches.transforms.Affine2D().translate(x, y) + ax.transData
        # rect.set_transform(t)
        #print the coordinates of the transformed rectangle
        # print(x, y)
        # print(rect.get_xy())
        # print(rect.get_width())
        # print(rect.get_height())

        ax.add_patch(rect)
        # break

    # Set plot limits and labels
    ax.set_aspect('equal', 'box')
    plt.xlim(-5.5, 5.5)
    plt.ylim(-5.5, 5.5)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Arena Configuration')

    plt.grid(True)
    plt.show()

# Usage
plot_arena('implementation_and_examples/experiments/hugo_experiment.argos')