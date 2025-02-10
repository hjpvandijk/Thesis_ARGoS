import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# def plot_arena(filename):
#     tree = ET.parse(filename)
#     root = tree.getroot()

#     # Find the arena element
#     arena = root.find('arena')

#     # Create a plot
#     fig, ax = plt.subplots()

#     # Extract and plot each box
#     for box in arena.findall('box'):
#         size = box.get('size').split(',')
#         position = box.find('body').get('position').split(',')
#         orientation = box.find('body').get('orientation').split(',')

#         height = float(size[0])
#         width = float(size[1])
#         y = float(position[0])#5
#         x = -float(position[1])#0
#         angle = float(orientation[2]) #+ 90  # Assuming rotation around the Z-axis

#         # Create a rectangle
#         rect = patches.Rectangle((x-width / 2, y-height / 2), width, height, linewidth=1, edgecolor='r', facecolor='gray', alpha=0.5, angle=angle)
#         # Apply rotation
#         # t = patches.transforms.Affine2D().rotate_deg(angle) + patches.transforms.Affine2D().translate(x, y) + ax.transData
#         # rect.set_transform(t)
#         #print the coordinates of the transformed rectangle
#         # print(x, y)
#         # print(rect.get_xy())
#         # print(rect.get_width())
#         # print(rect.get_height())

#         ax.add_patch(rect)
#         # break

#     # Set plot limits and labels
#     ax.set_aspect('equal', 'box')
#     plt.xlim(-10.5, 10.5)
#     plt.ylim(-5.7, 5.7)
#     plt.xlabel('X-axis')
#     plt.ylabel('Y-axis')
#     plt.title('Arena Configuration')

#     plt.grid(True)
#     plt.show()

# # Usage
# plot_arena('implementation_and_examples/experiments/hugo_experiment.argos')

def read_arena_boxes(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    arena = root.find('arena')
    boxes = []
    for box in arena.findall('box'):
        size = box.get('size').split(',')
        position = box.find('body').get('position').split(',')
        orientation = box.find('body').get('orientation').split(',')

        height = float(size[0])
        width = float(size[1])
        y = float(position[0])
        x = -float(position[1])
        angle = float(orientation[0])  # Assuming rotation around the Z-axis

        boxes.append({
            'width': width,
            'height': height,
            'x': x,
            'y': y,
            'angle': angle
        })
    return boxes

def read_arena_cylinders(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    arena = root.find('arena')
    cylinders = []
    for cylinder in arena.findall('cylinder'):
        radius = float(cylinder.get('radius'))
        position = cylinder.find('body').get('position').split(',')

        y = float(position[0])
        x = -float(position[1])

        cylinders.append({
            'radius': radius,
            'x': x,
            'y': y
        })
    return cylinders


arena_boxes = read_arena_boxes('implementation_and_examples/experiments/office_config.argos')
arena_cylinders = read_arena_cylinders('implementation_and_examples/experiments/office_config.argos')
fig, ax = plt.subplots()

arena_rectangles = []
for arena_box in arena_boxes:
    angle_rad = np.radians(arena_box['angle'])
    translation_x = arena_box['width'] /2 * (1-np.cos(angle_rad)) + arena_box['height'] /2 * np.sin(angle_rad)
    translation_y = arena_box['height'] /2 * (1-np.cos(angle_rad)) - arena_box['width'] /2 * np.sin(angle_rad)
    rect = patches.Rectangle(
        (arena_box['x'] - arena_box['width'] / 2 + translation_x, arena_box['y'] - arena_box['height'] / 2 + translation_y),
        arena_box['width'], arena_box['height'],
        linewidth=0, facecolor='gray', alpha=0.5,
        angle=arena_box['angle']
    )
    arena_rectangles.append(rect)

for arena_cylinder in arena_cylinders:
    circle = patches.Circle(
        (arena_cylinder['x'], arena_cylinder['y']),
        arena_cylinder['radius'],
        linewidth=0, facecolor='gray', alpha=0.5
    )
    ax.add_patch(circle)

#plot arena rectangles
for rect in arena_rectangles:
    ax.add_patch(rect)

# ax.set_aspect('equal', 'box')
plt.xlim(-10.5, 10.5)
plt.ylim(-5.6, 5.6)
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Mistakes in Quadtree Results')

plt.grid(True)
plt.show()