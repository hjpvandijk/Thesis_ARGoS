import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import csv

actual_arena_width = 9.5
actual_arena_height = 12
actual_arena = patches.Rectangle((-actual_arena_width / 2, -actual_arena_height / 2), actual_arena_width, actual_arena_height, linewidth=0, edgecolor='r', facecolor='blue', alpha=0.5)

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

def read_file(filename):
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        data = [row for row in reader]
    return data

import numpy as np
import matplotlib.patches as patches

def get_rotated_corners(rect):
    """Returns the four corner points of a rotated rectangle."""
    x, y = rect.get_xy()  # Bottom-left corner before rotation
    w, h = rect.get_width(), rect.get_height()
    angle = np.radians(rect.angle)  # Convert degrees to radians

    # Define the 4 rectangle corners relative to the bottom-left
    corners = np.array([
        [0, 0],    # Bottom-left
        [w, 0],    # Bottom-right
        [w, h],    # Top-right
        [0, h]     # Top-left
    ])

    # Rotation matrix
    rotation_matrix = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])

    # Rotate and translate corners
    rotated_corners = (rotation_matrix @ corners.T).T + np.array([x, y])
    return rotated_corners

def project_polygon(axis, corners):
    """Projects a polygon onto an axis and returns min & max projections."""
    projections = np.dot(corners, axis)
    return np.min(projections), np.max(projections)

def check_rectangle_overlap(rect1, rect2):
    """Checks if two rotated rectangles overlap using the Separating Axis Theorem (SAT)."""
    # Get corners of both rectangles
    corners1 = get_rotated_corners(rect1)
    corners2 = get_rotated_corners(rect2)

    # Get edge normals (perpendicular to edges)
    edges = np.vstack([np.diff(corners1, axis=0), np.diff(corners2, axis=0)])
    normals = np.array([-edges[:, 1], edges[:, 0]]).T  # Perpendicular vectors

    # Normalize normals
    normals = normals / np.linalg.norm(normals, axis=1, keepdims=True)

    # SAT: Check for separating axis
    for axis in normals:
        min1, max1 = project_polygon(axis, corners1)
        min2, max2 = project_polygon(axis, corners2)

        # If projections do not overlap, there is a separating axis → No collision
        if max1 < min2 or max2 < min1:
            return False

    return True  # Overlap exists

def check_circle_rectangle_overlap(circle, rectangle):
    """Check if a matplotlib.patches.Circle and matplotlib.patches.Rectangle overlap"""

    # Extract circle properties
    circle_center = np.array(circle.center)
    circle_radius = circle.radius

    # Extract rectangle properties
    rect_x, rect_y = rectangle.get_xy()  # Bottom-left corner
    rect_width = rectangle.get_width()
    rect_height = rectangle.get_height()

    # Get rectangle bounding box points
    rect_x1, rect_y1 = rect_x + rect_width, rect_y + rect_height  # Top-right corner

    # Step 1: Bounding Box Check
    if not (rect_x <= circle_center[0] <= rect_x1 and rect_y <= circle_center[1] <= rect_y1):
        # Check if bounding boxes overlap
        circle_bbox = patches.Rectangle((circle.center[0] - circle.radius, circle.center[1] - circle.radius), 2 * circle.radius, 2 * circle.radius).get_extents()
        rect_bbox = rectangle.get_extents()
        if not circle_bbox.overlaps(rect_bbox):
            return False  # No overlap

    # Step 2: Precise Check - Find the closest point on the rectangle to the circle
    closest_x = np.clip(circle_center[0], rect_x, rect_x1)
    closest_y = np.clip(circle_center[1], rect_y, rect_y1)

    # Compute the distance from the closest point to the circle center
    distance = np.linalg.norm(circle_center - np.array([closest_x, closest_y]))

    # Check if the closest point is inside the circle
    return distance <= circle_radius

def plot_arena(arena_boxes, arena_cylinders):
    fig, ax = plt.subplots()

    arena_rectangles = []
    for arena_box in arena_boxes:
        angle_rad = np.radians(arena_box['angle'])
        translation_x = arena_box['width'] /2 * (1-np.cos(angle_rad)) + arena_box['height'] /2 * np.sin(angle_rad)
        translation_y = arena_box['height'] /2 * (1-np.cos(angle_rad)) - arena_box['width'] /2 * np.sin(angle_rad)
        rect = patches.Rectangle(
            (arena_box['x'] - arena_box['width'] / 2 + translation_x, arena_box['y'] - arena_box['height'] / 2 + translation_y),
            arena_box['width'], arena_box['height'],
            linewidth=0, facecolor='gray', alpha=1,
            angle=arena_box['angle']
        )
        arena_rectangles.append(rect)
        ax.add_patch(rect)


    arena_circles = []
    for arena_cylinder in arena_cylinders:
        circle = patches.Circle(
            (arena_cylinder['x'], arena_cylinder['y']),
            arena_cylinder['radius'],
            linewidth=0, facecolor='gray', alpha=1
        )
        arena_circles.append(circle)
        ax.add_patch(circle)

    ax.set_aspect('equal', 'box')
    plt.xlim(-actual_arena_width / 2, actual_arena_width / 2)
    plt.ylim(-actual_arena_height / 2, actual_arena_height / 2)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Plotted Arena')

    plt.grid(True)
    plt.show()



# Usage
arena_boxes = read_arena_boxes('implementation_and_examples/experiments/house.argos')
arena_cylinders = read_arena_cylinders('implementation_and_examples/experiments/house.argos')

plot_arena(arena_boxes, arena_cylinders)