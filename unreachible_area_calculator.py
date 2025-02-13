import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import csv

actual_arena_width = 20
actual_arena_height = 10.2
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
    # Sort corners to ensure they are in the order: bottom left, bottom right, top right, top left
    sorted_corners = sorted(rotated_corners, key=lambda point: (point[1], point[0]))
    bottom_corners = sorted(sorted_corners[:2], key=lambda point: point[0])
    top_corners = sorted(sorted_corners[2:], key=lambda point: point[0])
    ordered_corners = np.array([bottom_corners[0], bottom_corners[1], top_corners[1], top_corners[0]])
    return ordered_corners

def project_polygon(axis, corners):
    """Projects a polygon onto an axis and returns min & max projections."""
    projections = np.dot(corners, axis)
    return np.min(projections), np.max(projections)

def check_rectangle_overlap(rect1, rect2):
    """Checks if two rotated rectangles overlap using the Separating Axis Theorem (SAT)."""
    corners1 = get_rotated_corners(rect1)
    corners2 = get_rotated_corners(rect2)

    edges = np.vstack([np.diff(corners1, axis=0), np.diff(corners2, axis=0)])
    normals = np.array([-edges[:, 1], edges[:, 0]]).T
    normals = normals / np.linalg.norm(normals, axis=1, keepdims=True)

    for axis in normals:
        min1, max1 = project_polygon(axis, corners1)
        min2, max2 = project_polygon(axis, corners2)
        if max1 < min2 or max2 < min1:
            return False  
    return True

def is_rectangle_overlapping_circle(rect, circle):
    """Checks if a rotated rectangle overlaps a circle."""
    rect_corners = get_rotated_corners(rect)
    circle_center = np.array(circle.center)
    circle_radius = circle.radius

    for corner in rect_corners:
        if np.linalg.norm(corner - circle_center) <= circle_radius:
            return True

    return False

def is_point_inside_rectangle(point, rect):
    """Checks if a point is inside a rotated rectangle."""
    corners = get_rotated_corners(rect)
    x, y = point
    inside = False
    px, py = corners[-1]
    for cx, cy in corners:
        if ((cy > y) != (py > y)) and (x < (px - cx) * (y - cy) / (py - cy) + cx):
            inside = not inside
        px, py = cx, cy
    return inside

def is_rectangle_fully_covered(cell, boxes, circles):
    """Checks if a rectangle is fully covered by boxes or circles."""
    corners = get_rotated_corners(cell)
    
    for corner in corners:
        covered = False
        for box in boxes:
            if is_point_inside_rectangle(corner, box):
                covered = True
                break
        if not covered:
            for circle in circles:
                if np.linalg.norm(corner - np.array(circle.center)) <= circle.radius:
                    covered = True
                    break
        if not covered:
            return False  # At least one point is uncovered
    return True  # All points are covered

def select_valid_cells(cells, boxes, circles):
    """Selects all cells that overlap with a box or circle but are not fully covered."""
    valid_cells = []
    
    for cell in cells:
        overlaps = any(check_rectangle_overlap(cell, box) for box in boxes) or \
                   any(is_rectangle_overlapping_circle(cell, circle) for circle in circles)
        
        if overlaps and not is_rectangle_fully_covered(cell, boxes, circles) or not overlaps:
            valid_cells.append(cell)
    
    return valid_cells

def calculate_unreachible_areas(arena_boxes, arena_cylinders, quadtree_data):
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
        ax.add_patch(rect)


    arena_circles = []
    for arena_cylinder in arena_cylinders:
        circle = patches.Circle(
            (arena_cylinder['x'], arena_cylinder['y']),
            arena_cylinder['radius'],
            linewidth=0, facecolor='gray', alpha=0.5
        )
        arena_circles.append(circle)
        ax.add_patch(circle)

    resolution = 0.164062  # Define the resolution of the grid
    x_coords = np.arange(0, actual_arena_width / 2, resolution)
    negative_x_coords = (-np.flip(x_coords)) - resolution  
    x_coords = np.concatenate((negative_x_coords, x_coords))
    y_coords = np.arange(0, actual_arena_height / 2, resolution)
    negative_y_coords = (-np.flip(y_coords)) - resolution
    y_coords = np.concatenate((negative_y_coords, y_coords))

    cells = []

    total_cells = x_coords.size * y_coords.size

    for i,x in enumerate(x_coords):
        for j,y in enumerate(y_coords):
            # if x < 4.6 or x > 5.6 or y < 0 or y > 1.1:
            #     continue
            cell = patches.Rectangle((x, y), resolution, resolution, facecolor='red', alpha=0.5)
            cells.append(cell)
            # ax.add_patch(patches.Rectangle((x, y), resolution, resolution, linewidth=1, facecolor='none', edgecolor='black'))

    


    valid_cells = select_valid_cells(cells, arena_rectangles, arena_circles)

    non_valid_cells = [cell for cell in cells if cell not in valid_cells]

    for cell in non_valid_cells:
        ax.add_patch(cell)

    non_valid_area = non_valid_cells.__len__() * resolution**2
    print(f'Non-valid area: {non_valid_area:f} m^2')

    ax.set_aspect('equal', 'box')
    plt.xlim(-10.5, 10.5)
    plt.ylim(-5.6, 5.6)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Mistakes in Quadtree Results')

    plt.grid(True)
    plt.show()
    return precision, recall

# Usage
arena_boxes = read_arena_boxes('implementation_and_examples/experiments/office_config.argos')
arena_cylinders = read_arena_cylinders('implementation_and_examples/experiments/office_config.argos')

quadtree_data = read_file('implementation_and_examples/experiment_results/experiment/quadtree.csv')
precision, recall = calculate_unreachible_areas(arena_boxes, arena_cylinders, quadtree_data)
# print(f'Precision: {precision:.4f}, Recall: {recall:.4f}')
# plot_mistakes(arena_boxes, quadtree_data)