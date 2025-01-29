import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import csv

actual_arena_width = 10
actual_arena_height = 10
actual_arena = patches.Rectangle((-actual_arena_width / 2, -actual_arena_height / 2), actual_arena_width, actual_arena_height, linewidth=0, edgecolor='r', facecolor='blue', alpha=0.5)

def read_arena(filename):
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
        angle = float(orientation[2])  # Assuming rotation around the Z-axis

        boxes.append({
            'width': width,
            'height': height,
            'x': x,
            'y': y,
            'angle': angle
        })
    return boxes

def read_quadtree(filename):
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        data = [row for row in reader]
    return data

def calculate_precision_recall(arena_boxes, quadtree_data):
    true_positives = 0
    false_positives = 0
    false_negatives = 0

    agent_id = 'pipuck1'  # Replace with the actual agent ID you want to consider

    arena_rectangles = []
    for arena_box in arena_boxes:
        rect = patches.Rectangle(
            (arena_box['x'] - arena_box['width'] / 2, arena_box['y'] - arena_box['height'] / 2),
            arena_box['width'], arena_box['height'],
            linewidth=0, facecolor='gray', alpha=0.5,
            angle=arena_box['angle']
        )
        arena_rectangles.append(rect)

    for row in quadtree_data:
        if row['agent_id'] != agent_id:
            continue

        box_size = float(row['box_size'])
        box_x = float(row['box_x']) - box_size / 2
        box_y = float(row['box_y']) - box_size / 2
        pheromone = float(row['pheromone'])

        box_rect = patches.Rectangle((box_x, box_y), box_size, box_size)
        #If the entire box is outside the actual arena, ignore it
        if not actual_arena.get_bbox().fully_overlaps(box_rect.get_bbox()):
            continue

        box_contains_arena = any(
            box_rect.get_bbox().overlaps(arena_rect.get_bbox())
            for arena_rect in arena_rectangles
        )

        if box_contains_arena and pheromone < 0.5:
            true_positives += 1 # Correctly identified as occupied
        elif box_contains_arena and pheromone >= 0.5:
            false_negatives += 1 # Incorrectly identified as free (actually occupied)
        elif not box_contains_arena and pheromone >= 0.5:
            true_positives += 1 # Correctly identified as free
        else:
            false_positives += 1 # Incorrectly identified as occupied (actually free)

    precision = true_positives / (true_positives + false_positives)
    recall = true_positives / (true_positives + false_negatives)
    print("True Positives: ", true_positives)
    print("False Positives: ", false_positives)
    print("False Negatives: ", false_negatives)

    return precision, recall

def plot_mistakes(arena_boxes, quadtree_data):
    fig, ax = plt.subplots()
    arena_rectangles = []

    for arena_box in arena_boxes:
        rect = patches.Rectangle(
            (arena_box['x'] - arena_box['width'] / 2, arena_box['y'] - arena_box['height'] / 2),
            arena_box['width'], arena_box['height'],
            linewidth=0, facecolor='gray', alpha=0.5,
            angle=arena_box['angle']
        )
        arena_rectangles.append(rect)
        ax.add_patch(rect)

    agent_id = 'pipuck1'  # Replace with the actual agent ID you want to consider

    for row in quadtree_data:
        if row['agent_id'] != agent_id:
            continue
        box_size = float(row['box_size'])
        box_x = float(row['box_x']) - box_size / 2
        box_y = float(row['box_y']) - box_size / 2
        pheromone = float(row['pheromone'])

        box_rect = patches.Rectangle((box_x, box_y), box_size, box_size)

        # #If the entire box is outside the actual arena, ignore it
        # if not actual_arena.get_bbox().fully_overlaps(box_rect.get_bbox()):
        #     continue

        if not actual_arena.get_bbox().fully_overlaps(box_rect.get_bbox()):
            continue

        box_contains_arena = any(
            box_rect.get_bbox().overlaps(arena_rect.get_bbox())
            for arena_rect in arena_rectangles
        )
        if box_contains_arena and pheromone < 0.5:
            color = 'red'
        elif (not box_contains_arena) and pheromone >= 0.5:
            color = 'green'
        elif box_contains_arena and pheromone >= 0.5:
            color = 'pink'
        else:
            color = 'yellow'

        # if (box_contains_arena and pheromone >= 0.5) or (not box_contains_arena and pheromone < 0.5):
        #     color = 'red'
        # else:
        #     color = 'green'
        # color = (1 - pheromone, pheromone, 0)  # Red to Green gradient

        rect = plt.Rectangle((box_x, box_y), box_size, box_size, color=color, alpha=1)
        ax.add_patch(rect)

    ax.set_aspect('equal', 'box')
    plt.xlim(-5.5, 5.5)
    plt.ylim(-5.5, 5.5)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Mistakes in Quadtree Results')

    plt.grid(True)
    plt.show()

# Usage
arena_boxes = read_arena('implementation_and_examples/experiments/hugo_experiment.argos')
quadtree_data = read_quadtree('implementation_and_examples/experiment_results/quadtree.csv')
precision, recall = calculate_precision_recall(arena_boxes, quadtree_data)
print(f'Precision: {precision:.4f}, Recall: {recall:.4f}')
plot_mistakes(arena_boxes, quadtree_data)