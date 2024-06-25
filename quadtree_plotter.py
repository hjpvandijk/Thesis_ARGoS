import matplotlib.pyplot as plt

def plot_boxes(filename):
    # Read the file
    with open(filename, 'r') as file:
        lines = file.readlines()

    # Extract the data
    boxes = []
    for line in lines:
        parts = line.strip().split()
        if len(parts) == 3:
            left_x, top_y, size = map(float, parts)
            color = None
        elif len(parts) == 4:
            left_x, top_y, size, color = map(float, parts)
        else:
            continue

        boxes.append((left_x, top_y, size, color))

    # Plot the boxes
    fig, ax = plt.subplots()

    for box in boxes:
        left_x, top_y, size, color = box
        rect = plt.Rectangle((left_x, top_y), size, size, fill=False, edgecolor='black')

        if color == 1:
            # rect.set_edgecolor('pink')
            rect.fill=True
            rect.set_facecolor('green')
        elif color == 2:
            rect.fill=True
            # rect.set_edgecolor('pink')
            rect.set_facecolor('red')


        ax.add_patch(rect)

    ax.set_aspect('equal', 'box')
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Boxes Plot')

    plt.grid(True)
    plt.show()

# Usage
plot_boxes('pipuck1.txt')
