import math
import sys

def amantides_woo(box_size, c1, c2):
    points = []

    x1 = c1[0]
    y1 = c1[1]
    x2 = c2[0]
    y2 = c2[1]

    x = math.floor(x1/box_size)
    y = math.floor(y1/box_size)

    xEnd = math.floor(x2/box_size)
    yEnd = math.floor(y2/box_size)

    dx = x2-x1
    dy = y2-y1

    stepX = 1 if dx > 0 else -1
    stepY = 1 if dy > 0 else -1

    tMaxX = sys.float_info.max if dx == 0 else ((math.floor(x1/box_size)+1)*box_size - x1 if dx > 0 else x1 - math.floor(x1/box_size)*box_size) / abs(dx)
    tMaxY = sys.float_info.max if dy == 0 else ((math.floor(y1/box_size)+1)*box_size - y1 if dy > 0 else y1 - math.floor(y1/box_size)*box_size) / abs(dy)

    tDeltaX = sys.float_info.max if dx == 0 else box_size / abs(dx)
    tDeltaY = sys.float_info.max if dy == 0 else box_size / abs(dy)

    points.append(((x+0.5)*box_size, (y+0.5)*box_size))

    while x != xEnd or y != yEnd:
        if tMaxX < tMaxY:
            tMaxX += tDeltaX
            x += stepX
        else:
            tMaxY += tDeltaY
            y += stepY

        points.append(((x+0.5)*box_size, (y+0.5)*box_size))

    return points

box_size = 0.203125
start = (-1.7170290039795573, -2.7998086036802663)
target = (-1.625, -3.453125)

#Plot and visualize
import matplotlib.pyplot as plt

fig, ax = plt.subplots()
ax.set_xlim(-4, 4)
ax.set_ylim(-5, 4)

#Small offset towards the target, so the raytracing method selects the correct cell
start_with_offset = (start[0] + (target[0] - start[0]) / 1000, start[1] + (target[1] - start[1]) / 1000)
#Small offset towards the start, so the raytracing method selects the correct cell
target_with_offset = (target[0] + (start[0] - target[0]) / 1000, target[1] + (start[1] - target[1]) / 1000)

points = amantides_woo(box_size, start_with_offset, target_with_offset)

ax.plot([p[0] for p in points], [p[1] for p in points], 'ro-')
ax.plot(start[0], start[1], 'bo')
ax.plot(target[0], target[1], 'go')
ax.plot([start_with_offset[0], target_with_offset[0]], [start_with_offset[1], target_with_offset[1]], 'k--')

#plot lines every box_size
for i in range(-20, 20):
    ax.axvline(x=i*box_size, color='gray', linestyle='--')
    ax.axhline(y=i*box_size, color='gray', linestyle='--')

plt.show()