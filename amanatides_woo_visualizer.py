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

def is_multiple(a,b,epsilon=0.0000000001):
    remainder = a % b
    return remainder < epsilon or abs(remainder - b) < epsilon

box_size = 0.18124999999999999
print(box_size - 0.18125 < pow(10, -6))
start = (-4.3499999999999996, -0.64025461807926942)
target = (-4.3499999999999996, -1.0874999999999999)
import matplotlib.pyplot as plt


fig, ax = plt.subplots()
ax.set_xlim(min(start[0], target[0]) - 1, max(start[0], target[0]) + 1)
ax.set_ylim(min(start[1], target[1]) - 1, max(start[1], target[1]) + 1)
ax.plot(target[0], target[1], 'go')

#round
# target = (round(target[0]*1000000)/1000000), (round(target[1]*1000000)/1000000)
# start = (round(start[0]*1000000)/1000000), (round(start[1]*1000000)/1000000)

print(start)
print(target)

#Plot and visualize

# print((target[0] - start[0]) / 100000)
# print((target[1] - start[1]) / 100000)

# print((start[0] - target[0]) / 100000)
# print((start[1] - target[1]) / 100000)

while is_multiple(start[0], box_size) or is_multiple(start[1], box_size):
    addx = 0.000001
    if target[0] < start[0]:
        addx = -0.000001
    addy = 0.000001
    if target[1] < start[1]:
        addy = -0.000001
    start = (start[0] + addx, start[1] + addy)

while is_multiple(target[0], box_size) or is_multiple(target[1], box_size):
    addx = 0.000001
    if start[0] < target[0]:
        addx = -0.000001
    addy = 0.000001
    if start[1] < target[1]:
        addy = -0.000001
    target = (target[0] + addx, target[1] + addy)

print(start)
print(target)

points = amantides_woo(box_size, start, target)
# 
# ax.plot([p[0] for p in points], [p[1] for p in points], 'ro-')
ax.plot(start[0], start[1], 'bo')
ax.plot([start[0], target[0]], [start[1], target[1]], 'k--')

#plot lines every box_size
for i in range(-50, 50):
    ax.axvline(x=i*box_size, color='gray', linestyle='--')
    ax.axhline(y=i*box_size, color='gray', linestyle='--')

plt.show()