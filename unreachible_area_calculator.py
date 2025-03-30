import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import csv


from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely import normalize
from shapely.ops import unary_union
from shapely import contains

map = "museum"

#house
if map == "house":
    resolution = 0.203125
    actual_arena_width = 9.5
    actual_arena_height = 12
    map_width = 9.18
    map_height = 11.65
    inaccurate_cells = []
    outside_area = [(4.46875, -2.03125),(4.46875, -1.828125),(4.46875, -1.625),(4.46875, -1.421875),(4.46875, -1.21875),(4.46875, -1.015625),(4.46875, -0.8125),(4.46875, -0.609375),(4.46875, -0.40625),(4.46875, -0.203125),(4.46875, 0.0),(4.46875, 0.203125),(4.46875, 0.40625),(4.46875, 0.609375),(4.46875, 0.8125),(4.46875, 1.015625),(4.46875, 1.21875),(4.46875, 1.421875),(4.46875, 1.625),(4.46875, 1.828125),(4.46875, 2.03125),(4.46875, 2.234375),(4.46875, 2.4375),(4.46875, 2.640625),(4.46875, 2.84375),(4.46875, 3.046875),(4.46875, 3.25),(4.46875, 3.453125),(4.46875, 3.65625),(4.46875, 3.859375),(4.46875, 4.0625),(4.46875, 4.265625),(4.46875, 4.46875),(4.46875, 4.671875),(4.46875, 4.875),(4.46875, 5.078125),(4.46875, 5.28125),(4.46875, 5.484375),(4.46875, 5.6875),(4.265625, -2.03125),(4.265625, -1.828125),(4.265625, -1.625),(4.265625, -1.421875),(4.265625, -1.21875),(4.265625, -1.015625),(4.265625, -0.8125),(4.265625, -0.609375),(4.265625, -0.40625),(4.265625, -0.203125),(4.265625, 0.0),(4.265625, 0.203125),(4.265625, 0.40625),(4.265625, 0.609375),(4.265625, 0.8125),(4.265625, 1.015625),(4.265625, 1.21875),(4.265625, 1.421875),(4.265625, 1.625),(4.265625, 1.828125),(4.265625, 2.03125),(4.265625, 2.234375),(4.265625, 2.4375),(4.265625, 2.640625),(4.265625, 2.84375),(4.265625, 3.046875),(4.265625, 3.25),(4.265625, 3.453125),(4.265625, 3.65625),(4.265625, 3.859375),(4.265625, 4.0625),(4.265625, 4.265625),(4.265625, 4.46875),(4.265625, 4.671875),(4.265625, 4.875),(4.265625, 5.078125),(4.265625, 5.28125),(4.265625, 5.484375),(4.265625, 5.6875),(4.0625, 0.609375),(4.0625, 0.8125),(4.0625, 1.015625),(4.0625, 1.21875),(4.0625, 1.421875),(4.0625, 1.625),(4.0625, 1.828125),(4.0625, 2.03125),(4.0625, 2.234375),(4.0625, 2.4375),(4.0625, 2.640625),(4.0625, 2.84375),(4.0625, 3.046875),(4.0625, 3.25),(4.0625, 3.453125),(4.0625, 3.65625),(4.0625, 3.859375),(4.0625, 4.0625),(4.0625, 4.265625),(4.0625, 4.46875),(4.0625, 4.671875),(4.0625, 4.875),(4.0625, 5.078125),(4.0625, 5.28125),(4.0625, 5.484375),(4.0625, 5.6875),(3.859375, 0.609375),(3.859375, 0.8125),(3.859375, 1.015625),(3.859375, 1.21875),(3.859375, 1.421875),(3.859375, 1.625),(3.859375, 1.828125),(3.859375, 2.03125),(3.859375, 2.234375),(3.859375, 2.4375),(3.859375, 2.640625),(3.859375, 2.84375),(3.859375, 3.046875),(3.859375, 3.25),(3.859375, 3.453125),(3.859375, 3.65625),(3.859375, 3.859375),(3.859375, 4.0625),(3.859375, 4.265625),(3.859375, 4.46875),(3.859375, 4.671875),(3.859375, 4.875),(3.859375, 5.078125),(3.859375, 5.28125),(3.859375, 5.484375),(3.859375, 5.6875),(3.65625, 0.609375),(3.65625, 0.8125),(3.65625, 1.015625),(3.65625, 1.21875),(3.65625, 1.421875),(3.65625, 1.625),(3.65625, 1.828125),(3.65625, 2.03125),(3.65625, 2.234375),(3.65625, 2.4375),(3.65625, 2.640625),(3.65625, 2.84375),(3.65625, 3.046875),(3.65625, 3.25),(3.65625, 3.453125),(3.65625, 3.65625),(3.65625, 3.859375),(3.65625, 4.0625),(3.65625, 4.265625),(3.65625, 4.46875),(3.65625, 4.671875),(3.65625, 4.875),(3.65625, 5.078125),(3.65625, 5.28125),(3.65625, 5.484375),(3.65625, 5.6875),(3.453125, 0.609375),(3.453125, 0.8125),(3.453125, 1.015625),(3.453125, 1.21875),(3.453125, 1.421875),(3.453125, 1.625),(3.453125, 1.828125),(3.453125, 2.03125),(3.453125, 2.234375),(3.453125, 2.4375),(3.453125, 2.640625),(3.453125, 2.84375),(3.453125, 3.046875),(3.453125, 3.25),(3.453125, 3.453125),(3.453125, 3.65625),(3.453125, 3.859375),(3.453125, 4.0625),(3.453125, 4.265625),(3.453125, 4.46875),(3.453125, 4.671875),(3.453125, 4.875),(3.453125, 5.078125),(3.453125, 5.28125),(3.453125, 5.484375),(3.453125, 5.6875),(3.25, 0.609375),(3.25, 0.8125),(3.25, 1.015625),(3.25, 1.21875),(3.25, 1.421875),(3.25, 1.625),(3.25, 1.828125),(3.25, 2.03125),(3.25, 2.234375),(3.25, 2.4375),(3.25, 2.640625),(3.25, 2.84375),(3.25, 3.046875),(3.25, 3.25),(3.25, 3.453125),(3.25, 3.65625),(3.25, 3.859375),(3.25, 4.0625),(3.25, 4.265625),(3.25, 4.46875),(3.25, 4.671875),(3.25, 4.875),(3.25, 5.078125),(3.25, 5.28125),(3.25, 5.484375),(3.25, 5.6875),(3.046875, 0.609375),(3.046875, 0.8125),(3.046875, 1.015625),(3.046875, 1.21875),(3.046875, 1.421875),(3.046875, 1.625),(3.046875, 1.828125),(3.046875, 2.03125),(3.046875, 2.234375),(3.046875, 2.4375),(3.046875, 2.640625),(3.046875, 2.84375),(3.046875, 3.046875),(3.046875, 3.25),(3.046875, 3.453125),(3.046875, 3.65625),(3.046875, 3.859375),(3.046875, 4.0625),(3.046875, 4.265625),(3.046875, 4.46875),(3.046875, 4.671875),(3.046875, 4.875),(3.046875, 5.078125),(3.046875, 5.28125),(3.046875, 5.484375),(3.046875, 5.6875),(2.84375, 0.609375),(2.84375, 0.8125),(2.84375, 1.015625),(2.84375, 1.21875),(2.84375, 1.421875),(2.84375, 1.625),(2.84375, 1.828125),(2.84375, 2.03125),(2.84375, 2.234375),(2.84375, 2.4375),(2.84375, 2.640625),(2.84375, 2.84375),(2.84375, 3.046875),(2.84375, 3.25),(2.84375, 3.453125),(2.84375, 3.65625),(2.84375, 3.859375),(2.84375, 4.0625),(2.84375, 4.265625),(2.84375, 4.46875),(2.84375, 4.671875),(2.84375, 4.875),(2.84375, 5.078125),(2.84375, 5.28125),(2.84375, 5.484375),(2.84375, 5.6875),(2.640625, 0.609375),(2.640625, 0.8125),(2.640625, 1.015625),(2.640625, 1.21875),(2.640625, 1.421875),(2.640625, 1.625),(2.640625, 1.828125),(2.640625, 2.03125),(2.640625, 2.234375),(2.640625, 2.4375),(2.640625, 2.640625),(2.640625, 2.84375),(2.640625, 3.046875),(2.640625, 3.25),(2.640625, 3.453125),(2.640625, 3.65625),(2.640625, 3.859375),(2.640625, 4.0625),(2.640625, 4.265625),(2.640625, 4.46875),(2.640625, 4.671875),(2.640625, 4.875),(2.640625, 5.078125),(2.640625, 5.28125),(2.640625, 5.484375),(2.640625, 5.6875),(2.4375, 0.609375),(2.4375, 0.8125),(2.4375, 1.015625),(2.4375, 1.21875),(2.4375, 1.421875),(2.4375, 1.625),(2.4375, 1.828125),(2.4375, 2.03125),(2.4375, 2.234375),(2.4375, 2.4375),(2.4375, 2.640625),(2.4375, 2.84375),(2.4375, 3.046875),(2.4375, 3.25),(2.4375, 3.453125),(2.4375, 3.65625),(2.4375, 3.859375),(2.4375, 4.0625),(2.4375, 4.265625),(2.4375, 4.46875),(2.4375, 4.671875),(2.4375, 4.875),(2.4375, 5.078125),(2.4375, 5.28125),(2.4375, 5.484375),(2.4375, 5.6875),(2.234375, 0.609375),(2.234375, 0.8125),(2.234375, 1.015625),(2.234375, 1.21875),(2.234375, 1.421875),(2.234375, 1.625),(2.234375, 1.828125),(2.234375, 2.03125),(2.234375, 2.234375),(2.234375, 2.4375),(2.234375, 2.640625),(2.234375, 2.84375),(2.234375, 3.046875),(2.234375, 3.25),(2.234375, 3.453125),(2.234375, 3.65625),(2.234375, 3.859375),(2.234375, 4.0625),(2.234375, 4.265625),(2.234375, 4.46875),(2.234375, 4.671875),(2.234375, 4.875),(2.234375, 5.078125),(2.234375, 5.28125),(2.234375, 5.484375),(2.234375, 5.6875),(2.03125, 0.609375),(2.03125, 0.8125),(2.03125, 1.015625),(2.03125, 1.21875),(2.03125, 1.421875),(2.03125, 1.625),(2.03125, 1.828125),(2.03125, 2.03125),(2.03125, 2.234375),(2.03125, 2.4375),(2.03125, 2.640625),(2.03125, 2.84375),(2.03125, 3.046875),(2.03125, 3.25),(2.03125, 3.453125),(2.03125, 3.65625),(2.03125, 3.859375),(2.03125, 4.0625),(2.03125, 4.265625),(2.03125, 4.46875),(2.03125, 4.671875),(2.03125, 4.875),(2.03125, 5.078125),(2.03125, 5.28125),(2.03125, 5.484375),(2.03125, 5.6875),(1.828125, 0.609375),(1.828125, 0.8125),(1.828125, 1.015625),(1.828125, 1.21875),(1.828125, 1.421875),(1.828125, 1.625),(1.828125, 1.828125),(1.828125, 2.03125),(1.828125, 2.234375),(1.828125, 2.4375),(1.828125, 2.640625),(1.828125, 2.84375),(1.828125, 3.046875),(1.828125, 3.25),(1.828125, 3.453125),(1.828125, 3.65625),(1.828125, 3.859375),(1.828125, 4.0625),(1.828125, 4.265625),(1.828125, 4.46875),(1.828125, 4.671875),(1.828125, 4.875),(1.828125, 5.078125),(1.828125, 5.28125),(1.828125, 5.484375),(1.828125, 5.6875),(1.625, 0.609375),(1.625, 0.8125),(1.625, 1.015625),(1.625, 1.21875),(1.625, 1.421875),(1.625, 1.625),(1.625, 1.828125),(1.625, 2.03125),(1.625, 2.234375),(1.625, 2.4375),(1.625, 2.640625),(1.625, 2.84375),(1.625, 3.046875),(1.625, 3.25),(1.625, 3.453125),(1.625, 3.65625),(1.625, 3.859375),(1.625, 4.0625),(1.625, 4.265625),(1.625, 4.46875),(1.625, 4.671875),(1.625, 4.875),(1.625, 5.078125),(1.625, 5.28125),(1.625, 5.484375),(1.625, 5.6875),(1.421875, 0.609375),(1.421875, 0.8125),(1.421875, 1.015625),(1.421875, 1.21875),(1.421875, 1.421875),(1.421875, 1.625),(1.421875, 1.828125),(1.421875, 2.03125),(1.421875, 2.234375),(1.421875, 2.4375),(1.421875, 2.640625),(1.421875, 2.84375),(1.421875, 3.046875),(1.421875, 3.25),(1.421875, 3.453125),(1.421875, 3.65625),(1.421875, 3.859375),(1.421875, 4.0625),(1.421875, 4.265625),(1.421875, 4.46875),(1.421875, 4.671875),(1.421875, 4.875),(1.421875, 5.078125),(1.421875, 5.28125),(1.421875, 5.484375),(1.421875, 5.6875),(1.21875, 0.609375),(1.21875, 0.8125),(1.21875, 1.015625),(1.21875, 1.21875),(1.21875, 1.421875),(1.21875, 1.625),(1.21875, 1.828125),(1.21875, 2.03125),(1.21875, 2.234375),(1.21875, 2.4375),(1.21875, 2.640625),(1.21875, 2.84375),(1.21875, 3.046875),(1.21875, 3.25),(1.21875, 3.453125),(1.21875, 3.65625),(1.21875, 3.859375),(1.21875, 4.0625),(1.21875, 4.265625),(1.21875, 4.46875),(1.21875, 4.671875),(1.21875, 4.875),(1.21875, 5.078125),(1.21875, 5.28125),(1.21875, 5.484375),(1.21875, 5.6875),]
if map == "house_tilted":
    resolution = 0.24375
    actual_arena_width = 13
    actual_arena_height = 14.6
    map_width = 9.18
    map_height = 11.65
    inaccurate_cells = []
    outside_area = [(-0.975, 5.85),(-0.73125, 5.1187499999999995),(-0.73125, 5.3625),(-0.73125, 5.60625),(-0.73125, 5.85),(-0.4875, 4.3875),(-0.4875, 4.63125),(-0.4875, 4.875),(-0.4875, 5.1187499999999995),(-0.4875, 5.3625),(-0.4875, 5.60625),(-0.4875, 5.85),(-0.24375, 3.9),(-0.24375, 4.14375),(-0.24375, 4.3875),(-0.24375, 4.63125),(-0.24375, 4.875),(-0.24375, 5.1187499999999995),(-0.24375, 5.3625),(-0.24375, 5.60625),(-0.24375, 5.85),(-0.24375, 6.09375),(0.0, 3.1687499999999997),(0.0, 3.4125),(0.0, 3.65625),(0.0, 3.9),(0.0, 4.14375),(0.0, 4.3875),(0.0, 4.63125),(0.0, 4.875),(0.0, 5.1187499999999995),(0.0, 5.3625),(0.0, 5.60625),(0.0, 5.85),(0.0, 6.09375),(0.24375, 2.4375),(0.24375, 2.68125),(0.24375, 2.925),(0.24375, 3.1687499999999997),(0.24375, 3.4125),(0.24375, 3.65625),(0.24375, 3.9),(0.24375, 4.14375),(0.24375, 4.3875),(0.24375, 4.63125),(0.24375, 4.875),(0.24375, 5.1187499999999995),(0.24375, 5.3625),(0.24375, 5.60625),(0.24375, 5.85),(0.24375, 6.09375),(0.24375, 6.3374999999999995),(0.4875, 1.70625),(0.4875, 1.95),(0.4875, 2.19375),(0.4875, 2.4375),(0.4875, 2.68125),(0.4875, 2.925),(0.4875, 3.1687499999999997),(0.4875, 3.4125),(0.4875, 3.65625),(0.4875, 3.9),(0.4875, 4.14375),(0.4875, 4.3875),(0.4875, 4.63125),(0.4875, 4.875),(0.4875, 5.1187499999999995),(0.4875, 5.3625),(0.4875, 5.60625),(0.4875, 5.85),(0.4875, 6.09375),(0.4875, 6.3374999999999995),(0.73125, 1.4625),(0.73125, 1.70625),(0.73125, 1.95),(0.73125, 2.19375),(0.73125, 2.4375),(0.73125, 2.68125),(0.73125, 2.925),(0.73125, 3.1687499999999997),(0.73125, 3.4125),(0.73125, 3.65625),(0.73125, 3.9),(0.73125, 4.14375),(0.73125, 4.3875),(0.73125, 4.63125),(0.73125, 4.875),(0.73125, 5.1187499999999995),(0.73125, 5.3625),(0.73125, 5.60625),(0.73125, 5.85),(0.73125, 6.09375),(0.73125, 6.3374999999999995),(0.975, 0.975),(0.975, 1.21875),(0.975, 1.4625),(0.975, 1.70625),(0.975, 1.95),(0.975, 2.19375),(0.975, 2.4375),(0.975, 2.68125),(0.975, 2.925),(0.975, 3.1687499999999997),(0.975, 3.4125),(0.975, 3.65625),(0.975, 3.9),(0.975, 4.14375),(0.975, 4.3875),(0.975, 4.63125),(0.975, 4.875),(0.975, 5.1187499999999995),(0.975, 5.3625),(0.975, 5.60625),(0.975, 5.85),(0.975, 6.09375),(0.975, 6.3374999999999995),(0.975, 6.58125),(1.21875, 1.21875),(1.21875, 1.4625),(1.21875, 1.70625),(1.21875, 1.95),(1.21875, 2.19375),(1.21875, 2.4375),(1.21875, 2.68125),(1.21875, 2.925),(1.21875, 3.1687499999999997),(1.21875, 3.4125),(1.21875, 3.65625),(1.21875, 3.9),(1.21875, 4.14375),(1.21875, 4.3875),(1.21875, 4.63125),(1.21875, 4.875),(1.21875, 5.1187499999999995),(1.21875, 5.3625),(1.21875, 5.60625),(1.21875, 5.85),(1.21875, 6.09375),(1.21875, 6.3374999999999995),(1.21875, 6.58125),(1.4625, 1.21875),(1.4625, 1.4625),(1.4625, 1.70625),(1.4625, 1.95),(1.4625, 2.19375),(1.4625, 2.4375),(1.4625, 2.68125),(1.4625, 2.925),(1.4625, 3.1687499999999997),(1.4625, 3.4125),(1.4625, 3.65625),(1.4625, 3.9),(1.4625, 4.14375),(1.4625, 4.3875),(1.4625, 4.63125),(1.4625, 4.875),(1.4625, 5.1187499999999995),(1.4625, 5.3625),(1.4625, 5.60625),(1.4625, 5.85),(1.4625, 6.09375),(1.4625, 6.3374999999999995),(1.4625, 6.58125),(1.70625, 1.21875),(1.70625, 1.4625),(1.70625, 1.70625),(1.70625, 1.95),(1.70625, 2.19375),(1.70625, 2.4375),(1.70625, 2.68125),(1.70625, 2.925),(1.70625, 3.1687499999999997),(1.70625, 3.4125),(1.70625, 3.65625),(1.70625, 3.9),(1.70625, 4.14375),(1.70625, 4.3875),(1.70625, 4.63125),(1.70625, 4.875),(1.70625, 5.1187499999999995),(1.70625, 5.3625),(1.70625, 5.60625),(1.70625, 5.85),(1.70625, 6.09375),(1.70625, 6.3374999999999995),(1.70625, 6.58125),(1.70625, 6.825),(1.95, 1.4625),(1.95, 1.70625),(1.95, 1.95),(1.95, 2.19375),(1.95, 2.4375),(1.95, 2.68125),(1.95, 2.925),(1.95, 3.1687499999999997),(1.95, 3.4125),(1.95, 3.65625),(1.95, 3.9),(1.95, 4.14375),(1.95, 4.3875),(1.95, 4.63125),(1.95, 4.875),(1.95, 5.1187499999999995),(1.95, 5.3625),(1.95, 5.60625),(1.95, 5.85),(1.95, 6.09375),(1.95, 6.3374999999999995),(1.95, 6.58125),(1.95, 6.825),(2.19375, 1.4625),(2.19375, 1.70625),(2.19375, 1.95),(2.19375, 2.19375),(2.19375, 2.4375),(2.19375, 2.68125),(2.19375, 2.925),(2.19375, 3.1687499999999997),(2.19375, 3.4125),(2.19375, 3.65625),(2.19375, 3.9),(2.19375, 4.14375),(2.19375, 4.3875),(2.19375, 4.63125),(2.19375, 4.875),(2.19375, 5.1187499999999995),(2.19375, 5.3625),(2.19375, 5.60625),(2.19375, 5.85),(2.19375, 6.09375),(2.19375, 6.3374999999999995),(2.19375, 6.58125),(2.19375, 6.825),(2.4375, 1.4625),(2.4375, 1.70625),(2.4375, 1.95),(2.4375, 2.19375),(2.4375, 2.4375),(2.4375, 2.68125),(2.4375, 2.925),(2.4375, 3.1687499999999997),(2.4375, 3.4125),(2.4375, 3.65625),(2.4375, 3.9),(2.4375, 4.14375),(2.4375, 4.3875),(2.4375, 4.63125),(2.4375, 4.875),(2.4375, 5.1187499999999995),(2.4375, 5.3625),(2.4375, 5.60625),(2.4375, 5.85),(2.4375, 6.09375),(2.4375, 6.3374999999999995),(2.4375, 6.58125),(2.4375, 6.825),(2.68125, 1.70625),(2.68125, 1.95),(2.68125, 2.19375),(2.68125, 2.4375),(2.68125, 2.68125),(2.68125, 2.925),(2.68125, 3.1687499999999997),(2.68125, 3.4125),(2.68125, 3.65625),(2.68125, 3.9),(2.68125, 4.14375),(2.68125, 4.3875),(2.68125, 4.63125),(2.68125, 4.875),(2.68125, 5.1187499999999995),(2.68125, 5.3625),(2.68125, 5.60625),(2.68125, 5.85),(2.68125, 6.09375),(2.925, 1.70625),(2.925, 1.95),(2.925, 2.19375),(2.925, 2.4375),(2.925, 2.68125),(2.925, 2.925),(2.925, 3.1687499999999997),(2.925, 3.4125),(2.925, 3.65625),(2.925, 3.9),(2.925, 4.14375),(2.925, 4.3875),(2.925, 4.63125),(2.925, 4.875),(2.925, 5.1187499999999995),(2.925, 5.3625),(3.1687499999999997, 1.95),(3.1687499999999997, 2.19375),(3.1687499999999997, 2.4375),(3.1687499999999997, 2.68125),(3.1687499999999997, 2.925),(3.1687499999999997, 3.1687499999999997),(3.1687499999999997, 3.4125),(3.1687499999999997, 3.65625),(3.1687499999999997, 3.9),(3.1687499999999997, 4.14375),(3.1687499999999997, 4.3875),(3.1687499999999997, 4.63125),(3.1687499999999997, 4.875),(3.4125, 1.95),(3.4125, 2.19375),(3.4125, 2.4375),(3.4125, 2.68125),(3.4125, 2.925),(3.4125, 3.1687499999999997),(3.4125, 3.4125),(3.4125, 3.65625),(3.4125, 3.9),(3.4125, 4.14375),(3.65625, 1.95),(3.65625, 2.19375),(3.65625, 2.4375),(3.65625, 2.68125),(3.65625, 2.925),(3.65625, 3.1687499999999997),(3.65625, 3.4125),(3.9, 1.4625),(3.9, 1.70625),(3.9, 1.95),(3.9, 2.19375),(3.9, 2.4375),(3.9, 2.68125),(4.14375, 0.975),(4.14375, 1.21875),(4.14375, 1.4625),(4.14375, 1.70625),(4.14375, 1.95),(4.14375, 2.19375),(4.3875, 0.24375),(4.3875, 0.4875),(4.3875, 0.73125),(4.3875, 0.975),(4.3875, 1.21875),(4.3875, 1.4625),(4.63125, -0.24375),(4.63125, 0.0),(4.63125, 0.24375),(4.63125, 0.4875),(4.63125, 0.73125),(4.875, -0.24375),(4.875, 0.0),(5.1187499999999995, -0.975),(5.1187499999999995, -0.73125),(5.1187499999999995, -0.4875),(5.3625, -1.4625),(5.3625, -1.21875),(5.3625, -1.7062499999999998),(5.3625, -1.4625),(5.3625, -1.21875),(5.60625, -2.19375),(5.60625, -1.95),(5.85, -2.68125),(6.09375, -3.1687499999999997),(6.3374999999999995, -3.9),(-1.7062499999999998, 5.60625),(-2.4375, 5.3625),(-3.65625, 4.875),(-5.11875, 4.3875),(-5.60625, 4.14375),(-6.3375, 3.9),(-0.4875, 6.09375),(-6.3375, 3.1687499999999997),(-6.3375, 3.4125),(-5.8500000000000005, 4.14375),(-6.09375, 2.68125),(-5.8500000000000005, 2.19375),(-5.8500000000000005, 1.95),(2.68125, -5.3625),(2.925, -5.3625),(0.73125, -6.09375),(0.24375, -6.3375),(0.0, -6.3375),(-0.4875, -6.58125),(2.19375, -5.60625),(-4.3875, -2.19375),(-4.3875, -1.95),(-4.6312500000000005, -1.4625),(-4.6312500000000005, -1.21875),(-5.3625, 0.4875),(4.875, -4.6312500000000005),(5.60625, -4.3875),]
if map == "office":
    resolution = 0.1640625
    actual_arena_width = 20
    actual_arena_height = 10.2
    map_width = 19.705
    map_height = 10.025
    inaccurate_cells = []
    outside_area = []
if map == "office_tilted":
    resolution = 0.18125 
    actual_arena_width = 22.2
    actual_arena_height = 16.4
    map_width = 19.705
    map_height = 10.025
    inaccurate_cells = []
    outside_area = []
if map == "museum":
    resolution = 0.2421875
    actual_arena_width = 30
    actual_arena_height = 30
    map_width = 29.514
    map_height = 29.514
    inaccurate_cells = []
    outside_area = []
if map == "museum_tilted":
    resolution = 0.265625
    actual_arena_width = 33
    actual_arena_height = 33
    map_width = 30
    map_height = 30
    inaccurate_cells = []
    outside_area = []


angle = 20 if "tilted" in map else 0
angle_rad = np.radians(angle) if "tilted" in map else 0
translation_x = map_width /2 * (1-np.cos(angle_rad)) + map_height /2 * np.sin(angle_rad)
translation_y = map_height /2 * (1-np.cos(angle_rad)) - map_width /2 * np.sin(angle_rad)
actual_arena = patches.Rectangle(
    (0 - map_width / 2 + translation_x, 0 - map_height / 2 + translation_y),
    map_width, map_height,
    linewidth=0, facecolor='Pink', alpha=0.5,
    angle=angle
)



def read_arena_boxes(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    arena = root.find('arena')
    boxes = []
    for box in arena.findall('box'):
        id = box.get('id')
        if id.__contains__('spawn_box'):
            continue
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
        if (((cy > y) != (py > y)) and (x < (px - cx) * (y - cy) / (py - cy) + cx)):
            inside = not inside
        px, py = cx, cy
    return inside

def are_all_corners_of_rectangle_fully_covered(cell, boxes, circles):
    """Checks if a rectangle is fully covered by boxes or circles."""
    corners = get_rotated_corners(cell)
    
    for corner in corners:
        covered = False
        for box in boxes:
            if is_point_inside_rectangle(corner, box):
                covered = True
        if not covered:
            for circle in circles:
                if np.linalg.norm(corner - np.array(circle.center)) <= circle.radius:
                    covered = True
        
        if not covered:
            return False  # At least one point is uncovered
    return True  # All points are covered

def select_valid_cells(cells, boxes, circles):
    """Selects all cells that overlap with a box or circle but are not fully covered."""
    valid_cells = []
    
    for cell in cells:
        overlaps = any(check_rectangle_overlap(cell, box) for box in boxes) or \
                   any(is_rectangle_overlapping_circle(cell, circle) for circle in circles)
        
        if overlaps:
            if not are_all_corners_of_rectangle_fully_covered(cell, boxes, circles):
                valid_cells.append(cell)                
        else:
            valid_cells.append(cell)


    
    return valid_cells

def merge_rectangles(l_corners):
    """
    Compute the overlapping area of two rotated rectangles given by their corner coordinates.
    
    rect1_corners and rect2_corners should be lists of 4 (x, y) tuples defining each rectangle.
    """
    l_poly = [Polygon(corners) for corners in l_corners]

    merged_lpoly = unary_union(l_poly)

    return merged_lpoly

def merge_circles(l_circles):
    """
    Compute the overlapping area between a rotated rectangle and a circle using Shapely.
    
    rect_corners: List of 4 (x, y) tuples defining the rectangle.
    circle_center: (cx, cy) tuple representing the center of the circle.
    radius: Circle radius.
    """
    l_circle_polygons = [Point(circle_center).buffer(radius) for circle_center, radius in l_circles]

    merged_circle_polygons = unary_union(l_circle_polygons)
    return merged_circle_polygons

def merge_rectangles_cicles(rectangles, circles):
    l_corners = [get_rotated_corners(rect2) for rect2 in rectangles]
    l_center_radius_circles = [(circle.center, circle.radius) for circle in circles]

    
    merged_rectangles = merge_rectangles(l_corners)
    merged_circles = merge_circles(l_center_radius_circles)

    merged_rectangles_and_circles = unary_union([merged_rectangles, merged_circles])
    return merged_rectangles_and_circles

def calculate_unreachible_areas(arena_boxes, arena_cylinders):
    actual_arena_corners = get_rotated_corners(actual_arena)
    actual_arena_polygon = Polygon(actual_arena_corners)



    fig, ax = plt.subplots()
    ax.add_patch(actual_arena)

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

    x_coords = np.arange(0, actual_arena_width / 2, resolution)
    negative_x_coords = (-np.flip(x_coords)) - resolution  
    x_coords = np.concatenate((negative_x_coords, x_coords))
    y_coords = np.arange(0, actual_arena_height / 2, resolution)
    negative_y_coords = (-np.flip(y_coords)) - resolution
    y_coords = np.concatenate((negative_y_coords, y_coords))

    cells = []
    grid = []

    total_cells = x_coords.size * y_coords.size

    merged_rectangles_circles = merge_rectangles_cicles(arena_rectangles, arena_circles)

    valid_cells = []

    for i,x in enumerate(x_coords):
        for j,y in enumerate(y_coords):
            if not (x > 0 and y > 0):
                continue
            if (x,y) not in outside_area:
                if (x, y) not in inaccurate_cells:
                    cell = patches.Rectangle((x, y), resolution, resolution, facecolor='red', alpha=0.5)
                    cellcorners = get_rotated_corners(cell)
                    cellpoly = Polygon(cellcorners)
                    if not actual_arena_polygon.intersects(cellpoly):
                        continue
                    cells.append(cell)
                    

                gridcell = patches.Rectangle((x, y), resolution, resolution, linewidth=1, facecolor='none', edgecolor='black')
                gridcell
                grid.append(gridcell)
                ax.add_patch(gridcell)
    
    # for cell in cells:
    #     cellcorners = get_rotated_corners(cell)
    #     cellpoly = Polygon(cellcorners)
    #     if (actual_arena_polygon.intersects(cellpoly) or contains(actual_arena_polygon, cellpoly)) and not contains(merged_rectangles_circles, cellpoly):
    #         valid_cells.append(cell)

    


    # valid_cells = select_valid_cells(cells, arena_rectangles, arena_circles)

    non_valid_cells = [cell for cell in cells if cell not in valid_cells]
    

    for cell in non_valid_cells:
        ax.add_patch(cell)

    non_valid_area = non_valid_cells.__len__() * resolution**2
    print(f'Non-valid area: {non_valid_area:f} m^2')


    valid_area = grid.__len__() * resolution**2 - non_valid_area
    print(f'Valid area: {valid_area:f} m^2')

    ax.set_aspect('equal', 'box')
    plt.xlim(-actual_arena_width / 2, actual_arena_width / 2)
    plt.ylim(-actual_arena_height / 2, actual_arena_height / 2)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Unreachable areas')

    # plt.grid(True)
    def on_click(event):
        if event.inaxes is not None:
            x, y = event.xdata, event.ydata
            for cell in grid:
                cell_x, cell_y = cell.get_xy()
                cell_width, cell_height = cell.get_width(), cell.get_height()
                if cell_x <= x <= cell_x + cell_width and cell_y <= y <= cell_y + cell_height:
                    if cell.get_facecolor() != 'blue':
                        print(f'({cell_x}, {cell_y}),', end=None)
                        # highlight cell
                        cell.set_facecolor('blue')

                        # #also include all cells that are in the same column
                        for cell2 in grid:
                            cell2_x, cell2_y = cell2.get_xy()
                            cell2_width, cell2_height = cell2.get_width(), cell2.get_height()
                            # if cell2_x == cell_x and cell2_y > cell_y and cell2.get_facecolor() != 'blue':
                            if cell2_x == cell and cell2_y > cell_y and cell2.get_facecolor() != 'blue':
                                print(f'({cell2_x}, {cell2_y}),', end=None)
                                cell2.set_facecolor('blue')
                    fig.canvas.draw()
                    break

    fig.canvas.mpl_connect('button_press_event', on_click)
    plt.show()



# Usage
arena_boxes = read_arena_boxes('implementation_and_examples/experiments/'+map+'.argos')
arena_cylinders = read_arena_cylinders('implementation_and_examples/experiments/'+map+'.argos')

calculate_unreachible_areas(arena_boxes, arena_cylinders)
# print(f'Precision: {precision:.4f}, Recall: {recall:.4f}')
# plot_mistakes(arena_boxes, quadtree_data)