import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import csv
import re
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.ops import unary_union
from shapely.strtree import STRtree
from shapely.geometry import LineString
from shapely.ops import split
import concurrent.futures
import os



def set_variables_based_on_map(map):
    global resolution, actual_arena_width, actual_arena_height, map_width, map_height, inaccurate_cells, outside_area, actual_arena, actual_arena2, exp_path, env_map
    
    exp_path = 'implementation_and_examples/experiments/'+ map + '.argos'

    if map == "house":
        env_map = "house"
        resolution = 0.203125
        actual_arena_width = 9.5
        actual_arena_height = 12
        map_width = 9.18
        map_height = 11.65
        inaccurate_cells = []
        outside_area = [(4.46875, -2.03125),(4.46875, -1.828125),(4.46875, -1.625),(4.46875, -1.421875),(4.46875, -1.21875),(4.46875, -1.015625),(4.46875, -0.8125),(4.46875, -0.609375),(4.46875, -0.40625),(4.46875, -0.203125),(4.46875, 0.0),(4.46875, 0.203125),(4.46875, 0.40625),(4.46875, 0.609375),(4.46875, 0.8125),(4.46875, 1.015625),(4.46875, 1.21875),(4.46875, 1.421875),(4.46875, 1.625),(4.46875, 1.828125),(4.46875, 2.03125),(4.46875, 2.234375),(4.46875, 2.4375),(4.46875, 2.640625),(4.46875, 2.84375),(4.46875, 3.046875),(4.46875, 3.25),(4.46875, 3.453125),(4.46875, 3.65625),(4.46875, 3.859375),(4.46875, 4.0625),(4.46875, 4.265625),(4.46875, 4.46875),(4.46875, 4.671875),(4.46875, 4.875),(4.46875, 5.078125),(4.46875, 5.28125),(4.46875, 5.484375),(4.46875, 5.6875),(4.265625, -2.03125),(4.265625, -1.828125),(4.265625, -1.625),(4.265625, -1.421875),(4.265625, -1.21875),(4.265625, -1.015625),(4.265625, -0.8125),(4.265625, -0.609375),(4.265625, -0.40625),(4.265625, -0.203125),(4.265625, 0.0),(4.265625, 0.203125),(4.265625, 0.40625),(4.265625, 0.609375),(4.265625, 0.8125),(4.265625, 1.015625),(4.265625, 1.21875),(4.265625, 1.421875),(4.265625, 1.625),(4.265625, 1.828125),(4.265625, 2.03125),(4.265625, 2.234375),(4.265625, 2.4375),(4.265625, 2.640625),(4.265625, 2.84375),(4.265625, 3.046875),(4.265625, 3.25),(4.265625, 3.453125),(4.265625, 3.65625),(4.265625, 3.859375),(4.265625, 4.0625),(4.265625, 4.265625),(4.265625, 4.46875),(4.265625, 4.671875),(4.265625, 4.875),(4.265625, 5.078125),(4.265625, 5.28125),(4.265625, 5.484375),(4.265625, 5.6875),(4.0625, 0.609375),(4.0625, 0.8125),(4.0625, 1.015625),(4.0625, 1.21875),(4.0625, 1.421875),(4.0625, 1.625),(4.0625, 1.828125),(4.0625, 2.03125),(4.0625, 2.234375),(4.0625, 2.4375),(4.0625, 2.640625),(4.0625, 2.84375),(4.0625, 3.046875),(4.0625, 3.25),(4.0625, 3.453125),(4.0625, 3.65625),(4.0625, 3.859375),(4.0625, 4.0625),(4.0625, 4.265625),(4.0625, 4.46875),(4.0625, 4.671875),(4.0625, 4.875),(4.0625, 5.078125),(4.0625, 5.28125),(4.0625, 5.484375),(4.0625, 5.6875),(3.859375, 0.609375),(3.859375, 0.8125),(3.859375, 1.015625),(3.859375, 1.21875),(3.859375, 1.421875),(3.859375, 1.625),(3.859375, 1.828125),(3.859375, 2.03125),(3.859375, 2.234375),(3.859375, 2.4375),(3.859375, 2.640625),(3.859375, 2.84375),(3.859375, 3.046875),(3.859375, 3.25),(3.859375, 3.453125),(3.859375, 3.65625),(3.859375, 3.859375),(3.859375, 4.0625),(3.859375, 4.265625),(3.859375, 4.46875),(3.859375, 4.671875),(3.859375, 4.875),(3.859375, 5.078125),(3.859375, 5.28125),(3.859375, 5.484375),(3.859375, 5.6875),(3.65625, 0.609375),(3.65625, 0.8125),(3.65625, 1.015625),(3.65625, 1.21875),(3.65625, 1.421875),(3.65625, 1.625),(3.65625, 1.828125),(3.65625, 2.03125),(3.65625, 2.234375),(3.65625, 2.4375),(3.65625, 2.640625),(3.65625, 2.84375),(3.65625, 3.046875),(3.65625, 3.25),(3.65625, 3.453125),(3.65625, 3.65625),(3.65625, 3.859375),(3.65625, 4.0625),(3.65625, 4.265625),(3.65625, 4.46875),(3.65625, 4.671875),(3.65625, 4.875),(3.65625, 5.078125),(3.65625, 5.28125),(3.65625, 5.484375),(3.65625, 5.6875),(3.453125, 0.609375),(3.453125, 0.8125),(3.453125, 1.015625),(3.453125, 1.21875),(3.453125, 1.421875),(3.453125, 1.625),(3.453125, 1.828125),(3.453125, 2.03125),(3.453125, 2.234375),(3.453125, 2.4375),(3.453125, 2.640625),(3.453125, 2.84375),(3.453125, 3.046875),(3.453125, 3.25),(3.453125, 3.453125),(3.453125, 3.65625),(3.453125, 3.859375),(3.453125, 4.0625),(3.453125, 4.265625),(3.453125, 4.46875),(3.453125, 4.671875),(3.453125, 4.875),(3.453125, 5.078125),(3.453125, 5.28125),(3.453125, 5.484375),(3.453125, 5.6875),(3.25, 0.609375),(3.25, 0.8125),(3.25, 1.015625),(3.25, 1.21875),(3.25, 1.421875),(3.25, 1.625),(3.25, 1.828125),(3.25, 2.03125),(3.25, 2.234375),(3.25, 2.4375),(3.25, 2.640625),(3.25, 2.84375),(3.25, 3.046875),(3.25, 3.25),(3.25, 3.453125),(3.25, 3.65625),(3.25, 3.859375),(3.25, 4.0625),(3.25, 4.265625),(3.25, 4.46875),(3.25, 4.671875),(3.25, 4.875),(3.25, 5.078125),(3.25, 5.28125),(3.25, 5.484375),(3.25, 5.6875),(3.046875, 0.609375),(3.046875, 0.8125),(3.046875, 1.015625),(3.046875, 1.21875),(3.046875, 1.421875),(3.046875, 1.625),(3.046875, 1.828125),(3.046875, 2.03125),(3.046875, 2.234375),(3.046875, 2.4375),(3.046875, 2.640625),(3.046875, 2.84375),(3.046875, 3.046875),(3.046875, 3.25),(3.046875, 3.453125),(3.046875, 3.65625),(3.046875, 3.859375),(3.046875, 4.0625),(3.046875, 4.265625),(3.046875, 4.46875),(3.046875, 4.671875),(3.046875, 4.875),(3.046875, 5.078125),(3.046875, 5.28125),(3.046875, 5.484375),(3.046875, 5.6875),(2.84375, 0.609375),(2.84375, 0.8125),(2.84375, 1.015625),(2.84375, 1.21875),(2.84375, 1.421875),(2.84375, 1.625),(2.84375, 1.828125),(2.84375, 2.03125),(2.84375, 2.234375),(2.84375, 2.4375),(2.84375, 2.640625),(2.84375, 2.84375),(2.84375, 3.046875),(2.84375, 3.25),(2.84375, 3.453125),(2.84375, 3.65625),(2.84375, 3.859375),(2.84375, 4.0625),(2.84375, 4.265625),(2.84375, 4.46875),(2.84375, 4.671875),(2.84375, 4.875),(2.84375, 5.078125),(2.84375, 5.28125),(2.84375, 5.484375),(2.84375, 5.6875),(2.640625, 0.609375),(2.640625, 0.8125),(2.640625, 1.015625),(2.640625, 1.21875),(2.640625, 1.421875),(2.640625, 1.625),(2.640625, 1.828125),(2.640625, 2.03125),(2.640625, 2.234375),(2.640625, 2.4375),(2.640625, 2.640625),(2.640625, 2.84375),(2.640625, 3.046875),(2.640625, 3.25),(2.640625, 3.453125),(2.640625, 3.65625),(2.640625, 3.859375),(2.640625, 4.0625),(2.640625, 4.265625),(2.640625, 4.46875),(2.640625, 4.671875),(2.640625, 4.875),(2.640625, 5.078125),(2.640625, 5.28125),(2.640625, 5.484375),(2.640625, 5.6875),(2.4375, 0.609375),(2.4375, 0.8125),(2.4375, 1.015625),(2.4375, 1.21875),(2.4375, 1.421875),(2.4375, 1.625),(2.4375, 1.828125),(2.4375, 2.03125),(2.4375, 2.234375),(2.4375, 2.4375),(2.4375, 2.640625),(2.4375, 2.84375),(2.4375, 3.046875),(2.4375, 3.25),(2.4375, 3.453125),(2.4375, 3.65625),(2.4375, 3.859375),(2.4375, 4.0625),(2.4375, 4.265625),(2.4375, 4.46875),(2.4375, 4.671875),(2.4375, 4.875),(2.4375, 5.078125),(2.4375, 5.28125),(2.4375, 5.484375),(2.4375, 5.6875),(2.234375, 0.609375),(2.234375, 0.8125),(2.234375, 1.015625),(2.234375, 1.21875),(2.234375, 1.421875),(2.234375, 1.625),(2.234375, 1.828125),(2.234375, 2.03125),(2.234375, 2.234375),(2.234375, 2.4375),(2.234375, 2.640625),(2.234375, 2.84375),(2.234375, 3.046875),(2.234375, 3.25),(2.234375, 3.453125),(2.234375, 3.65625),(2.234375, 3.859375),(2.234375, 4.0625),(2.234375, 4.265625),(2.234375, 4.46875),(2.234375, 4.671875),(2.234375, 4.875),(2.234375, 5.078125),(2.234375, 5.28125),(2.234375, 5.484375),(2.234375, 5.6875),(2.03125, 0.609375),(2.03125, 0.8125),(2.03125, 1.015625),(2.03125, 1.21875),(2.03125, 1.421875),(2.03125, 1.625),(2.03125, 1.828125),(2.03125, 2.03125),(2.03125, 2.234375),(2.03125, 2.4375),(2.03125, 2.640625),(2.03125, 2.84375),(2.03125, 3.046875),(2.03125, 3.25),(2.03125, 3.453125),(2.03125, 3.65625),(2.03125, 3.859375),(2.03125, 4.0625),(2.03125, 4.265625),(2.03125, 4.46875),(2.03125, 4.671875),(2.03125, 4.875),(2.03125, 5.078125),(2.03125, 5.28125),(2.03125, 5.484375),(2.03125, 5.6875),(1.828125, 0.609375),(1.828125, 0.8125),(1.828125, 1.015625),(1.828125, 1.21875),(1.828125, 1.421875),(1.828125, 1.625),(1.828125, 1.828125),(1.828125, 2.03125),(1.828125, 2.234375),(1.828125, 2.4375),(1.828125, 2.640625),(1.828125, 2.84375),(1.828125, 3.046875),(1.828125, 3.25),(1.828125, 3.453125),(1.828125, 3.65625),(1.828125, 3.859375),(1.828125, 4.0625),(1.828125, 4.265625),(1.828125, 4.46875),(1.828125, 4.671875),(1.828125, 4.875),(1.828125, 5.078125),(1.828125, 5.28125),(1.828125, 5.484375),(1.828125, 5.6875),(1.625, 0.609375),(1.625, 0.8125),(1.625, 1.015625),(1.625, 1.21875),(1.625, 1.421875),(1.625, 1.625),(1.625, 1.828125),(1.625, 2.03125),(1.625, 2.234375),(1.625, 2.4375),(1.625, 2.640625),(1.625, 2.84375),(1.625, 3.046875),(1.625, 3.25),(1.625, 3.453125),(1.625, 3.65625),(1.625, 3.859375),(1.625, 4.0625),(1.625, 4.265625),(1.625, 4.46875),(1.625, 4.671875),(1.625, 4.875),(1.625, 5.078125),(1.625, 5.28125),(1.625, 5.484375),(1.625, 5.6875),(1.421875, 0.609375),(1.421875, 0.8125),(1.421875, 1.015625),(1.421875, 1.21875),(1.421875, 1.421875),(1.421875, 1.625),(1.421875, 1.828125),(1.421875, 2.03125),(1.421875, 2.234375),(1.421875, 2.4375),(1.421875, 2.640625),(1.421875, 2.84375),(1.421875, 3.046875),(1.421875, 3.25),(1.421875, 3.453125),(1.421875, 3.65625),(1.421875, 3.859375),(1.421875, 4.0625),(1.421875, 4.265625),(1.421875, 4.46875),(1.421875, 4.671875),(1.421875, 4.875),(1.421875, 5.078125),(1.421875, 5.28125),(1.421875, 5.484375),(1.421875, 5.6875),(1.21875, 0.609375),(1.21875, 0.8125),(1.21875, 1.015625),(1.21875, 1.21875),(1.21875, 1.421875),(1.21875, 1.625),(1.21875, 1.828125),(1.21875, 2.03125),(1.21875, 2.234375),(1.21875, 2.4375),(1.21875, 2.640625),(1.21875, 2.84375),(1.21875, 3.046875),(1.21875, 3.25),(1.21875, 3.453125),(1.21875, 3.65625),(1.21875, 3.859375),(1.21875, 4.0625),(1.21875, 4.265625),(1.21875, 4.46875),(1.21875, 4.671875),(1.21875, 4.875),(1.21875, 5.078125),(1.21875, 5.28125),(1.21875, 5.484375),(1.21875, 5.6875),]
    if map == "house_tilted":
        env_map = "house_tilted"
        resolution = 0.24375
        actual_arena_width = 13
        actual_arena_height = 14.6
        map_width = 9.18
        map_height = 11.65
        inaccurate_cells = []
        outside_area = [(-0.975, 5.85),(-0.73125, 5.1187499999999995),(-0.73125, 5.3625),(-0.73125, 5.60625),(-0.73125, 5.85),(-0.4875, 4.3875),(-0.4875, 4.63125),(-0.4875, 4.875),(-0.4875, 5.1187499999999995),(-0.4875, 5.3625),(-0.4875, 5.60625),(-0.4875, 5.85),(-0.24375, 3.9),(-0.24375, 4.14375),(-0.24375, 4.3875),(-0.24375, 4.63125),(-0.24375, 4.875),(-0.24375, 5.1187499999999995),(-0.24375, 5.3625),(-0.24375, 5.60625),(-0.24375, 5.85),(-0.24375, 6.09375),(0.0, 3.1687499999999997),(0.0, 3.4125),(0.0, 3.65625),(0.0, 3.9),(0.0, 4.14375),(0.0, 4.3875),(0.0, 4.63125),(0.0, 4.875),(0.0, 5.1187499999999995),(0.0, 5.3625),(0.0, 5.60625),(0.0, 5.85),(0.0, 6.09375),(0.24375, 2.4375),(0.24375, 2.68125),(0.24375, 2.925),(0.24375, 3.1687499999999997),(0.24375, 3.4125),(0.24375, 3.65625),(0.24375, 3.9),(0.24375, 4.14375),(0.24375, 4.3875),(0.24375, 4.63125),(0.24375, 4.875),(0.24375, 5.1187499999999995),(0.24375, 5.3625),(0.24375, 5.60625),(0.24375, 5.85),(0.24375, 6.09375),(0.24375, 6.3374999999999995),(0.4875, 1.70625),(0.4875, 1.95),(0.4875, 2.19375),(0.4875, 2.4375),(0.4875, 2.68125),(0.4875, 2.925),(0.4875, 3.1687499999999997),(0.4875, 3.4125),(0.4875, 3.65625),(0.4875, 3.9),(0.4875, 4.14375),(0.4875, 4.3875),(0.4875, 4.63125),(0.4875, 4.875),(0.4875, 5.1187499999999995),(0.4875, 5.3625),(0.4875, 5.60625),(0.4875, 5.85),(0.4875, 6.09375),(0.4875, 6.3374999999999995),(0.73125, 1.4625),(0.73125, 1.70625),(0.73125, 1.95),(0.73125, 2.19375),(0.73125, 2.4375),(0.73125, 2.68125),(0.73125, 2.925),(0.73125, 3.1687499999999997),(0.73125, 3.4125),(0.73125, 3.65625),(0.73125, 3.9),(0.73125, 4.14375),(0.73125, 4.3875),(0.73125, 4.63125),(0.73125, 4.875),(0.73125, 5.1187499999999995),(0.73125, 5.3625),(0.73125, 5.60625),(0.73125, 5.85),(0.73125, 6.09375),(0.73125, 6.3374999999999995),(0.975, 0.975),(0.975, 1.21875),(0.975, 1.4625),(0.975, 1.70625),(0.975, 1.95),(0.975, 2.19375),(0.975, 2.4375),(0.975, 2.68125),(0.975, 2.925),(0.975, 3.1687499999999997),(0.975, 3.4125),(0.975, 3.65625),(0.975, 3.9),(0.975, 4.14375),(0.975, 4.3875),(0.975, 4.63125),(0.975, 4.875),(0.975, 5.1187499999999995),(0.975, 5.3625),(0.975, 5.60625),(0.975, 5.85),(0.975, 6.09375),(0.975, 6.3374999999999995),(0.975, 6.58125),(1.21875, 1.21875),(1.21875, 1.4625),(1.21875, 1.70625),(1.21875, 1.95),(1.21875, 2.19375),(1.21875, 2.4375),(1.21875, 2.68125),(1.21875, 2.925),(1.21875, 3.1687499999999997),(1.21875, 3.4125),(1.21875, 3.65625),(1.21875, 3.9),(1.21875, 4.14375),(1.21875, 4.3875),(1.21875, 4.63125),(1.21875, 4.875),(1.21875, 5.1187499999999995),(1.21875, 5.3625),(1.21875, 5.60625),(1.21875, 5.85),(1.21875, 6.09375),(1.21875, 6.3374999999999995),(1.21875, 6.58125),(1.4625, 1.21875),(1.4625, 1.4625),(1.4625, 1.70625),(1.4625, 1.95),(1.4625, 2.19375),(1.4625, 2.4375),(1.4625, 2.68125),(1.4625, 2.925),(1.4625, 3.1687499999999997),(1.4625, 3.4125),(1.4625, 3.65625),(1.4625, 3.9),(1.4625, 4.14375),(1.4625, 4.3875),(1.4625, 4.63125),(1.4625, 4.875),(1.4625, 5.1187499999999995),(1.4625, 5.3625),(1.4625, 5.60625),(1.4625, 5.85),(1.4625, 6.09375),(1.4625, 6.3374999999999995),(1.4625, 6.58125),(1.70625, 1.21875),(1.70625, 1.4625),(1.70625, 1.70625),(1.70625, 1.95),(1.70625, 2.19375),(1.70625, 2.4375),(1.70625, 2.68125),(1.70625, 2.925),(1.70625, 3.1687499999999997),(1.70625, 3.4125),(1.70625, 3.65625),(1.70625, 3.9),(1.70625, 4.14375),(1.70625, 4.3875),(1.70625, 4.63125),(1.70625, 4.875),(1.70625, 5.1187499999999995),(1.70625, 5.3625),(1.70625, 5.60625),(1.70625, 5.85),(1.70625, 6.09375),(1.70625, 6.3374999999999995),(1.70625, 6.58125),(1.70625, 6.825),(1.95, 1.4625),(1.95, 1.70625),(1.95, 1.95),(1.95, 2.19375),(1.95, 2.4375),(1.95, 2.68125),(1.95, 2.925),(1.95, 3.1687499999999997),(1.95, 3.4125),(1.95, 3.65625),(1.95, 3.9),(1.95, 4.14375),(1.95, 4.3875),(1.95, 4.63125),(1.95, 4.875),(1.95, 5.1187499999999995),(1.95, 5.3625),(1.95, 5.60625),(1.95, 5.85),(1.95, 6.09375),(1.95, 6.3374999999999995),(1.95, 6.58125),(1.95, 6.825),(2.19375, 1.4625),(2.19375, 1.70625),(2.19375, 1.95),(2.19375, 2.19375),(2.19375, 2.4375),(2.19375, 2.68125),(2.19375, 2.925),(2.19375, 3.1687499999999997),(2.19375, 3.4125),(2.19375, 3.65625),(2.19375, 3.9),(2.19375, 4.14375),(2.19375, 4.3875),(2.19375, 4.63125),(2.19375, 4.875),(2.19375, 5.1187499999999995),(2.19375, 5.3625),(2.19375, 5.60625),(2.19375, 5.85),(2.19375, 6.09375),(2.19375, 6.3374999999999995),(2.19375, 6.58125),(2.19375, 6.825),(2.4375, 1.4625),(2.4375, 1.70625),(2.4375, 1.95),(2.4375, 2.19375),(2.4375, 2.4375),(2.4375, 2.68125),(2.4375, 2.925),(2.4375, 3.1687499999999997),(2.4375, 3.4125),(2.4375, 3.65625),(2.4375, 3.9),(2.4375, 4.14375),(2.4375, 4.3875),(2.4375, 4.63125),(2.4375, 4.875),(2.4375, 5.1187499999999995),(2.4375, 5.3625),(2.4375, 5.60625),(2.4375, 5.85),(2.4375, 6.09375),(2.4375, 6.3374999999999995),(2.4375, 6.58125),(2.4375, 6.825),(2.68125, 1.70625),(2.68125, 1.95),(2.68125, 2.19375),(2.68125, 2.4375),(2.68125, 2.68125),(2.68125, 2.925),(2.68125, 3.1687499999999997),(2.68125, 3.4125),(2.68125, 3.65625),(2.68125, 3.9),(2.68125, 4.14375),(2.68125, 4.3875),(2.68125, 4.63125),(2.68125, 4.875),(2.68125, 5.1187499999999995),(2.68125, 5.3625),(2.68125, 5.60625),(2.68125, 5.85),(2.68125, 6.09375),(2.925, 1.70625),(2.925, 1.95),(2.925, 2.19375),(2.925, 2.4375),(2.925, 2.68125),(2.925, 2.925),(2.925, 3.1687499999999997),(2.925, 3.4125),(2.925, 3.65625),(2.925, 3.9),(2.925, 4.14375),(2.925, 4.3875),(2.925, 4.63125),(2.925, 4.875),(2.925, 5.1187499999999995),(2.925, 5.3625),(3.1687499999999997, 1.95),(3.1687499999999997, 2.19375),(3.1687499999999997, 2.4375),(3.1687499999999997, 2.68125),(3.1687499999999997, 2.925),(3.1687499999999997, 3.1687499999999997),(3.1687499999999997, 3.4125),(3.1687499999999997, 3.65625),(3.1687499999999997, 3.9),(3.1687499999999997, 4.14375),(3.1687499999999997, 4.3875),(3.1687499999999997, 4.63125),(3.1687499999999997, 4.875),(3.4125, 1.95),(3.4125, 2.19375),(3.4125, 2.4375),(3.4125, 2.68125),(3.4125, 2.925),(3.4125, 3.1687499999999997),(3.4125, 3.4125),(3.4125, 3.65625),(3.4125, 3.9),(3.4125, 4.14375),(3.65625, 1.95),(3.65625, 2.19375),(3.65625, 2.4375),(3.65625, 2.68125),(3.65625, 2.925),(3.65625, 3.1687499999999997),(3.65625, 3.4125),(3.9, 1.4625),(3.9, 1.70625),(3.9, 1.95),(3.9, 2.19375),(3.9, 2.4375),(3.9, 2.68125),(4.14375, 0.975),(4.14375, 1.21875),(4.14375, 1.4625),(4.14375, 1.70625),(4.14375, 1.95),(4.14375, 2.19375),(4.3875, 0.24375),(4.3875, 0.4875),(4.3875, 0.73125),(4.3875, 0.975),(4.3875, 1.21875),(4.3875, 1.4625),(4.63125, -0.24375),(4.63125, 0.0),(4.63125, 0.24375),(4.63125, 0.4875),(4.63125, 0.73125),(4.875, -0.24375),(4.875, 0.0),(5.1187499999999995, -0.975),(5.1187499999999995, -0.73125),(5.1187499999999995, -0.4875),(5.3625, -1.4625),(5.3625, -1.21875),(5.3625, -1.7062499999999998),(5.3625, -1.4625),(5.3625, -1.21875),(5.60625, -2.19375),(5.60625, -1.95),(5.85, -2.68125),(6.09375, -3.1687499999999997),(6.3374999999999995, -3.9),(-1.7062499999999998, 5.60625),(-2.4375, 5.3625),(-3.65625, 4.875),(-5.11875, 4.3875),(-5.60625, 4.14375),(-6.3375, 3.9),(-0.4875, 6.09375),(-6.3375, 3.1687499999999997),(-6.3375, 3.4125),(-5.8500000000000005, 4.14375),(-6.09375, 2.68125),(-5.8500000000000005, 2.19375),(-5.8500000000000005, 1.95),(2.68125, -5.3625),(2.925, -5.3625),(0.73125, -6.09375),(0.24375, -6.3375),(0.0, -6.3375),(-0.4875, -6.58125),(2.19375, -5.60625),(-4.3875, -2.19375),(-4.3875, -1.95),(-4.6312500000000005, -1.4625),(-4.6312500000000005, -1.21875),(-5.3625, 0.4875),(4.875, -4.6312500000000005),(5.60625, -4.3875),]
    if map == "office":
        env_map = "office"
        resolution = 0.1640625
        actual_arena_width = 20
        actual_arena_height = 10.2
        map_width = 19.705
        map_height = 10.025
        inaccurate_cells = []
        outside_area = []
    if map == "office_tilted":
        env_map = "office_tilted"
        resolution = 0.18125
        actual_arena_width = 22.2
        actual_arena_height = 16.4
        map_width = 19.705
        map_height = 10.025
        inaccurate_cells = []
        outside_area = []
    if map == "museum":
       env_map = "museum"
       resolution = 0.2421875
       actual_arena_width = 30
       actual_arena_height = 30
       map_width = 29.514
       map_height = 29.514
       map_width2 = 29.75
       map_height2 = 29.75
    if map == "museum_tilted":
        env_map = "museum_tilted"
        resolution = 0.265625
        actual_arena_width = 33
        actual_arena_height = 33
        map_width = 29.21
        map_height = 29.21
        map_width2 = 29.75
        map_height2 = 29.75
        inaccurate_cells = []
        outside_area = []
    
    # actual_arena = patches.Rectangle((-actual_arena_width / 2, -actual_arena_height / 2), actual_arena_width, actual_arena_height, linewidth=0, edgecolor='r', facecolor='blue', alpha=0.5

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

    if 'museum' in map:
        angle2 = 45+20 if "tilted" in map else 45
        angle_rad2 = np.radians(angle2)
        translation_x2 = map_width2 /2 * (1-np.cos(angle_rad2)) + map_height2 /2 * np.sin(angle_rad2)
        translation_y2 = map_height2 /2 * (1-np.cos(angle_rad2)) - map_width2 /2 * np.sin(angle_rad2)
        actual_arena2 = patches.Rectangle(
            (0 - map_width2 / 2 + translation_x2, 0 - map_height2 / 2 + translation_y2),
            map_width2, map_height2,
            linewidth=0, facecolor='Purple', alpha=0.5,
            angle=angle2
        )

def get_total_area_from_map_spawn_time(env_map, spawn_time):
    spawn_time_int = int(spawn_time.split('_')[2])  
    if env_map == 'house':
        # if spawn_time_int == 0:
        total_area=75.299072
    elif env_map == 'house_tilted':
        # if spawn_time_int == 0:
        total_area=78.604805
    elif env_map == 'office':
        # if spawn_time_int == 0:
        total_area=173.153870
    elif env_map == 'office_tilted':
        # if spawn_time_int == 0:
        total_area=176.347188
    elif env_map == 'museum':
        # if spawn_time_int == 0:
        total_area=689.311035
    elif env_map == 'museum_tilted':
        # if spawn_time_int == 0:
        total_area=701.826904
    return total_area

def find_smallest_box_size(quadtree_data):
        smallest_box_size = 999999
        for row in quadtree_data:
            box_size = float(row['box_size'])
            if box_size < smallest_box_size:
                smallest_box_size = box_size
        return smallest_box_size

def read_arena_spawn_boxes(spawn_data, mission_time):
    spawn_boxes_incl = []
    if spawn_data:
        for box_id, spawn_time in spawn_data:
            if spawn_time <= mission_time:
                spawn_boxes_incl.append(box_id)
    else:
        return []
    print("Parsing spawn boxes")

    tree = ET.parse(exp_path)
    root = tree.getroot()
    arena = root.find('arena')
    spawn_boxes = []
    for box in arena.findall('box'):
        id = box.get('id')
        if id not in spawn_boxes_incl:
            continue

        # Else parse the box

        size = box.get('size').split(',')
        position = box.find('body').get('position').split(',')
        orientation = box.find('body').get('orientation').split(',')

        height = float(size[0])
        width = float(size[1])
        y = float(position[0])
        x = -float(position[1])
        angle = float(orientation[0])  # Assuming rotation around the Z-axis

        spawn_boxes.append({
            'width': width,
            'height': height,
            'x': x,
            'y': y,
            'angle': angle
        })
    return spawn_boxes


def read_arena_boxes():
    ignore_spawn_boxes = True
    tree = ET.parse(exp_path)
    root = tree.getroot()
    arena = root.find('arena')
    boxes = []
    for box in arena.findall('box'):
        id = box.get('id')
        if ignore_spawn_boxes and id.startswith('spawn'):   
            continue

        # Else parse the box

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

def read_arena_cylinders(): 
    tree = ET.parse(exp_path)
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

def read_quadtree_data(filename):
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

def merge_rectangles(l_corners):
    """
    Compute the overlapping area of two rotated rectangles given by their corner coordinates.
    
    rect1_corners and rect2_corners should be lists of 4 (x, y) tuples defining each rectangle.
    """
    l_poly = [Polygon(corners) for corners in l_corners]

    merged_lpoly = unary_union(l_poly)

    return merged_lpoly

# Example usage
# rect1 = [(1, 1), (4, 1), (4, 3), (1, 3)]  # Rectangle 1 corners
# rect2 = [(2, 0), (5, 1), (4, 4), (1, 3)]  # Rotated rectangle 2 corners

# overlap_area = rotated_rectangle_overlap_area(rect1, rect2)
# print("Overlap Area:", overlap_area)


def check_rectangle_map_overlap(rect1, str_tree, grid):
    """Checks if two rotated rectangles overlap using the Separating Axis Theorem (SAT)."""
    # Get corners of both rectangles
    corners1 = get_rotated_corners(rect1)
    
    poly1 = Polygon(corners1)
    possible_matches_indices = str_tree.query(poly1)
    intersection_area = 0
    for match_index in possible_matches_indices:
        match = grid[match_index]
        intersection = poly1.intersection(match)
        intersection_area += intersection.area
    
    # #Check intersection with spawn boxes
    # for spawn_box in spawn_boxes_polygons:
    #     intersection = poly1.intersection(spawn_box)
    #     intersection_area += intersection.area
    
    return intersection_area / poly1.area

# def check_rectangle_map_overlap(rect1, merged_rectangles_and_circles):
#     """Checks if two rotated rectangles overlap using the Separating Axis Theorem (SAT)."""
#     # Get corners of both rectangles
#     corners1 = get_rotated_corners(rect1)
    
#     poly1 = Polygon(corners1)
#     intersection = poly1.intersection(merged_rectangles_and_circles)
    
#     return intersection.area / poly1.area


import numpy as np

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

# Example usage
# rect = [(1, 1), (4, 1), (4, 3), (1, 3)]  # Rectangle corners
# circle = (3, 2), 1.5  # Center (cx, cy) and radius

# overlap_area = rectangle_circle_overlap_shapely(rect, circle[0], circle[1])
# print("Overlap Area:", overlap_area)



def check_circle_rectangle_overlap(rectangle, circles):
    """Check if a matplotlib.patches.Circle and matplotlib.patches.Rectangle overlap"""

    # Extract circle properties
    # circle_center = np.array(circle.center)
    # circle_radius = circle.radius

    # # Extract rectangle properties
    # rect_x, rect_y = rectangle.get_xy()  # Bottom-left corner
    # rect_width = rectangle.get_width()
    # rect_height = rectangle.get_height()

    # # Get rectangle bounding box points
    # rect_x1, rect_y1 = rect_x + rect_width, rect_y + rect_height  # Top-right corner

    # overlap_area 
    return merge_circles(get_rotated_corners(rectangle), circles)

    # # Step 1: Bounding Box Check
    # if not (rect_x <= circle_center[0] <= rect_x1 and rect_y <= circle_center[1] <= rect_y1):
    #     # Check if bounding boxes overlap
    #     circle_bbox = patches.Rectangle((circle.center[0] - circle.radius, circle.center[1] - circle.radius), 2 * circle.radius, 2 * circle.radius).get_extents()
    #     rect_bbox = rectangle.get_extents()
    #     if not circle_bbox.overlaps(rect_bbox):
    #         return False  # No overlap

    # # Step 2: Precise Check - Find the closest point on the rectangle to the circle
    # closest_x = np.clip(circle_center[0], rect_x, rect_x1)
    # closest_y = np.clip(circle_center[1], rect_y, rect_y1)

    # # Compute the distance from the closest point to the circle center
    # distance = np.linalg.norm(circle_center - np.array([closest_x, closest_y]))

    # # Check if the closest point is inside the circle
    # return distance <= circle_radius

def coordinate_equals_with_epsilon(a, b, epsilon=1e-4):
    return abs(a[0] - b[0]) < epsilon and abs(a[1] - b[1]) < epsilon

def equals_with_epsilon(a, b, epsilon=1e-4):
    return abs(a - b) < epsilon

def create_arena_rectangles_and_circles(arena_boxes, arena_cylinders):
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
        # if plot:
        #     ax.add_patch(rect)


    arena_circles = []
    for arena_cylinder in arena_cylinders:
        circle = patches.Circle(
            (arena_cylinder['x'], arena_cylinder['y']),
            arena_cylinder['radius'],
            linewidth=0, facecolor='gray', alpha=1
        )
        arena_circles.append(circle)
        # if plot:
        #     ax.add_patch(circle)
    
    return arena_rectangles, arena_circles

def union_map(rectangles, circles):
    l_center_radius_circles = [(circle.center, circle.radius) for circle in circles]
    l_corners = [get_rotated_corners(rect2) for rect2 in rectangles]
    
    merged_rectangles = merge_rectangles(l_corners)
    merged_circles = merge_circles(l_center_radius_circles)

    merged_rectangles_and_circles = unary_union([merged_rectangles, merged_circles])
    return merged_rectangles_and_circles

def union_u_map_and_spawn_boxes(u_map, spawn_boxes):
    """
    Merges the union of rectangles and circles with spawn boxes.
    """

    l_corners = [get_rotated_corners(rect2) for rect2 in spawn_boxes]
    merged_rectangles = merge_rectangles(l_corners)
    # Merge the union map with spawn boxes
    merged_u_map = unary_union([u_map, merged_rectangles])
    
    return merged_u_map

def create_map_from_arena(arena_boxes, arena_cylinders):
    arena_rectangles, arena_circles = create_arena_rectangles_and_circles(arena_boxes, arena_cylinders)
    return union_map(arena_rectangles, arena_circles)

def divide_map_into_grid_tree(union_map, cell_size=0.5):
    """
    Divides a polygon into grid cells and stores them in an STRTree.
    """
    minx, miny, maxx, maxy = union_map.bounds
    
    # Generate horizontal split lines
    horizontal_lines = [LineString([(minx, y), (maxx, y)]) for y in np.arange(miny, maxy, cell_size)]
    
    # First split horizontally
    split_polygons = [union_map]
    for line in horizontal_lines:
        split_polygons = [piece for poly in split_polygons for piece in list(split(poly, line).geoms)]

    # Generate vertical split lines
    vertical_lines = [LineString([(x, miny), (x, maxy)]) for x in np.arange(minx, maxx, cell_size)]
    
    # Then split each resulting piece vertically
    grid_cells = []
    for poly in split_polygons:
        sub_polygons = [poly]
        for line in vertical_lines:
            sub_polygons = [piece for p in sub_polygons for piece in list(split(p, line).geoms)]
        grid_cells.extend(sub_polygons)

    # Build spatial tree
    tree = STRtree(grid_cells)
    
    return tree, grid_cells

def process_row(row, resolution, actual_arena_polygon, actual_arena2_polygon, outside_area, str_tree, str_tree_with_spawn_boxes, grid, grid_with_spawn_boxes, plot, ax):
    box_size = float(row['box_size'])
    #due to rounding errors, we need to round the box size to the nearest resolution
    for i in range(1,7):
        if equals_with_epsilon(box_size, resolution*i):
            box_size = resolution*i
    box_x = float(row['box_x']) - box_size / 2
    box_y = float(row['box_y']) - box_size / 2
    pheromone = float(row['pheromone'])

    sub_boxes = []

    # We split it into smaller boxes if applicable
    n_cells = box_size / resolution
    for i in range(int(n_cells)):
        for j in range(int(n_cells)):
            sub_boxes.append((box_x + i * resolution, box_y + j * resolution))

    results = []
    
    for box_x, box_y in sub_boxes:
        box_size = resolution
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')

        # Update boundaries
        min_x = min(min_x, box_x)
        max_x = max(max_x, box_x + box_size)
        min_y = min(min_y, box_y)
        max_y = max(max_y, box_y + box_size)

        box_rect = patches.Rectangle((box_x, box_y), box_size, box_size)
        boxcorners = get_rotated_corners(box_rect)
        box_polygon = Polygon(boxcorners)

        # If the entire box is outside the actual arena, ignore it
        if not actual_arena_polygon.intersects(box_polygon):
            results.append((box_x, box_y, -1, -1, True))
            continue
        if 'museum' in env_map:
            if not actual_arena2_polygon.intersects(box_polygon):
                results.append((box_x, box_y, -1, -1, True))
                continue
        skip = False
        for (x, y) in outside_area:
            if coordinate_equals_with_epsilon((x, y), (box_x, box_y)):
                skip = True
                break
        if skip:
            results.append((box_x, box_y, -1, -1, True))
            continue

        # Check overlap
        box_covered_area_irrelevant = check_rectangle_map_overlap(box_rect, str_tree, grid)
        if box_covered_area_irrelevant >= 0.9999999:
            # The box is completely inside the map
            results.append((box_x, box_y, -1, -1, True))
            continue
        box_covered_area = check_rectangle_map_overlap(box_rect, str_tree_with_spawn_boxes, grid_with_spawn_boxes)
        box_contains_arena = box_covered_area > 0.1
        results.append((box_x, box_y, box_size, pheromone, box_contains_arena))
        
    return results

def parallel_process(quadtree_data, resolution, actual_arena_polygon, actual_arena2_polygon, outside_area, str_tree, str_tree_with_spawn_boxes, grid, grid_with_spawn_boxes, plot, ax):
    true_positives = 0
    false_positives = 0
    false_negatives = 0
    n_cells_outside_area = 0


    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = [executor.submit(process_row, row, resolution, actual_arena_polygon, actual_arena2_polygon, outside_area, str_tree, str_tree_with_spawn_boxes, grid, grid_with_spawn_boxes, plot, ax) for row in quadtree_data]
        
        # Collect the results as they are completed
        for future in concurrent.futures.as_completed(futures):
            results = future.result()
            # You can now process the results, e.g., accumulate them or update shared data
            # This could involve updating `true_positives`, `false_negatives`, etc.
            for res in results:
                box_x, box_y, box_size, pheromone, box_contains_arena = res
                if box_size == -1:
                    # print("Box: ", box_x, box_y, "is outside the arena")
                    n_cells_outside_area += 1
                    continue
                # Determine results based on pheromone and overlap status
                if box_contains_arena and pheromone < 0.5:
                    true_positives += 1 # Correctly identified as occupied
                    color = 'red'
                elif box_contains_arena and pheromone >= 0.5:
                    false_negatives += 1 # Incorrectly identified as free (actually occupied)
                    color = 'pink'
                elif not box_contains_arena and pheromone >= 0.5:
                    true_positives += 1 # Correctly identified as free
                    color = 'green'
                else:
                    false_positives += 1 # Incorrectly identified as occupied (actually free)
                    color = 'yellow'
                
                if plot:    
                    rect = patches.Rectangle((box_x, box_y), box_size, box_size, color=color, alpha=0.5)
                    ax.add_patch(rect)
                
                # # Store result for later use
                # results.append((box_x, box_y, color))
    return true_positives, false_positives, false_negatives, n_cells_outside_area

def get_spawn_times(log_data, ticks_per_second):

    # Regex pattern to find lines like:
    # Found spawn box: spawn_box_0
    # Spawn time: 3476
    pattern = r'Found spawn box: (spawn_box_\d+)\s+Spawn time: (\d+)'

    # Find all matches
    matches = [(box_id, float(int(spawn_time)/ticks_per_second)) for box_id, spawn_time in re.findall(pattern, log_data)]

    # Sort by spawn_time (second item in the tuple)
    sorted_spawn_data = sorted(matches, key=lambda x: x[1])

    # Print results
    for box_id, spawn_time in sorted_spawn_data:
        print(f"Spawn box {box_id}: Spawn time {spawn_time}")

    return sorted_spawn_data


        
def calculate_precision_recall_coverage(union_map, str_tree, str_tree_with_spawn_boxes, grid, grid_with_spawn_boxes, quadtree_data, spawn_time, plot):
        
    ax = None
    if plot:
        fig, ax = plt.subplots()
        # for poly in union_map.geoms:
        #     x, y = poly.exterior.xy
        #     # ax.plot(x, y, 'b-', linewidth=2)  # Blue outline
        #     ax.fill(x, y, color="grey", alpha=0.5, edgecolor="grey")
        #     # Plot interior holes (if any)
        #     for interior in poly.interiors:
        #         x, y = interior.xy
        #         ax.fill(x, y,color="grey", alpha=0.5    , edgecolor="grey")
        for poly in grid_with_spawn_boxes:
            x, y = poly.exterior.xy
            patch = patches.Polygon(list(poly.exterior.coords), closed=True, facecolor="grey", edgecolor="none", alpha=0.5)
            ax.add_patch(patch)
            # ax.plot(x, y, 'b-', linewidth=2)  # Blue outline
            # ax.fill(x, y, color="grey", alpha=0.5, linewidth=0)
            # Plot interior holes (if any)
            # for interior in poly.interiors:
            #     x, y = interior.xy
            #     ax.fill(x, y,color="grey", alpha=0.5    , edgecolor="grey")
    
    actual_arena_corners = get_rotated_corners(actual_arena)
    actual_arena_polygon = Polygon(actual_arena_corners)

    if 'museum' in env_map:
        actual_arena2_corners = get_rotated_corners(actual_arena2)
        actual_arena2_polygon = Polygon(actual_arena2_corners)
    else:
        actual_arena2_polygon = None

    min_x = -5
    max_x = 5
    min_y = -5
    max_y = 5


    # agent_id = 'pipuck1'  # Replace with the actual agent ID you want to consider


    true_positives, false_positives, false_negatives, n_cell_outside_area = parallel_process(quadtree_data, resolution, actual_arena_polygon, actual_arena2_polygon, outside_area, str_tree, str_tree_with_spawn_boxes, grid, grid_with_spawn_boxes, plot, ax)
    
    covered_cells = true_positives + false_positives + false_negatives
    coverage = covered_cells*resolution*resolution / get_total_area_from_map_spawn_time(env_map, spawn_time)
    covered_invalid_area = n_cell_outside_area*resolution*resolution

    precision = true_positives / (true_positives + false_positives)
    recall = true_positives / (true_positives + false_negatives)
    print("True Positives: ", true_positives)
    print("False Positives: ", false_positives)
    print("False Negatives: ", false_negatives)

    print(f'Precision: {precision:.4f}, Recall: {recall:.4f}')
    print(f'Coverage: {coverage:.4f}')
    print(f'Covered Invalid Area: {covered_invalid_area:.4f}')
    if plot:
        ax.set_aspect('equal', 'box')
        # plt.xlim(min_x, max_x)
        # plt.ylim(min_y, max_y)
        plt.xlim(-actual_arena_width/2-1, actual_arena_width/2+1)
        plt.ylim(-actual_arena_height/2-1, actual_arena_height/2+1)
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Mistakes in Quadtree Results')

        plt.grid(False)
        plt.show()
    return precision, recall, coverage, covered_invalid_area


def parse_experiment_string(s):
    # Regular expression to match key-value pairs
    pattern = r'([a-zA-Z_]+)_(-?\d+(?:_\d+)?)'
    matches = re.findall(pattern, s)

    # Convert values: replace underscores in numbers with a decimal point
    parsed_config = {key: float(value.replace('_', '.')) if '_' in value else int(value) for key, value in matches}
    
    end_time = parsed_config['end_time']
    noise = parsed_config['_noise']
    comm_range = parsed_config['_wifi_range']
    message_loss_probability = parsed_config['_message_loss_probability']
    frontier_search_radius = parsed_config['_frontier_search_radius']
    evaporation_time = parsed_config['_evaporation_time']
    max_frontier_cells = parsed_config['_max_frontier_cells']
    max_route_length = parsed_config['_max_route_length']
    
    return end_time, noise, comm_range, message_loss_probability, frontier_search_radius, evaporation_time, max_frontier_cells, max_route_length





# data_file = 'metric_plotting_scripts/quadtree_finished_exploring_pipuck1.csv'
# split_exp = data_file.split('/')
# # config = split_exp[-5]
# # spawn_time = split_exp[-4]
# spawn_time = 'spawn_time_0'
# # end_time, noise, comm_range, message_loss_probability, frontier_search_radius, evaporation_time, max_frontier_cells, max_route_length = parse_experiment_string(config)
# print("setting variables based on map")
# set_variables_based_on_map(map)

# print("reading arena boxes and cylinders")
# # # Usage
# arena_boxes = read_arena_boxes(spawn_time)
# arena_cylinders = read_arena_cylinders()

# print("creating arena rectangles and circles")
# arena_rectangles, arena_circles = create_arena_rectangles_and_circles(arena_boxes, arena_cylinders)
# print("union map")
# u_map = union_map(arena_rectangles, arena_circles)
# print("divide map into grid tree")
# gridtree, grid = divide_map_into_grid_tree(u_map)

# print("reading quadtree data")
# quadtree_data = read_quadtree_data(data_file)
# # smallest_box_size = find_smallest_box_size(quadtree_data)
# # print("smallest box size: ", smallest_box_size)
# # resolution = smallest_box_size

# print("calculating precision recall")
# precision, recall = calculate_precision_recall(u_map,gridtree, grid, quadtree_data, spawn_time, True)
