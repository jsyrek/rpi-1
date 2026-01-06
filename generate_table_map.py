#!/usr/bin/env python3
import numpy as np
from PIL import Image
import yaml
import os

TABLE_WIDTH_M = 0.4
TABLE_HEIGHT_M = 1.5
RESOLUTION_M_PER_PX = 0.01
EDGE_PX = 5
MAP_NAME = "table_2x1.5m_edges"

print(f"Generuję mapę {TABLE_WIDTH_M}x{TABLE_HEIGHT_M}m...")

w_px = int(TABLE_WIDTH_M / RESOLUTION_M_PER_PX)
h_px = int(TABLE_HEIGHT_M / RESOLUTION_M_PER_PX)

map_px = np.ones((h_px, w_px), dtype=np.uint8) * 255

map_px[0:EDGE_PX, :] = 0
map_px[-EDGE_PX:, :] = 0
map_px[:, 0:EDGE_PX] = 0
map_px[:, -EDGE_PX:] = 0

pgm_file = os.path.expanduser(f"~/maps/{MAP_NAME}.pgm")
Image.fromarray(map_px, 'L').save(pgm_file)

yaml_file = os.path.expanduser(f"~/maps/{MAP_NAME}.yaml")
with open(yaml_file, 'w') as f:
    yaml.dump({
        'image': f'{MAP_NAME}.pgm',
        'resolution': RESOLUTION_M_PER_PX,
        'origin': [0.0, 0.0, 0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'negate': 0
    }, f)

print(f"✅ PGM: {pgm_file}")
print(f"✅ YAML: {yaml_file}")
print(f"   Wymiary: {TABLE_WIDTH_M}x{TABLE_HEIGHT_M}m = {w_px}x{h_px}px")

