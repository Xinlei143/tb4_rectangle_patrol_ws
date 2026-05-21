from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class FreeSpaceBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def contains(self, x, y, margin=0.0):
        return (
            self.min_x + margin <= x <= self.max_x - margin
            and self.min_y + margin <= y <= self.max_y - margin
        )


def _read_ascii_pgm(path):
    tokens = path.read_text(encoding='ascii').split()
    if tokens[0] != 'P2':
        raise ValueError(f'Only ASCII PGM P2 maps are supported: {path}')
    width = int(tokens[1])
    height = int(tokens[2])
    max_value = int(tokens[3])
    pixels = [int(value) for value in tokens[4:]]
    if len(pixels) != width * height:
        raise ValueError(f'PGM pixel count does not match dimensions: {path}')
    return width, height, max_value, pixels


def load_free_space_bounds(map_yaml_path):
    map_yaml_path = Path(map_yaml_path)
    metadata = yaml.safe_load(map_yaml_path.read_text(encoding='utf-8'))
    resolution = float(metadata['resolution'])
    origin_x = float(metadata['origin'][0])
    origin_y = float(metadata['origin'][1])
    free_thresh = float(metadata['free_thresh'])
    occupied_thresh = float(metadata['occupied_thresh'])

    image_path = map_yaml_path.parent / metadata['image']
    width, height, max_value, pixels = _read_ascii_pgm(image_path)

    free_cells = []
    for row in range(height):
        for col in range(width):
            value = pixels[row * width + col]
            occupancy = (max_value - value) / max_value
            if occupancy <= free_thresh and occupancy < occupied_thresh:
                free_cells.append((col, row))

    if not free_cells:
        raise ValueError(f'No free cells found in map: {map_yaml_path}')

    cols = [cell[0] for cell in free_cells]
    rows = [cell[1] for cell in free_cells]

    min_x = origin_x + (min(cols) + 0.5) * resolution
    max_x = origin_x + (max(cols) + 0.5) * resolution
    min_y = origin_y + (height - max(rows) - 0.5) * resolution
    max_y = origin_y + (height - min(rows) - 0.5) * resolution
    return FreeSpaceBounds(min_x=min_x, max_x=max_x, min_y=min_y, max_y=max_y)
