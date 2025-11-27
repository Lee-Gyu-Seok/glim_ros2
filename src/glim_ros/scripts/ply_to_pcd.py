#!/usr/bin/env python3
"""
PLY to PCD converter script for GLIM map files.
Usage: python3 ply_to_pcd.py <input.ply>
Output: map.pcd in the same directory as input file
"""

import sys
import os
import numpy as np

def read_ply(filepath):
    """Read PLY file and return points as numpy array."""
    with open(filepath, 'rb') as f:
        # Read header
        header_lines = []
        while True:
            line = f.readline().decode('utf-8').strip()
            header_lines.append(line)
            if line == 'end_header':
                break

        # Parse header
        vertex_count = 0
        properties = []
        is_binary = False

        for line in header_lines:
            if line.startswith('element vertex'):
                vertex_count = int(line.split()[-1])
            elif line.startswith('property'):
                parts = line.split()
                properties.append((parts[1], parts[2]))  # (type, name)
            elif line.startswith('format'):
                is_binary = 'binary' in line

        # Read data
        if is_binary:
            # Determine dtype
            dtype_map = {
                'float': np.float32,
                'float32': np.float32,
                'double': np.float64,
                'float64': np.float64,
                'uchar': np.uint8,
                'uint8': np.uint8,
                'int': np.int32,
                'int32': np.int32,
            }

            dtype_list = []
            for prop_type, prop_name in properties:
                dtype_list.append((prop_name, dtype_map.get(prop_type, np.float32)))

            data = np.frombuffer(f.read(), dtype=np.dtype(dtype_list), count=vertex_count)
        else:
            # ASCII format
            data = np.loadtxt(f, max_rows=vertex_count)
            if len(properties) > 0:
                dtype_list = [(prop[1], np.float32) for prop in properties]
                structured = np.zeros(vertex_count, dtype=dtype_list)
                for i, (_, name) in enumerate(properties):
                    if data.ndim == 1:
                        structured[name] = data[i] if i < len(data) else 0
                    else:
                        structured[name] = data[:, i] if i < data.shape[1] else 0
                data = structured

    return data, properties

def write_pcd(filepath, data, properties):
    """Write PCD file from point data."""
    # Extract x, y, z coordinates
    has_x = 'x' in data.dtype.names
    has_y = 'y' in data.dtype.names
    has_z = 'z' in data.dtype.names

    if not (has_x and has_y and has_z):
        raise ValueError("PLY file must contain x, y, z coordinates")

    points = np.column_stack([data['x'], data['y'], data['z']])

    # Check for intensity or RGB
    has_intensity = 'intensity' in data.dtype.names
    has_rgb = all(c in data.dtype.names for c in ['red', 'green', 'blue'])

    num_points = len(points)

    with open(filepath, 'w') as f:
        # Write PCD header
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")

        if has_intensity:
            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
        elif has_rgb:
            f.write("FIELDS x y z rgb\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F U\n")
            f.write("COUNT 1 1 1 1\n")
        else:
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")

        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")

        # Write points
        for i in range(num_points):
            x, y, z = points[i]
            if has_intensity:
                intensity = data['intensity'][i]
                f.write(f"{x} {y} {z} {intensity}\n")
            elif has_rgb:
                r = int(data['red'][i])
                g = int(data['green'][i])
                b = int(data['blue'][i])
                rgb = (r << 16) | (g << 8) | b
                f.write(f"{x} {y} {z} {rgb}\n")
            else:
                f.write(f"{x} {y} {z}\n")

    print(f"Saved {num_points} points to {filepath}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 ply_to_pcd.py <input.ply>")
        print("Output: map.pcd in the same directory as input file")
        sys.exit(1)

    input_path = sys.argv[1]

    if not os.path.exists(input_path):
        print(f"Error: File not found: {input_path}")
        sys.exit(1)

    # Output path: same directory, named map.pcd
    output_dir = os.path.dirname(input_path)
    output_path = os.path.join(output_dir, "map.pcd")

    print(f"Reading PLY file: {input_path}")
    data, properties = read_ply(input_path)

    print(f"Converting to PCD format...")
    write_pcd(output_path, data, properties)

    print("Done!")

if __name__ == "__main__":
    main()
