"""
map_file_io.py
Save/load an occupancy grid in the exact same text format as map_io.h (C side),
so a map built by the Python GUI can be loaded directly by main_shuttle.c on the
real robot, and vice versa.

File format:
  line 1: rows cols resolution
  line 2+: each row as a string of '0'/'1' characters
"""


def save_map(path, grid_2d, resolution):
    """grid_2d: list of lists (rows x cols), each cell 0 or 1."""
    rows = len(grid_2d)
    cols = len(grid_2d[0]) if rows > 0 else 0
    with open(path, "w") as f:
        f.write(f"{rows} {cols} {resolution:.6f}\n")
        for row in grid_2d:
            f.write("".join("1" if c else "0" for c in row) + "\n")
    print(f"[map_file_io] map saved: {path} ({rows} x {cols}, resolution={resolution:.3f})")


def load_map(path):
    """Returns (grid_2d, rows, cols, resolution)."""
    with open(path, "r") as f:
        header = f.readline().split()
        rows, cols, resolution = int(header[0]), int(header[1]), float(header[2])
        grid_2d = []
        for r in range(rows):
            line = f.readline().rstrip("\n")
            grid_2d.append([1 if ch == "1" else 0 for ch in line[:cols]])
    print(f"[map_file_io] map loaded: {path} ({rows} x {cols}, resolution={resolution:.3f})")
    return grid_2d, rows, cols, resolution


def save_points(path, points):
    """points: dict of {label: (x, y, theta_deg)}."""
    with open(path, "w") as f:
        for label, (x, y, theta_deg) in points.items():
            f.write(f"{label} {x:.4f} {y:.4f} {theta_deg:.2f}\n")
    print(f"[map_file_io] points saved: {path}")


def load_points(path):
    """Returns dict of {label: (x, y, theta_deg)}."""
    points = {}
    with open(path, "r") as f:
        for line in f:
            parts = line.split()
            if len(parts) != 4:
                continue
            label, x, y, theta_deg = parts
            points[label] = (float(x), float(y), float(theta_deg))
    print(f"[map_file_io] points loaded: {path}")
    return points
