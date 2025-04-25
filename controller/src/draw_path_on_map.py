#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from pathlib import Path
import yaml

def load_pgm_map(pgm_path):
    """Load PGM map file"""
    return mpimg.imread(pgm_path)

def load_recorded_data(data_path):
    """Load recorded data from txt file"""
    times = []
    x_coords = []
    y_coords = []
    with open(data_path, 'r') as f:
        lines = f.readlines()[1:]  # Skip header
        for line in lines:
            values = line.strip().split('\t')
            times.append(float(values[0]))
            x_coords.append(float(values[1]))
            y_coords.append(float(values[2]))
    return times, x_coords, y_coords

def world_to_map(x, y, origin_x, origin_y, resolution, map_height):
    """Convert world coordinates to map pixel coordinates, flipping Y for correct orientation"""
    map_x = int((x - origin_x) / resolution)
    map_y = int((map_height - (y - origin_y) / resolution))  # 翻轉 Y 座標
    return map_x, map_y

def plot_path_on_map():
    # Define the map environment name
    map_name = "eight_f"  # 可以修改這個變數來切換地圖

    # Construct file paths based on the map name
    map_yaml_path = f"/home/user/{map_name}.yaml"
    map_path = f"/home/user/{map_name}.pgm"
    data_path1 = f"/home/user/wei_ws/src/controller/src/data/recorded_data_{map_name}_dwa.txt"  # 第一組數據
    data_path2 = f"/home/user/wei_ws/src/controller/src/data/recorded_data_{map_name}_fuz.txt"  # 第二組數據 for fuzzy

    # Load parameters from YAML file
    try:
        with open(map_yaml_path, 'r') as yaml_file:
            map_params = yaml.safe_load(yaml_file)
            resolution = map_params['resolution']
            origin = map_params['origin']
            origin_x = origin[0]
            origin_y = origin[1]
    except FileNotFoundError:
        print(f"Error: YAML file not found at {map_yaml_path}")
        return
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
        return

    # Load map and data
    try:
        map_img = load_pgm_map(map_path)
        map_height = map_img.shape[0]  # 獲取地圖高度，用於翻轉 Y 座標
    except FileNotFoundError:
        print(f"Error: PGM map file not found at {map_path}")
        return

    try:
        times1, x_coords1, y_coords1 = load_recorded_data(data_path1)  # 載入第一組數據
        times2, x_coords2, y_coords2 = load_recorded_data(data_path2)  # 載入第二組數據
    except FileNotFoundError as e:
        print(f"Error loading recorded data: {e}")
        return

    # Create figure
    plt.figure(figsize=(10, 10))
    plt.imshow(map_img, cmap='gray')  # 使用預設 origin='upper'，保持地圖方向

    # Convert and plot first path (blue)
    map_coords_x1 = []
    map_coords_y1 = []
    for x, y in zip(x_coords1, y_coords1):
        mx, my = world_to_map(x, y, origin_x, origin_y, resolution, map_height)
        map_coords_x1.append(mx)
        map_coords_y1.append(my)
    plt.plot(map_coords_x1, map_coords_y1, 'b-', linewidth=1, label='Path 1 (DWA)')

    # Label points for first path
    points_to_label1 = [
        (0, 'Start 1', 'green'),
        (len(x_coords1)//2, 'Middle 1', 'yellow'),
        (-1, 'End 1', 'red')
    ]
    for idx, label, color in points_to_label1:
        mx, my = map_coords_x1[idx], map_coords_y1[idx]
        plt.plot(mx, my, 'o', color=color, markersize=8)
        plt.annotate(f'{label}\n({x_coords1[idx]:.2f}, {y_coords1[idx]:.2f})',
                     (mx, my),
                     xytext=(5, 5),
                     textcoords='offset points',
                     color=color,
                     bbox=dict(facecolor='white', alpha=0.8))

    # Convert and plot second path (orange)
    map_coords_x2 = []
    map_coords_y2 = []
    for x, y in zip(x_coords2, y_coords2):
        mx, my = world_to_map(x, y, origin_x, origin_y, resolution, map_height)
        map_coords_x2.append(mx)
        map_coords_y2.append(my)
    plt.plot(map_coords_x2, map_coords_y2, 'orange', linewidth=1, label='Path 2 (FC)')

    # Label points for second path
    points_to_label2 = [
        (0, 'Start 2', 'green'),
        (len(x_coords2)//2, 'Middle 2', 'yellow'),
        (-1, 'End 2', 'red')
    ]
    for idx, label, color in points_to_label2:
        mx, my = map_coords_x2[idx], map_coords_y2[idx]
        plt.plot(mx, my, 'o', color=color, markersize=8)
        plt.annotate(f'{label}\n({x_coords2[idx]:.2f}, {y_coords2[idx]:.2f})',
                     (mx, my),
                     xytext=(5, 5),
                     textcoords='offset points',
                     color=color,
                     bbox=dict(facecolor='white', alpha=0.8))

    # Add map details
    height, width = map_img.shape
    plt.title(f'Robot Paths on Map\nResolution: {resolution}m/pixel, Origin: ({origin_x}, {origin_y})')
    plt.xlabel('X (pixels)')
    plt.ylabel('Y (pixels)')
    plt.legend()
    plt.grid(True)

    # Set axis limits
    plt.xlim(0, width)
    plt.ylim(height, 0)  # 保持 Y 軸從上到下，與地圖一致

    # Save and show plot
    output_path = f"/home/user/wei_ws/src/controller/src/data/path_visualization_{map_name}.png"
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.show()
    print(f"Visualization saved to: {output_path}")

if __name__ == "__main__":
    plot_path_on_map()
