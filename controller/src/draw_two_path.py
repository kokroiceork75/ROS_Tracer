#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml
import sys

def load_pgm_map(pgm_path):
    return mpimg.imread(pgm_path)

def load_recorded_data(data_path):
    times, x_coords, y_coords, lin_vels = [], [], [], []
    with open(data_path, 'r') as f:
        lines = f.readlines()[1:]  # Skip header
        for line in lines:
            values = line.strip().split('\t')
            times.append(float(values[0]))
            x_coords.append(float(values[1]))
            y_coords.append(float(values[2]))
            lin_vels.append(float(values[7]))  # linear velocity column
    return times, x_coords, y_coords, lin_vels

def load_recorded_data_with_flags(data_path):
    times, x_coords, y_coords, lin_vels, controller_flags = [], [], [], [], []
    with open(data_path, 'r') as f:
        lines = f.readlines()[1:]  # Skip header
        for line in lines:
            values = line.strip().split('\t')
            times.append(float(values[0]))
            x_coords.append(float(values[1]))
            y_coords.append(float(values[2]))
            lin_vels.append(float(values[7]))  # linear velocity column
            controller_flags.append(int(values[9]))  # controller_flag column
    return times, x_coords, y_coords, lin_vels, controller_flags

def world_to_map(x, y, origin_x, origin_y, resolution, map_height):
    map_x = int((x - origin_x) / resolution)
    map_y = int(map_height - ((y - origin_y) / resolution))
    return map_x, map_y

def compute_total_distance(x_coords, y_coords):
    dist = 0.0
    for i in range(1, len(x_coords)):
        dx = x_coords[i] - x_coords[i-1]
        dy = y_coords[i] - y_coords[i-1]
        dist += np.hypot(dx, dy)
    return dist

def plot_paths():
    map_name = "paper4"
    map_yaml_path = f"/home/user/{map_name}.yaml"
    map_path = f"/home/user/{map_name}.pgm"
    data_path = "/home/user/wei_ws/src/controller/src/data/data_fuzzy4.txt"
    dwa_path = "/home/user/wei_ws/src/controller/src/data/data_mpc4.txt"
    #dwa_path = "/home/user/wei_ws/src/controller/src/data/data_dwa6.txt"
    try:
        with open(map_yaml_path, 'r') as yaml_file:
            map_params = yaml.safe_load(yaml_file)
            resolution = map_params['resolution']
            origin = map_params['origin']
            origin_x, origin_y = origin[0], origin[1]
    except Exception as e:
        print(f"Failed to load YAML: {e}")
        return
    
    try:
        map_img = load_pgm_map(map_path)
        map_height = map_img.shape[0]
    except Exception as e:
        print(f"Failed to load map image: {e}")
        return
    
    # Create figure
    plt.figure(figsize=(10, 10))
    plt.imshow(map_img, cmap='gray')
    
    # Load and plot data.txt (with controller flags)
    try:
        # Load data with controller flags
        times, x_coords, y_coords, lin_vels, controller_flags = load_recorded_data_with_flags(data_path)
        
        # Calculate statistics
        total_distance = compute_total_distance(x_coords, y_coords)
        avg_speed = total_distance / (times[-1] - times[0]) if times[-1] != times[0] else 0.0
        
        # Convert to map coordinates
        map_x, map_y = [], []
        for x, y in zip(x_coords, y_coords):
            mx, my = world_to_map(x, y, origin_x, origin_y, resolution, map_height)
            map_x.append(mx)
            map_y.append(my)
        
        # Plot points based on controller_flag
        for i, flag in enumerate(controller_flags):
            if flag == 1:  # Target Search (TS)
                plt.scatter(map_x[i], map_y[i], c='blue', s=10, label='Target Search' if i == 0 or controller_flags[i-1] != 1 else '')
            elif flag == 2:  # Obstacle Boundary Following (OBF)
                plt.scatter(map_x[i], map_y[i], c='red', s=10, label='Obstacle Boundary Following' if i == 0 or controller_flags[i-1] != 2 else '')
            elif flag == 3:  # Spin
                plt.scatter(map_x[i], map_y[i], c='green', s=10, label='Spin' if i == 0 or controller_flags[i-1] != 3 else '')
            else:  # controller_flag = 0 or invalid
                plt.scatter(map_x[i], map_y[i], c='gray', s=10, label='Stopped/Unknown' if i == 0 or controller_flags[i-1] != flag else '')
        
        # Mark start and end
        plt.plot(map_x[0], map_y[0], 'go', markersize=8, label='Controller Start')
        plt.plot(map_x[-1], map_y[-1], 'ro', markersize=8, label='Controller End')
        
        # Annotate total distance and avg speed
        plt.annotate(f'Controller Path Distance: {total_distance:.2f} m\nController Avg Speed: {avg_speed:.2f} m/s',
                    xy=(10, 50), xycoords='axes pixels',
                    bbox=dict(facecolor='white', alpha=0.8))
        
    except Exception as e:
        print(f"Failed to load and plot data.txt: {e}")
    
    # Load and plot dwa.txt (without controller flags)
    try:
        # Load DWA data without controller flags
        dwa_times, dwa_x_coords, dwa_y_coords, dwa_lin_vels = load_recorded_data(dwa_path)
        
        # Calculate statistics
        dwa_total_distance = compute_total_distance(dwa_x_coords, dwa_y_coords)
        dwa_avg_speed = dwa_total_distance / (dwa_times[-1] - dwa_times[0]) if dwa_times[-1] != dwa_times[0] else 0.0
        
        # Convert to map coordinates
        dwa_map_x, dwa_map_y = [], []
        for x, y in zip(dwa_x_coords, dwa_y_coords):
            mx, my = world_to_map(x, y, origin_x, origin_y, resolution, map_height)
            dwa_map_x.append(mx)
            dwa_map_y.append(my)
        
        # Plot DWA path with a single color
        plt.scatter(dwa_map_x, dwa_map_y, c='purple', s=10, label='DWA Path')
        
        # Mark start and end
        plt.plot(dwa_map_x[0], dwa_map_y[0], 'co', markersize=8, label='DWA Start')
        plt.plot(dwa_map_x[-1], dwa_map_y[-1], 'mo', markersize=8, label='DWA End')
        
        # Annotate total distance and avg speed
        plt.annotate(f'DWA Path Distance: {dwa_total_distance:.2f} m\nDWA Avg Speed: {dwa_avg_speed:.2f} m/s',
                    xy=(10, 120), xycoords='axes pixels',
                    bbox=dict(facecolor='white', alpha=0.8))
        
    except Exception as e:
        print(f"Failed to load and plot dwa.txt: {e}")
    
    plt.title(f"Robot Paths on {map_name} Map")
    plt.xlabel('X (pixels)')
    plt.ylabel('Y (pixels)')
    plt.legend()
    plt.grid(True)
    height, width = map_img.shape
    plt.xlim(0, width)
    plt.ylim(height, 0)
    plt.tight_layout()
    plt.savefig(f"/home/user/wei_ws/src/controller/src/data/path_comparison_{map_name}.png", dpi=300)
    plt.show()

if __name__ == "__main__":
    plot_paths()
