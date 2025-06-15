#!/usr/bin/env python3

import ezdxf
import matplotlib.pyplot as plt
import numpy as np


def load_dxf(file_path):
    """
    Load and return the modelspace entities from a DXF file.
    """
    doc = ezdxf.readfile(file_path)
    return doc.modelspace()


def calculate_min_distance(points):
    """
    Calculate the minimum distance between subsequent points in a list of points.
    """
    if len(points) < 2:
        return float('inf')
    
    distances = []
    for i in range(len(points) - 1):
        p1 = np.array(points[i])
        p2 = np.array(points[i + 1])
        distance = np.linalg.norm(p2 - p1)
        distances.append(distance)
    
    return min(distances) if distances else float('inf')


def extract_figure(msp, title="DXF Viewer", print_min_distances=True):
    """
    Plot the DXF entities using matplotlib and return a dictionary of points for each entity.
    """
    fig, ax = plt.subplots()
    entity_points = {}  # Dictionary to store points for each entity
    entity_count = 0

    for entity in msp:
        entity_count += 1
        if entity.dxftype() == 'LWPOLYLINE':
            pts = [(p[0], p[1]) for p in entity.get_points()]
            pts.append(pts[0])  # Close the polyline
            entity_points[entity_count] = pts
            xs, ys = zip(*pts)
            ax.plot(xs, ys, linewidth=1.0)
            if print_min_distances:
                min_dist = calculate_min_distance(pts)
                print(f"Entity {entity_count} (LWPOLYLINE) - Minimum distance between points: {min_dist:.4f}")

        elif entity.dxftype() == 'LINE':
            s, t = entity.dxf.start, entity.dxf.end
            pts = [(s.x, s.y), (t.x, t.y)]
            entity_points[entity_count] = pts
            ax.plot([s.x, t.x], [s.y, t.y], linewidth=1.0)
            ax.plot([s.x, t.x], [s.y, t.y], 'o', markersize=2)
            if print_min_distances:
                min_dist = calculate_min_distance(pts)
                print(f"Entity {entity_count} (LINE) - Distance between points: {min_dist:.4f}")

    ax.set_aspect('equal', adjustable='datalim')
    ax.margins(0.02)
    plt.grid(True)
    plt.show()
    
    return entity_points


def interpolate_points(entity_points, min_distance):
    """
    Interpolate points along each entity to maintain roughly equal distances between points.
    
    Args:
        entity_points (dict): Dictionary containing points for each entity
        min_distance (float): Target distance between interpolated points
        
    Returns:
        dict: Dictionary with interpolated points for each entity
    """
    interpolated_points = {}
    
    for entity_id, points in entity_points.items():
        interpolated_entity_points = []
        
        for i in range(len(points) - 1):
            p1 = np.array(points[i])
            p2 = np.array(points[i + 1])
            
            # Calculate distance between current points
            segment_length = np.linalg.norm(p2 - p1)
            
            # Calculate number of points to insert
            num_points = max(1, int(segment_length / min_distance))
            
            # Generate interpolated points
            for j in range(num_points + 1):
                t = j / num_points
                interpolated_point = p1 + t * (p2 - p1)
                interpolated_entity_points.append(tuple(interpolated_point))
        
        interpolated_points[entity_id] = interpolated_entity_points
        
    return interpolated_points


def analyze_and_interpolate_points(points_in_dxf):
    """
    Analyze the minimum distances between points and interpolate new points to maintain consistent spacing.
    
    Args:
        points_in_dxf (dict): Dictionary containing points for each entity
        
    Returns:
        dict: Dictionary containing interpolated points for each entity
    """
    # Calculate minimum distances for each entity
    min_distances = {}
    for entity_id, points in points_in_dxf.items():
        min_distances[entity_id] = calculate_min_distance(points)

    # Interpolate points for each entity using their respective minimum distances
    interpolated_points = {}
    for entity_id, points in points_in_dxf.items():
        min_dist = min_distances[entity_id]
        interpolated_points[entity_id] = interpolate_points({entity_id: points}, min_dist)[entity_id]

    # Create a new figure for plotting points
    fig, ax = plt.subplots()
    

    # Plot interpolated points in blue
    for entity_id, points in interpolated_points.items():
        xs, ys = zip(*points)
        ax.plot(xs, ys, 'o', markersize=4, label=f'Interpolated Entity {entity_id}')
    
    ax.set_aspect('equal', adjustable='datalim')
    ax.margins(0.02)
    plt.grid(True)
    plt.legend()
    plt.title('Original (red) and Interpolated (blue) Points')
    plt.show()

    # Print results
    print("\nInterpolated points for each entity:")
    for entity_id, points in interpolated_points.items():
        print(f"Entity {entity_id}: {len(points)} points")
        # Calculate and print the average distance between points
        distances = []
        for i in range(len(points) - 1):
            p1 = np.array(points[i])
            p2 = np.array(points[i + 1])
            distances.append(np.linalg.norm(p2 - p1))
        avg_distance = sum(distances) / len(distances) if distances else 0
        print(f"Average distance between points: {avg_distance:.4f}")
    
    return interpolated_points
def extrapolate_points(interpolated_points, corners):
    """
    Extrapolate points based on the four corners of the projection.
    
    Args:
        interpolated_points (dict): Dictionary containing interpolated points for each entity
        corners (list): List of four corner points [(x1,y1), (x2,y2), (x3,y3), (x4,y4)]
        
    Returns:
        dict: Dictionary containing extrapolated points for each entity
    """
    # Convert corners to numpy arrays for easier calculations
    corners = np.array(corners)
    
    # Calculate the bounding box of the corners
    min_x = np.min(corners[:, 0])
    max_x = np.max(corners[:, 0])
    min_y = np.min(corners[:, 1])
    max_y = np.max(corners[:, 1])
    
    # Create a new figure for plotting
    fig, ax = plt.subplots()
    
    # Plot the corners
    ax.plot(corners[:, 0], corners[:, 1], 'k*', markersize=10, label='Corners')
    
    # Extrapolate points for each entity
    extrapolated_points = {}
    for entity_id, points in interpolated_points.items():
        points = np.array(points)

        if len(points) >= 2:
            direction = points[-1] - points[0]
            norm = np.linalg.norm(direction)
            if norm == 0:
                continue  # Skip extrapolation if direction vector is invalid
            direction = direction / norm

            extended_points = []
            for point in points:
                if (min_x <= point[0] <= max_x) and (min_y <= point[1] <= max_y):
                    extended_points.append(point)
                else:
                    t_values = []
                    if direction[0] != 0:
                        t_values.append((min_x - point[0]) / direction[0])
                        t_values.append((max_x - point[0]) / direction[0])
                    if direction[1] != 0:
                        t_values.append((min_y - point[1]) / direction[1])
                        t_values.append((max_y - point[1]) / direction[1])

                    valid_t = [t for t in t_values if t > 0]
                    if valid_t:
                        t = min(valid_t)
                        extended_point = point + t * direction
                        extended_points.append(extended_point)

            extrapolated_points[entity_id] = extended_points
            extended_points = np.array(extended_points)

            if extended_points.shape[0] > 0 and extended_points.ndim == 2:
                ax.plot(extended_points[:, 0], extended_points[:, 1], 'o', markersize=4, label=f'Entity {entity_id}')

    # Set plot properties
    ax.set_aspect('equal', adjustable='datalim')
    ax.margins(0.02)
    plt.grid(True)
    plt.legend()
    plt.title('Extrapolated Points')
    plt.show()
    
    return extrapolated_points


if __name__ == "__main__":
    dxf_path = "../assets/cutting_patterns/custom_cutting_pattern_2_converted.dxf"  # <-- Replace this with the path to your DXF file
    msp = load_dxf(dxf_path)
    points_in_dxf = extract_figure(msp, title=f"DXF: {dxf_path}")
    interpolated_points = analyze_and_interpolate_points(points_in_dxf)
    corners = [(0, 0), (0, 100), (100, 0), (100, 100)]
    extrapolated_points = extrapolate_points(interpolated_points, corners)
    
    
