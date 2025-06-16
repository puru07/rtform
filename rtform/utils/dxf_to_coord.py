#!/usr/bin/env python3

import ezdxf
import matplotlib.pyplot as plt
import numpy as np
import json
import os
from pathlib import Path


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


def extract_figure(msp,  percentage_increase=20, title="DXF Viewer",):
    """
    Plot the DXF entities using matplotlib and return a dictionary of points for each entity.
    Also, calculates and prints the four corners of the DXF drawing and increases the dimensions by 20%.
    """
    fig, ax = plt.subplots()
    entity_points = {}  # Dictionary to store points for each entity
    entity_count = 0

    # Initialize variables to track the bounding box
    min_x, min_y = float('inf'), float('inf')
    max_x, max_y = float('-inf'), float('-inf')

    for entity in msp:
        entity_count += 1
        if entity.dxftype() == 'LWPOLYLINE':
            pts = [(p[0], p[1]) for p in entity.get_points()]
            pts.append(pts[0])  # Close the polyline
            entity_points[entity_count] = pts
            xs, ys = zip(*pts)
            ax.plot(xs, ys, linewidth=1.0)

            # Update bounding box
            for x, y in pts:
                min_x = min(min_x, x)
                min_y = min(min_y, y)
                max_x = max(max_x, x)
                max_y = max(max_y, y)

        elif entity.dxftype() == 'LINE':
            s, t = entity.dxf.start, entity.dxf.end
            pts = [(s.x, s.y), (t.x, t.y)]
            entity_points[entity_count] = pts
            ax.plot([s.x, t.x], [s.y, t.y], linewidth=1.0)
            ax.plot([s.x, t.x], [s.y, t.y], 'o', markersize=2)

            # Update bounding box
            min_x = min(min_x, s.x, t.x)
            min_y = min(min_y, s.y, t.y)
            max_x = max(max_x, s.x, t.x)
            max_y = max(max_y, s.y, t.y)

    # Print the original dimensions of the DXF file (bounding box)
    width = max_x - min_x
    height = max_y - min_y
    print(f"Original DXF Dimensions: Width = {width:.4f}, Height = {height:.4f}")

    # Increase the bounding box by the specified percentage
    width *= (1 + percentage_increase/100)
    height *= (1 + percentage_increase/100)

    # Calculate the new min_x, max_x, min_y, max_y by expanding the original box
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2

    # Adjust the corners to expand by 20%
    half_width = width / 2
    half_height = height / 2

    min_x = center_x - half_width
    max_x = center_x + half_width
    min_y = center_y - half_height
    max_y = center_y + half_height

    # Calculate the four corners after expanding the bounding box by 20%
    corners = [(min_x, min_y), (min_x, max_y), (max_x, min_y), (max_x, max_y)]
    print(f"Expanded DXF Dimensions: Width = {width:.4f}, Height = {height:.4f}")
    print(f"Corners of the expanded DXF bounding box: {corners}")

    # Plot the corners
    corners = np.array(corners)
    ax.plot(corners[:, 0], corners[:, 1], 'k*', markersize=10, label='Corners')

    ax.set_aspect('equal', adjustable='datalim')
    ax.margins(0.02)
    plt.grid(True)
    plt.legend()
    plt.title('DXF Drawing with Expanded Bounding Box')
    plt.show()
    
    return entity_points, corners  # Return corners as well



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


def analyze_and_interpolate_points(points_in_dxf, original_corners):
    """
    Analyze the minimum distances between points and interpolate new points to maintain consistent spacing.
    
    Args:
        points_in_dxf (dict): Dictionary containing points for each entity
        original_corners (list): List of corner points of the expanded bounding box
        
    Returns:
        dict: Dictionary containing interpolated points for each entity and corners
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

    # Create a new dictionary that includes both interpolated points and corners
    points_and_corners = {'entities': interpolated_points, 'corners': original_corners}

    # Create a new figure for plotting points
    fig, ax = plt.subplots()
    
    # Plot interpolated points
    for entity_id, points in interpolated_points.items():
        xs, ys = zip(*points)
        ax.plot(xs, ys, 'o', markersize=4, label=f'Interpolated Entity {entity_id}')
    
    # Plot corners
    corners = np.array(original_corners)
    ax.plot(corners[:, 0], corners[:, 1], 'k*', markersize=10, label='Corners')
    
    ax.set_aspect('equal', adjustable='datalim')
    ax.margins(0.02)
    plt.grid(True)
    plt.legend()
    plt.title('Interpolated Points and Corners')
    plt.show()
    
    return points_and_corners


def process_dxf_to_json(dxf_path, output_dir=None, percentage_increase=20):
    """
    Process a DXF file and save the interpolated points and corners to a JSON file.
    
    Args:
        dxf_path (str): Path to the input DXF file
        output_dir (str, optional): Directory to save the JSON file. If None, saves in the same directory as the DXF file
        percentage_increase (int, optional): Percentage to increase the bounding box dimensions. Defaults to 20
        
    Returns:
        str: Path to the created JSON file
    """
    # Load and process the DXF file
    msp = load_dxf(dxf_path)
    points_in_dxf, original_corners = extract_figure(msp, percentage_increase=percentage_increase, title=f"DXF: {dxf_path}")
    result = analyze_and_interpolate_points(points_in_dxf, original_corners)
    
    # Print summary of results
    print("\nResults Summary:")
    print(f"Number of entities: {len(result['entities'])}")
    for entity_id, points in result['entities'].items():
        print(f"Entity {entity_id}: {len(points)} points")
    print(f"Number of corner points: {len(result['corners'])}")

    # Determine output path
    if output_dir is None:
        output_dir = os.path.dirname(dxf_path)
    else:
        os.makedirs(output_dir, exist_ok=True)
    
    # Create output filename based on input filename
    input_filename = os.path.basename(dxf_path)
    output_filename = f"{os.path.splitext(input_filename)[0]}.json"
    output_path = os.path.join(output_dir, output_filename)
    
    # Convert numpy arrays to lists for JSON serialization
    json_result = {
        'entities': {
            str(k): [[float(x) for x in p] for p in v] 
            for k, v in result['entities'].items()
        },
        'corners': [[float(x) for x in p] for p in result['corners']]
    }
    
    # Save to JSON file
    with open(output_path, 'w') as f:
        json.dump(json_result, f, indent=2)
    
    print(f"\nResults saved to: {output_path}")
    return output_path


if __name__ == "__main__":
    dxf_path = "../assets/cutting_patterns/custom_cutting_pattern_2.dxf"  # <-- Replace this with the path to your DXF file
    output_path = process_dxf_to_json(dxf_path)
    



    
    
