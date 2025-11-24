import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.ops import unary_union
import pandas as pd
import os
import glob

# -------------------
# Constants
# -------------------
ALPHA = 90  # Angle in degrees
VECTOR_LENGTH = 0.2  # Arrow length for antenna orientation
PLOT_LIMITS = (-0.5, 4)
GRID_STEP = 0.5

# -------------------
# Utility Functions
# -------------------
def get_rms_rssi(rssi):
    """
    Calculate RMS based on the given RSSI value using the formula:
    RMS = 0.0000330307 * exp(-0.154 * RSSI) + 0.9145 + 1.5 + 2
    """
    return 0.0000330307 * np.exp(-0.154 * rssi) + 0.9145 + 1.5 + 2

def rssi_distance(d):
    """Calculate RSSI based on distance using a 6th order polynomial fit."""
    return (-30.625214*d**0 + -66.049565*d**1 + 47.932897*d**2 +
            6.934334*d**3 + -23.319914*d**4 + 9.552617*d**5 + -1.222853*d**6)

def rssi_distance_deriv(d):
    """Derivative of the RSSI-distance polynomial."""
    return (-66.049565*d**0 + 2*47.932897*d**1 + 3*6.934334*d**2 +
            -4*23.319914*d**3 + 5*9.552617*d**4 + -6*1.222853*d**5)

def rssi_angle(phi):
    """Calculate RSSI based on angle."""
    return 0.038186*phi - 0.003704 * phi**2

def solve_distance(rssi, init=1.5):
    """Solve for distance given RSSI using fsolve."""
    def equation(d):
        return rssi - rssi_distance(d)
    def deriv(d):
        return -rssi_distance_deriv(d)
    sol = fsolve(equation, init, fprime=deriv)[0]
    return sol

def rotate_points(x, y, angle_deg):
    """Rotate points (x, y) by angle_deg around the origin."""
    angle_rad = np.radians(-angle_deg)
    x_rot = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_rot = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_rot, y_rot

def translate_points(x, y, ant_x, ant_y):
    """Translate points (x, y) to antenna location (ant_x, ant_y)."""
    return x + ant_x, y + ant_y

def plot_curve(rssi_val, ant_x, ant_y, angle_deg, angles, angles_rad, style='-', label=None, color=None):
    """Calculate and plot a single signal curve for an antenna location."""
    r = solve_distance(rssi_val)
    rssi_phi = np.array([rssi_angle(phi) for phi in angles])
    signal_loss = -rssi_phi
    rssi_a = rssi_val + signal_loss
    distances = np.array([solve_distance(rssi) for rssi in rssi_a])
    to_keep = distances > 0
    angles_rad_limited = angles_rad[to_keep]
    distances_limited = distances[to_keep]
    x = distances_limited * np.sin(angles_rad_limited)
    y = distances_limited * np.cos(angles_rad_limited)
    x_rot, y_rot = rotate_points(x, y, angle_deg)
    x_trans, y_trans = translate_points(x_rot, y_rot, ant_x, ant_y)
    plt.plot(x_trans, y_trans, style, label=label, color=color)
    return x_trans, y_trans

def make_polygon_points(x_upper, y_upper, x_lower, y_lower):
    """Helper to create polygon points for intersection/hatching."""
    return np.column_stack((
        np.concatenate([x_upper, x_lower[::-1]]),
        np.concatenate([y_upper, y_lower[::-1]])
    ))

def create_single_plot(rssi_received, ant_x, ant_y, beta, location_num, all_antennas, tag_x, tag_y, tag_id):
    """Create a plot for a single antenna location and tag."""
    fig = plt.figure(figsize=(12, 12))
    angles = np.linspace(-ALPHA, ALPHA, int(ALPHA * 2 + 1))
    angles_rad = np.radians(angles)
    rms_rssi = get_rms_rssi(rssi_received)
    upper_rssi = rssi_received + rms_rssi
    lower_rssi = rssi_received - rms_rssi
    x_upper, y_upper = plot_curve(upper_rssi, ant_x, ant_y, beta, angles, angles_rad, '-', f'Upper bound')
    x_lower, y_lower = plot_curve(lower_rssi, ant_x, ant_y, beta, angles, angles_rad, '-', f'Lower bound')
    x_nominal, y_nominal = plot_curve(rssi_received, ant_x, ant_y, beta, angles, angles_rad, '--', f'Nominal')
    color = plt.cm.rainbow(0) if len(all_antennas) == 0 else plt.cm.rainbow((location_num - 1) / len(all_antennas))
    plt.plot(ant_x, ant_y, 'o', color=color, label=f'Antenna {location_num}', markersize=8)
    plt.text(ant_x, ant_y, f'({ant_x:.2f}, {ant_y:.2f})', fontsize=8, ha='left', va='bottom')
    dx = VECTOR_LENGTH * np.sin(np.radians(beta))
    dy = VECTOR_LENGTH * np.cos(np.radians(beta))
    plt.arrow(ant_x, ant_y, dx, dy, head_width=0.04, head_length=0.08, fc='k', ec='k', width=0.015, length_includes_head=True)
    plt.plot(tag_x, tag_y, 'k+', label='Tag', markersize=12, markeredgewidth=2)
    plt.text(tag_x, tag_y, f'({tag_x:.2f}, {tag_y:.2f})', fontsize=8, ha='left', va='bottom')
    polygon_points = make_polygon_points(x_upper, y_upper, x_lower, y_lower)
    polygon = Polygon(polygon_points, facecolor=color, edgecolor='none', hatch='//////', alpha=0.2, label=f'Intersection Area')
    plt.gca().add_patch(polygon)
    plt.plot(0, 0, 'r+', label='Origin (0,0)', markersize=10)
    plt.text(0, 0, '(0.00, 0.00)', fontsize=8, ha='left', va='bottom')
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.minorticks_on()
    plt.grid(True, which='minor', linestyle=':', alpha=0.3)
    plt.xlim(*PLOT_LIMITS)
    plt.ylim(*PLOT_LIMITS)
    plt.xlabel('X Distance (meters)')
    plt.ylabel('Y Distance (meters)')
    plt.title(f'Tag {tag_id} - Location {location_num}: Signal Path Visualization\nRSSI = {rssi_received} dBm (±{rms_rssi} dBm RMS), Rotated by {beta}°\nAntenna at ({ant_x}, {ant_y})')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    return x_upper, y_upper, x_lower, y_lower, x_nominal, y_nominal

def create_combined_plot(all_data, tag_x, tag_y, tag_id):
    """Create a combined plot for all antenna locations for a tag."""
    plt.figure(figsize=(12, 12))
    colors = plt.cm.rainbow(np.linspace(0, 1, len(all_data)))
    for i, (rssi, ant_x, ant_y, beta, curves) in enumerate(all_data):
        color = colors[i]
        x_upper, y_upper, x_lower, y_lower, x_nominal, y_nominal = curves
        plt.plot(x_upper, y_upper, '-', color=color, label=f'Location {i+1} Upper')
        plt.plot(x_lower, y_lower, '-', color=color, label=f'Location {i+1} Lower')
        plt.plot(x_nominal, y_nominal, '--', color=color, label=f'Location {i+1} Nominal')
        plt.plot(ant_x, ant_y, 'o', color=color, label=f'Antenna {i+1}', markersize=8)
        plt.text(ant_x, ant_y, f'({ant_x:.2f}, {ant_y:.2f})', fontsize=8, ha='left', va='bottom')
        dx = VECTOR_LENGTH * np.sin(np.radians(beta))
        dy = VECTOR_LENGTH * np.cos(np.radians(beta))
        plt.arrow(ant_x, ant_y, dx, dy, head_width=0.04, head_length=0.08, fc='k', ec='k', width=0.015, length_includes_head=True)
        polygon_points = make_polygon_points(x_upper, y_upper, x_lower, y_lower)
        polygon = Polygon(polygon_points, facecolor=color, edgecolor='none', hatch='//////', alpha=0.2, label=f'Location {i+1} Intersection')
        plt.gca().add_patch(polygon)
    plt.plot(0, 0, 'r+', label='Origin (0,0)', markersize=10)
    plt.text(0, 0, '(0.00, 0.00)', fontsize=8, ha='left', va='bottom')
    plt.plot(tag_x, tag_y, 'k+', label='Tag', markersize=12, markeredgewidth=2)
    plt.text(tag_x, tag_y, f'({tag_x:.2f}, {tag_y:.2f})', fontsize=8, ha='left', va='bottom')
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.minorticks_on()
    plt.grid(True, which='minor', linestyle=':', alpha=0.3)
    plt.xlim(*PLOT_LIMITS)
    plt.ylim(*PLOT_LIMITS)
    plt.xticks(np.arange(PLOT_LIMITS[0], PLOT_LIMITS[1]+0.1, GRID_STEP))
    plt.yticks(np.arange(PLOT_LIMITS[0], PLOT_LIMITS[1]+0.1, GRID_STEP))
    plt.xlabel('X Distance (meters)')
    plt.ylabel('Y Distance (meters)')
    plt.title(f'Tag {tag_id} - Combined Signal Path Visualization for {len(all_data)} Locations\n(3m × 3m Area from Origin)')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()

def find_most_common_intersection(shapely_polygons):
    """Find the most common intersection area among polygons."""
    from itertools import combinations
    if len(shapely_polygons) < 2:
        return None
    intersection_regions = []
    for num_polygons in range(len(shapely_polygons), 1, -1):
        for poly_indices in combinations(range(len(shapely_polygons)), num_polygons):
            current_polygons = [shapely_polygons[i] for i in poly_indices]
            current_intersection = current_polygons[0]
            for poly in current_polygons[1:]:
                if current_intersection.is_empty:
                    break
                current_intersection = current_intersection.intersection(poly)
            if not current_intersection.is_empty:
                intersection_regions.append({
                    'area': current_intersection,
                    'count': num_polygons,
                    'indices': poly_indices
                })
        if intersection_regions:
            break
    if not intersection_regions:
        return None
    most_common = max(intersection_regions, key=lambda x: x['count'])
    core_area = most_common['area']
    max_intersection_area = 0
    best_area_index = most_common['indices'][0]
    for i in most_common['indices']:
        intersection = shapely_polygons[i].intersection(core_area)
        if not intersection.is_empty:
            area = intersection.area
            if area > max_intersection_area:
                max_intersection_area = area
                best_area_index = i
    current_intersection = shapely_polygons[best_area_index]
    used_indices = {best_area_index}
    remaining_polygons = [(i, poly) for i, poly in enumerate(shapely_polygons) if i not in used_indices]
    while remaining_polygons and not current_intersection.is_empty:
        best_next = None
        best_area = 0
        best_index = -1
        for i, poly in remaining_polygons:
            intersection = poly.intersection(current_intersection)
            if not intersection.is_empty and intersection.area > best_area:
                best_area = intersection.area
                best_next = poly
                best_index = i
        if best_next is None:
            break
        current_intersection = current_intersection.intersection(best_next)
        used_indices.add(best_index)
        remaining_polygons = [(i, poly) for i, poly in remaining_polygons if i not in used_indices]
    return current_intersection

def create_intersection_plot(all_data, tag_x, tag_y, tag_id):
    """Create a plot showing the most common intersection area for a tag."""
    plt.figure(figsize=(12, 12))
    colors = plt.cm.rainbow(np.linspace(0, 1, len(all_data)))
    polygons = []
    for i, (rssi, ant_x, ant_y, beta, curves) in enumerate(all_data):
        x_upper, y_upper, x_lower, y_lower, x_nominal, y_nominal = curves
        polygon_points = make_polygon_points(x_upper, y_upper, x_lower, y_lower)
        polygons.append(polygon_points)
        plt.plot(ant_x, ant_y, 'o', color=colors[i], label=f'Antenna {i+1}', markersize=8)
        plt.text(ant_x, ant_y, f'({ant_x:.2f}, {ant_y:.2f})', fontsize=8, ha='left', va='bottom')
        dx = VECTOR_LENGTH * np.sin(np.radians(beta))
        dy = VECTOR_LENGTH * np.cos(np.radians(beta))
        plt.arrow(ant_x, ant_y, dx, dy, head_width=0.04, head_length=0.08, fc='k', ec='k', width=0.015, length_includes_head=True)
    shapely_polygons = [ShapelyPolygon(poly) for poly in polygons]
    common_intersection = find_most_common_intersection(shapely_polygons)
    if common_intersection is not None and not common_intersection.is_empty:
        if common_intersection.geom_type == 'MultiPolygon':
            for poly in common_intersection.geoms:
                intersection_coords = np.array(poly.exterior.coords)
                polygon = Polygon(intersection_coords, facecolor='purple', edgecolor='none', hatch='//////', alpha=0.3)
                plt.gca().add_patch(polygon)
        else:
            intersection_coords = np.array(common_intersection.exterior.coords)
            polygon = Polygon(intersection_coords, facecolor='purple', edgecolor='none', hatch='//////', alpha=0.3, label='Most Common Intersection Area')
            plt.gca().add_patch(polygon)
    plt.plot(tag_x, tag_y, 'k+', label='Tag', markersize=12, markeredgewidth=2)
    plt.text(tag_x, tag_y, f'({tag_x:.2f}, {tag_y:.2f})', fontsize=8, ha='left', va='bottom')
    plt.plot(0, 0, 'r+', label='Origin (0,0)', markersize=10)
    plt.text(0, 0, '(0.00, 0.00)', fontsize=8, ha='left', va='bottom')
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.minorticks_on()
    plt.grid(True, which='minor', linestyle=':', alpha=0.3)
    plt.xlim(*PLOT_LIMITS)
    plt.ylim(*PLOT_LIMITS)
    plt.xticks(np.arange(PLOT_LIMITS[0], PLOT_LIMITS[1]+0.1, GRID_STEP))
    plt.yticks(np.arange(PLOT_LIMITS[0], PLOT_LIMITS[1]+0.1, GRID_STEP))
    plt.xlabel('X Distance (meters)')
    plt.ylabel('Y Distance (meters)')
    plt.title(f'Tag {tag_id} - Most Common Intersection of {len(all_data)} Locations\n(3m × 3m Area from Origin)')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()

def create_all_tags_plot(tags_data):
    """Create a plot showing all tag positions and intersection areas."""
    plt.figure(figsize=(15, 15))
    colors = plt.cm.rainbow(np.linspace(0, 1, len(tags_data)))
    for i, (tag_id, tag_info) in enumerate(tags_data.items()):
        color = colors[i]
        tag_x = tag_info['tag_x']
        tag_y = tag_info['tag_y']
        plt.plot(tag_x, tag_y, 'k+', markersize=12, markeredgewidth=2)
        plt.text(tag_x, tag_y, f'Tag {tag_id}\n({tag_x:.2f}, {tag_y:.2f})', fontsize=10, ha='right', va='bottom', color='black')
        if tag_info['centroids'] and tag_info.get('intersection_polygon'):
            centroid_x, centroid_y = tag_info['centroids']
            intersection = tag_info['intersection_polygon']
            if intersection.geom_type == 'MultiPolygon':
                for poly in intersection.geoms:
                    x, y = poly.exterior.xy
                    plt.fill(x, y, alpha=0.2, color=color)
                    plt.plot(x, y, '--', color=color, alpha=0.5)
            else:
                x, y = intersection.exterior.xy
                plt.fill(x, y, alpha=0.2, color=color)
                plt.plot(x, y, '--', color=color, alpha=0.5, label=f'Tag {tag_id} Area')
            plt.plot(centroid_x, centroid_y, 'o', color=color, markersize=12, markeredgewidth=2)
            plt.text(centroid_x, centroid_y, f'Est. {tag_id}\n({centroid_x:.2f}, {centroid_y:.2f})', fontsize=10, ha='left', va='bottom', color=color)
    plt.plot(0, 0, 'r+', label='Origin (0,0)', markersize=10)
    plt.text(0, 0, '(0.00, 0.00)', fontsize=8, ha='left', va='bottom')
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.minorticks_on()
    plt.grid(True, which='minor', linestyle=':', alpha=0.3)
    plt.xlim(*PLOT_LIMITS)
    plt.ylim(*PLOT_LIMITS)
    plt.xticks(np.arange(PLOT_LIMITS[0], PLOT_LIMITS[1]+0.1, GRID_STEP))
    plt.yticks(np.arange(PLOT_LIMITS[0], PLOT_LIMITS[1]+0.1, GRID_STEP))
    plt.xlabel('X Distance (meters)')
    plt.ylabel('Y Distance (meters)')
    plt.title('Tag Positions and Intersection Areas Map')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()

def process_tag_data(excel_file):
    """Process tag data from an Excel file and generate plots for each tag."""
    xl = pd.ExcelFile(excel_file)
    all_tags_data = {}
    for sheet_name in xl.sheet_names:
        if sheet_name == 'All Data':
            continue
        df = pd.read_excel(excel_file, sheet_name=sheet_name)
        unique_distances = df['Distance [m]'].unique()
        num_locations = len(unique_distances)
        if num_locations < 2:
            continue
        all_data = []
        all_antennas = []
        tag_x = df['Tag X [m]'].iloc[0]
        tag_y = df['Tag Y [m]'].iloc[0]
        for distance in unique_distances:
            distance_data = df[df['Distance [m]'] == distance]
            max_rssi_row = distance_data.loc[distance_data['RSSI'].idxmax()]
            rssi_received = distance_data['RSSI'].mean()
            beta = max_rssi_row['Antenna Rot Z [deg]']
            ant_x = max_rssi_row['Antenna X [m]']
            ant_y = max_rssi_row['Antenna Y [m]']
            curves = create_single_plot(rssi_received, ant_x, ant_y, beta, len(all_data)+1, all_antennas, tag_x, tag_y, sheet_name)
            all_data.append((rssi_received, ant_x, ant_y, beta, curves))
            all_antennas.append((rssi_received, ant_x, ant_y, beta, curves))
        create_combined_plot(all_data, tag_x, tag_y, sheet_name)
        create_intersection_plot(all_data, tag_x, tag_y, sheet_name)
        shapely_polygons = []
        angles = np.linspace(-ALPHA, ALPHA, int(ALPHA * 2 + 1))
        for _, _, _, _, curves in all_data:
            x_upper, y_upper, x_lower, y_lower, _, _ = curves
            polygon_points = make_polygon_points(x_upper, y_upper, x_lower, y_lower)
            shapely_polygons.append(ShapelyPolygon(polygon_points))
        common_intersection = find_most_common_intersection(shapely_polygons)
        centroids = None
        if common_intersection is not None and not common_intersection.is_empty:
            centroid = common_intersection.centroid
            centroids = (centroid.x, centroid.y)
        all_tags_data[sheet_name] = {
            'tag_x': tag_x,
            'tag_y': tag_y,
            'centroids': centroids,
            'intersection_polygon': common_intersection
        }
        plt.show()
    create_all_tags_plot(all_tags_data)
    plt.show()

if __name__ == "__main__":
    excel_files = glob.glob('rfid_data_110425_115143_Test1.xlsx')
    if not excel_files:
        print("No RFID data files found in the current directory!")
        exit(1)
    latest_file = max(excel_files, key=lambda x: os.path.getmtime(x))
    print(f"Using most recent data file: {latest_file}")
    try:
        process_tag_data(latest_file)
    except KeyboardInterrupt:
        print("\nProcessing stopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {str(e)}")


