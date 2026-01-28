import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString
from scipy.interpolate import interp1d
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as ShapelyPolygon
import pandas as pd
import os
import glob
import time

# -------------------
# Constants
# -------------------
ALPHA = 90  # Angle in degrees
VECTOR_LENGTH = 0.2  # Arrow length for antenna orientation
PLOT_LIMITS = (-4, 8)
GRID_STEP = 0.5


MIN_POLY_AREA = 1e-5 # ~0.1 cm²
POLY_SHAPE_RATIO = 8 # 
MIN_RSSI = -85
MAX_RSSI = -20


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
    phi = np.asarray(phi)
    return 0.038186 * phi - 0.003704 * phi**2

# Precompute inverse model
d_table = np.linspace(0, 10, 10001)   # 1 mm steps
rssi_table = rssi_distance(d_table)

# Ensure monotonic decreasing RSSI
# If needed, sort by RSSI
order = np.argsort(rssi_table)
rssi_sorted = rssi_table[order]
d_sorted = d_table[order]

# Build interpolation function: RSSI -> distance
inverse_rssi = interp1d(
    rssi_sorted,
    d_sorted,
    bounds_error=False,
    fill_value="extrapolate",
    assume_sorted=True
)

def solve_distance(rssi, init=1.5):
    return float(inverse_rssi(rssi))

def simplify_polygon(x, y, tolerance=0.01):
    line = LineString(np.column_stack([x, y]))
    simplified = line.simplify(tolerance)
    return np.array(simplified.xy)

def rotate_points(x, y, angle_deg):
    """Rotate points (x, y) by angle_deg around the origin."""
    angle_rad = np.radians(-angle_deg - 90)
    x_rot = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_rot = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_rot, y_rot

def translate_points(x, y, ant_x, ant_y):
    """Translate points (x, y) to antenna location (ant_x, ant_y)."""
    return x + ant_x, y + ant_y

def plot_curve(rssi_val, ant_x, ant_y, angle_deg, angles, angles_rad, style='-', label=None, color=None):
    """Calculate and plot a single signal curve for an antenna location."""
    rssi_phi = rssi_angle(angles)
    signal_loss = -rssi_phi
    rssi_a = rssi_val + signal_loss
    distances = inverse_rssi(rssi_a)  # vectorized RSSI→distance
    
    mask = (distances > 0) & np.isfinite(distances)
    if not np.any(mask):
        return np.array([]), np.array([])
    
    angles_rad_limited = angles_rad[mask]
    distances_limited = distances[mask]
    x = distances_limited * np.sin(angles_rad_limited)
    y = distances_limited * np.cos(angles_rad_limited)
    x_rot, y_rot = rotate_points(x, y, angle_deg)
    x_trans, y_trans = translate_points(x_rot, y_rot, ant_x, ant_y)
    return x_trans, y_trans

def make_polygon_points(x_upper, y_upper, x_lower, y_lower):
    """Helper to create polygon points for intersection/hatching."""
    return np.column_stack((
        np.concatenate([x_upper, x_lower[::-1]]),
        np.concatenate([y_upper, y_lower[::-1]])
    ))


def create_single_plot(rssi_received, ant_x, ant_y, beta, location_num, all_antennas, tag_x, tag_y, tag_id):
    """Create a plot for a single antenna location and tag."""
    if not np.isfinite(rssi_received):
        raise Exception ("no RSSI value")
    
    # Ignore absurd RSSI values
    if rssi_received < MIN_RSSI or rssi_received > MAX_RSSI:
        raise Exception(f"Rejcted RSSI {rssi_received} to high or to low, RSSI should between {MIN_RSSI} and {MAX_RSSI}")
    fig = plt.figure(figsize=(12, 12))
    angles = np.linspace(-ALPHA, ALPHA, int(ALPHA * 2 + 1))
    angles_rad = np.radians(angles)
    rms_rssi = get_rms_rssi(rssi_received)
    upper_rssi = rssi_received + rms_rssi
    lower_rssi = rssi_received - rms_rssi
    x_upper, y_upper = plot_curve(upper_rssi, ant_x, ant_y, beta, angles, angles_rad, '-', f'Upper bound')
    #x_upper, y_upper = simplify_polygon(x_upper, y_upper, tolerance=0.01)
    x_lower, y_lower = plot_curve(lower_rssi, ant_x, ant_y, beta, angles, angles_rad, '-', f'Lower bound')
    #x_lower, y_lower = simplify_polygon(x_lower, y_lower, tolerance=0.01)
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
    common_intersection = find_core_intersection(shapely_polygons)
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
    
def find_core_intersection(polygons, min_area=0.0001):
    """
    Find the core area where most polygons overlap.
    polygons: list of ShapelyPolygon
    min_area: minimal polygon area to consider
    """
    if not polygons:
        return None
    
    # Start with polygon with largest area as core
    polygons_sorted = sorted(polygons, key=lambda p: p.area, reverse=True)
    core = polygons_sorted[0]
    used = {0}

    # Iteratively intersect with other polygons
    remaining = [(i, p) for i, p in enumerate(polygons_sorted[1:], start=1)]
    for i, p in remaining:
        intersection = core.intersection(p)
        if not intersection.is_empty and intersection.area >= min_area:
            core = intersection
            used.add(i)

    if core.is_empty:
        return None
    return core

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
    
from shapely.geometry import Point, MultiPoint
from shapely.ops import unary_union
import numpy as np

def estimate_location_with_uncertainty(
    polygons,
    prev_polygon=None,
    grid_step=0.05,      # 5 cm
    vote_fraction=0.8    # uncertainty contour
):
    """
    Estimate location and uncertainty region via overlap voting.
    Returns (x, y, uncertainty_polygon) or None.
    """

    if not polygons:
        return None

    union = unary_union(polygons)
    if union.is_empty:
        return None

    xmin, ymin, xmax, ymax = union.bounds

    xs = np.arange(xmin, xmax, grid_step)
    ys = np.arange(ymin, ymax, grid_step)

    vote_map = []
    max_score = 0.0

    for x in xs:
        for y in ys:
            p = Point(x, y)
            score = 0.0

            for poly in polygons:
                if poly.contains(p):
                    score += 1.0

            if prev_polygon is not None and prev_polygon.contains(p):
                score += 0.5   # soft bias

            vote_map.append((x, y, score))
            max_score = max(max_score, score)

    if max_score <= 0:
        return None

    # --- points near the maximum define uncertainty ---
    good_points = [
        (x, y) for x, y, s in vote_map
        if s >= vote_fraction * max_score
    ]

    if not good_points:
        return None

    xs, ys = zip(*good_points)
    X = float(np.mean(xs))
    Y = float(np.mean(ys))

    # --- uncertainty polygon ---
    uncertainty_poly = MultiPoint(good_points).convex_hull

    # Clean up very thin shapes
    if not uncertainty_poly.is_valid:
        uncertainty_poly = uncertainty_poly.buffer(0)

    return X, Y, uncertainty_poly



def process_tag_data(excel_file):
    """Process tag data from an Excel file and generate plots for each tag."""
    xl = pd.ExcelFile(excel_file)
    start = time.time()
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
            try:
                max_rssi_row = distance_data.loc[distance_data['RSSI'].idxmax()]
            except Exception as e:
                print(f"error calculting max_rssi_row for tag {sheet_name}: {e}\n Skiping to next")
                continue
            rssi_received = distance_data['RSSI'].mean()
            beta = max_rssi_row['Antenna Rot Z [deg]']
            beta = beta 
            ant_x = max_rssi_row['Antenna X [m]']
            ant_y = max_rssi_row['Antenna Y [m]']
            curve_start = time.time()
            try:
                curves = create_single_plot(rssi_received, ant_x, ant_y, beta, len(all_data)+1, all_antennas, tag_x, tag_y, sheet_name)
            except Exception as e:
                print (f"error creating single plot for tag {sheet_name}: {e}\n Skipping to next")
                continue
            curve_stop = time.time()
            #print ("cyrve time", curve_stop-curve_start)
            all_data.append((rssi_received, ant_x, ant_y, beta, curves))
            all_antennas.append((rssi_received, ant_x, ant_y, beta, curves))
        create_combined_plot(all_data, tag_x, tag_y, sheet_name)
        create_intersection_plot(all_data, tag_x, tag_y, sheet_name)
        shapely_polygons = []
        for _, _, _, _, curves in all_data:
            x_upper, y_upper, x_lower, y_lower, _, _ = curves
            polygon_points = make_polygon_points(x_upper, y_upper, x_lower, y_lower)
            poly = ShapelyPolygon(polygon_points)
            
            if not poly.is_valid:
                poly = poly.buffer(0)
            
            # Reject polygons that are too thin or too tiny
            if poly.area < MIN_POLY_AREA:   # ~0.1 cm²
                print (f"Rejected poly:{poly} area to small")
                continue
            
            # Reject polygons with weird geometry
            if poly.length / (2 * np.sqrt(np.pi * poly.area)) > POLY_SHAPE_RATIO:
                # Perimeter too large for the area: suspicious shape
                print (f"Rejected poly:{poly} weird shape")
                continue
            shapely_polygons.append(poly)
        OVERLAP_METHODE = 1
        if (OVERLAP_METHODE == 0):
            common = find_core_intersection(shapely_polygons)
            if common is None or common.is_empty:
                return(None)
                
        
        
            if common.geom_type == "MultiPolygon":
                common = min(common.geoms, key=lambda p: p.area)
            if common is not None and not common.is_empty:
                centroid = common.centroid
                centroids = (centroid.x, centroid.y)
        
        if (OVERLAP_METHODE == 1):
            estimate = estimate_location_with_uncertainty(
                shapely_polygons,
                grid_step=0.05,
                vote_fraction=0.7
            )
                        
            if estimate is None:
                return None
            
            X, Y, common  = estimate
            centroids = X,Y
            # build a small uncertainty polygon for persistence

        

        print (centroids)
        all_tags_data[sheet_name] = {
            'tag_x': tag_x,
            'tag_y': tag_y,
            'centroids': centroids,
            'intersection_polygon': common
        }
        plt.show()
    create_all_tags_plot(all_tags_data)
    #print (all_tags_data)
    plt.show()
    end = time.time()
    print ("full time =", end - start,"sec")

if __name__ == "__main__":
    excel_files = glob.glob('rfid_data_280126_104613.xlsx')
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


