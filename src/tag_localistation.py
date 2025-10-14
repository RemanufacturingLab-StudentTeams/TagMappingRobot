import numpy as np
from scipy.optimize import fsolve
from shapely.geometry import MultiPolygon, Polygon as ShapelyPolygon
from shapely.affinity import rotate
import pandas as pd

# -------------------
# Constants
# -------------------
ALPHA = 90  # Angle in degrees
VECTOR_LENGTH = 0.2  # Arrow length for antenna orientation
PLOT_LIMITS = (-0.5, 4)
GRID_STEP = 0.5

distance_cache = {}

# -------------------
# Utility Functions
# -------------------
##Get the average of the rms 
def equal_area_circle(polygon):
    """"Calculate circel radius with the same area"""
    area = polygon.area
    return (np.sqrt(area / np.pi))

def min_max_array(polygon):
    """"create array with xmin, ymin, xmax, ymax"""
    x, y = (polygon.exterior.xy)
    xmin, ymin, xmax, ymax = polygon.bounds
    xmin_index = x.index(xmin)
    ymin_index = y.index(ymin)
    xmax_index = x.index(xmax)
    ymax_index = y.index(ymax)
    return [(xmin, y[xmin_index]),(x[ymin_index], ymin), (xmax, y[xmax_index]),(x[ymax_index], ymax)]
   
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

def max_distence_circle(polygon):
    coords = min_max_array(polygon)
    delta = []
    for n in coords:
        distence = np.sqrt((n[0]-polygon.centroid.x)**2 +(n[1]-polygon.centroid.y)**2)
        delta.append(distence)
    return max(delta)

def enclosing_ellipse(geometry, scale=1.1):
    """
    Fast, lightweight ellipse that fully encloses a Shapely Polygon or MultiPolygon.
    - scale: enlarge slightly (>1 ensures it fully contains the shape)
    Returns: (rotation_deg, a, b)
    """

    # --- Combine all geometries if needed ---
    if isinstance(geometry, MultiPolygon):
        geometry = geometry.unary_union
    elif not isinstance(geometry, ShapelyPolygon):
        raise TypeError("Input must be a Shapely Polygon or MultiPolygon")

    # --- Find approximate orientation ---
    min_rect = geometry.minimum_rotated_rectangle
    rect_coords = np.array(min_rect.exterior.coords)
    dx, dy = rect_coords[1] - rect_coords[0]
    rotation_deg = np.degrees(np.arctan2(dy, dx))

    # --- Get bounding box of rotated geometry ---
    rotated_geom = rotate(geometry, -rotation_deg, origin='center', use_radians=False)
    xmin, ymin, xmax, ymax = rotated_geom.bounds

    # --- Compute ellipse parameters ---
    a = (xmax - xmin)  * scale
    b = (ymax - ymin)  * scale

    return rotation_deg, a, b

def solve_distance(rssi, init=1.5):
 
    key = round(rssi, 2)
 
    if key in distance_cache:
           return distance_cache[key]
    """Solve for distance given RSSI using fsolve."""
    def equation(d):
        return rssi - rssi_distance(d)
    def deriv(d):
        return -rssi_distance_deriv(d)
    sol = fsolve(equation, init, fprime=deriv)[0]
    distance_cache[key] = sol
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
    #r = solve_distance(rssi_val) ##Referece radius around antenna
    rssi_phi = np.array([rssi_angle(phi) for phi in angles])
    signal_loss = -rssi_phi
    ##A vector of RSSI values at different angles
    rssi_a = rssi_val + signal_loss
    distances = np.array([solve_distance(rssi) for rssi in rssi_a])
    to_keep = distances > 0
    angles_rad_limited = angles_rad[to_keep]
    distances_limited = distances[to_keep]
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
    #fig = plt.figure(figsize=(12, 12))
    angles = np.linspace(-ALPHA, ALPHA, int(ALPHA * 2 + 1))
    angles_rad = np.radians(angles)
    rms_rssi = get_rms_rssi(rssi_received)
    upper_rssi = rssi_received + rms_rssi
    lower_rssi = rssi_received - rms_rssi

    x_upper, y_upper = plot_curve(upper_rssi, ant_x, ant_y, beta, angles, angles_rad, '-', f'Upper bound')
    x_lower, y_lower = plot_curve(lower_rssi, ant_x, ant_y, beta, angles, angles_rad, '-', f'Lower bound')
    
    return x_upper, y_upper, x_lower, y_lower

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


def process_tag_data(excel_file, file_path):
    """Process tag data from an Excel file and generate plots for each tag."""
    ##It makes a panda file from the excel file
    xl = pd.ExcelFile(excel_file)
    all_tags_data = []
    for sheet_name in xl.sheet_names:
        if sheet_name == 'All Data':
            continue   ##The continue means it wont process the tag data in all data sheet
        df = pd.read_excel(excel_file, sheet_name=sheet_name)
        unique_distances = df['Distance [m]'].unique()
        num_locations = len(unique_distances)
        if num_locations < 2:
            continue #go to next tag info 
        all_data = []
        all_antennas = []
        
        #It gets the actual location of the tag, input from user, 
        tag_x = df['Tag X [m]'].iloc[0]
        tag_y = df['Tag Y [m]'].iloc[0]
        for distance in unique_distances:
            distance_data = df[df['Distance [m]'] == distance]
            ## Holds all the info for the strongest signal measured at that distance
            max_rssi_row = distance_data.loc[distance_data['RSSI'].idxmax()]
            rssi_received = distance_data['RSSI'].mean()
            beta = max_rssi_row['Antenna Rot Z [deg]']
            ant_x = max_rssi_row['Antenna X [m]']
            ant_y = max_rssi_row['Antenna Y [m]']
            ##First plot, curves is the coverage contour of that antenna for the signal strengthw
            curves = create_single_plot(rssi_received, ant_x, ant_y, beta, len(all_data)+1, all_antennas, tag_x, tag_y, sheet_name)
            all_data.append((rssi_received, ant_x, ant_y, beta, curves))
            all_antennas.append((rssi_received, ant_x, ant_y, beta, curves))
        shapely_polygons = []
        for _, _, _, _, curves in all_data:
            x_upper, y_upper, x_lower, y_lower = curves
            polygon_points = make_polygon_points(x_upper, y_upper, x_lower, y_lower)
            shapely_polygons.append(ShapelyPolygon(polygon_points))
        common_intersection = find_most_common_intersection(shapely_polygons)
        if common_intersection is not None and not common_intersection.is_empty:
            if (common_intersection.geom_type == "MultiPolygon"):
                continue
            centroid = common_intersection.centroid
            
            
        #equal_area_radius = equal_area_circle(common_intersection)
        #max_distence_radius = max_distence_circle(common_intersection)
        #bounds_polygon =    min_max_array(common_intersection)
        radius, width, height = enclosing_ellipse(common_intersection)
        
        
        tag_data = {
            "ID": sheet_name,
            "X" : round(centroid.x,2),
            "Y" : round(centroid.y,2),
            "r" : round(radius,2),
            "w" : round(width,2),
            "h" : round(height,2)
            # 'max distence radius' : max_distence_radius,
            # 'bounds polygon' : bounds_polygon
        }     
        all_tags_data.append(tag_data)
    return(all_tags_data)
        


