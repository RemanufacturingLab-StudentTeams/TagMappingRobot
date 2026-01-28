import numpy as np
from scipy.interpolate import interp1d
from shapely.geometry import MultiPolygon, Polygon as ShapelyPolygon
from shapely.affinity import rotate
from shapely.geometry import Point, MultiPoint
from shapely.ops import unary_union

import db

# -------------------
# Constants
# -------------------
ALPHA = 90  # Angle in degrees

MIN_POLY_AREA = 1e-5 # ~0.1 cm²
POLY_SHAPE_RATIO = 8 # 
MIN_RSSI = -85
MAX_RSSI = -20
DEFAULT_ELLIPSE = [0,1,1] #rotation(deg), width(m), heigt(m)
MAX_VARIANCE_TRESHOLD = 6
OVERLAP_METHOD = 0

# -------------------
# Utility Functions
# -------------------
def validate_globals():
    assert 0 < ALPHA <= 180
    assert MIN_POLY_AREA > 0
    assert POLY_SHAPE_RATIO > 0 
    assert 0 > MAX_RSSI
    assert MIN_RSSI < MAX_RSSI
    assert len(DEFAULT_ELLIPSE) == 3
    assert MAX_VARIANCE_TRESHOLD >= 1


def init_globals(cfg):
    global MIN_POLY_AREA, POLY_SHAPE_RATIO, MIN_RSSI, MAX_RSSI, DEFAULT_ELLIPSE, MAX_VARIANCE_TRESHOLD, OVERLAP_METHOD
    MIN_POLY_AREA = cfg["MIN_POLY_AREA"]
    POLY_SHAPE_RATIO = cfg["POLY_SHAPE_RATIO"] 
    MIN_RSSI = cfg["MIN_RSSI"]
    MAX_RSSI = cfg["MAX_RSSI"]
    DEFAULT_ELLIPSE = cfg["DEFAULT_ELLIPSE"]
    MAX_VARIANCE_TRESHOLD = cfg["MAX_VARIANCE_TRESHOLD"]
    OVERLAP_METHOD = cfg["OVERLAP_METHOD"]
    validate_globals()

##Get the average of the rms   
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

def rssi_angle(phi):
    """Calculate RSSI based on angle."""
    phi = np.asarray(phi)
    return 0.038186 * phi - 0.003704 * phi**2

def enclosing_ellipse(geometry, scale=1.1):
    """
    Fast, lightweight ellipse that fully encloses a Shapely Polygon or MultiPolygon.
    - scale: enlarge slightly (>1 ensures it fully contains the shape)
    Returns: (rotation_deg, a, b)
    """

    # --- Combine all geometries if needed ---
    if not isinstance(geometry, (ShapelyPolygon, MultiPolygon)):
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

def rotate_points(x, y, angle_deg):
    """Rotate points (x, y) by angle_deg around the origin."""
    angle_rad = np.radians(-angle_deg - ALPHA)
    x_rot =  x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_rot = -x * np.sin(angle_rad) - y * np.cos(angle_rad)
    return x_rot, y_rot

def translate_points(x, y, ant_x, ant_y):
    """Translate points (x, y) to antenna location (ant_x, ant_y)."""
    return x + ant_x, y + ant_y

def plot_curve(rssi_val, ant_x, ant_y, angle_deg, angles, angles_rad):
    """Calculate and plot a single signal curve for an antenna location."""
    rssi_phi = rssi_angle(angles)
    signal_loss = -rssi_phi
    rssi_a = rssi_val + signal_loss
    distances = inverse_rssi(rssi_a)
    
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

def create_single_plot(rssi_received, ant_x, ant_y, beta):
    """Create a plot for a single antenna location and tag."""
    angles = np.linspace(-ALPHA, ALPHA, int(ALPHA * 2 + 1))
    angles_rad = np.radians(angles)
    rms_rssi = get_rms_rssi(rssi_received)
    upper_rssi = rssi_received + rms_rssi
    lower_rssi = rssi_received - rms_rssi
    x_upper, y_upper = plot_curve(upper_rssi, ant_x, ant_y, beta, angles, angles_rad)
    x_lower, y_lower = plot_curve(lower_rssi, ant_x, ant_y, beta, angles, angles_rad)
    return x_upper, y_upper, x_lower, y_lower

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


def process_measurements(tag_id, measurements):
    """
    measurements: list of dicts for a single tag
    return dict with keys ID,X,Y,r,w,h or None
    """
    if not measurements:
        return None

    # group by distance value (float) using a small tolerance
    groups = {}
    for m in measurements:
        d = float(m.get('Distance [m]', 0.0))
        # group by rounded distance to avoid float grouping issues
        key = round(d, 3)
        groups.setdefault(key, []).append(m)
        
    if len(groups) < 2:
        return None
    
    shapely_polygons = []
    # for each distance group compute rssi mean and a representative antenna row
    for dist_key in sorted(groups.keys()):
        grp = groups[dist_key]        
        # Reject inconsistent RSSI
        rssi_vals = [x['RSSI'] for x in grp]
        if np.std(rssi_vals) > MAX_VARIANCE_TRESHOLD:  # tuned threshold
            print (f"variance inside group {dist_key} for tag (tag_id) to high: {rssi_vals} \n skipping to next group ")
            continue
        
        # choose row with max RSSI as representative
        try:
            max_row = max(grp, key=lambda x: x.get('RSSI', -999))
        except Exception as e:
            print(f"error calculting max_row for tag {tag_id}: {e}\n Skipping to next group")
            continue
        mean_rssi = sum([x.get('RSSI', 0.0) for x in grp]) / len(grp)
        ant_x = float(max_row['Antenna X [m]'])
        ant_y = float(max_row['Antenna Y [m]'])
        beta = float(max_row['Antenna Rot Z [deg]'])
        # Ignore absurd RSSI values
        if mean_rssi < MIN_RSSI or mean_rssi > MAX_RSSI:
            print(f"Rejcted RSSI {mean_rssi} to high or to low, RSSI should between {MIN_RSSI} and {MAX_RSSI}\n Skipping to next group")
            continue
        try:
            curves = create_single_plot(mean_rssi, ant_x, ant_y, beta)
        except Exception as e:
            print (f"error creating single plot for tag {tag_id}: {e}\n Skipping to next group")
            continue

        x_upper, y_upper, x_lower, y_lower = curves
        pts = make_polygon_points(x_upper, y_upper, x_lower, y_lower)
        poly = ShapelyPolygon(pts)
        # Reject polygons that are too thin or too tiny
        if poly.area < MIN_POLY_AREA:   # ~0.1 cm²
            print (f"Rejected poly:{poly} for tag {tag_id} area to small\n Skipping to next group")
            continue
        
        # Reject polygons with weird geometry
        if poly.length / (2 * np.sqrt(np.pi * poly.area)) > POLY_SHAPE_RATIO:
            # Perimeter too large for the area: suspicious shape
            print (f"Rejected poly:{poly} for tag {tag_id} weird shape\n Skipping to next group")
            continue
        shapely_polygons.append(poly)
    try:
        prev = db.get_polygon(tag_id)
    except Exception as e:
        prev = None
        print (f"error getting previus poly:{e}")

    if (OVERLAP_METHOD == 0):
        if prev:
            shapely_polygons.append(prev)
        common = find_core_intersection(shapely_polygons)
        if common is None or common.is_empty:
            print (f"No common intersection for tag {tag_id}")
            return(None)
            
    
        db.save_polygon(tag_id, common)
    
        if common.geom_type == "MultiPolygon":
            common = min(common.geoms, key=lambda p: p.area)
    
        centroid = common.centroid
        tagX = centroid.x
        tagY = centroid.y

    if (OVERLAP_METHOD == 1):
        if not prev:
            prev = None
        estimate = estimate_location_with_uncertainty(
            shapely_polygons,
            prev_polygon=prev,
            grid_step=0.05,
            vote_fraction=0.7
        )
        
        if estimate is None:
            return None
        
        tagX, tagY , common= estimate
        db.save_polygon(tag_id, common)
    
    try:
        rotation, width, height = enclosing_ellipse(common)
    except Exception as e:
        print (f"error calculting ellipse for tag {tag_id}: {e}\n using default ellipse")
        rotation, width, height = DEFAULT_ELLIPSE


    return {
        "ID": tag_id,
        "X": round(tagX, 2),
        "Y": round(tagY, 2),
        "r": round(rotation, 2),
        "w": round(width, 2),
        "h": round(height, 2)
    }
