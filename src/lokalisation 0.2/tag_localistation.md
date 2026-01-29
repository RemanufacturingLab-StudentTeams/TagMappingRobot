# tag_localisgtation.py
### estimates tag location based on rssi, x, y and rotation
### Features
- filters data
- calculate posible loaction area
- overlap area's
- calculate ellipse around polygon to simplefy uncertainty

### Functions

def validate_globals():
- filter varibles to prevent unrealistic values

def init_globals(cfg):
- set all global varibles

def enclosing_ellipse(geometry, scale=1.1):
- generates a rotated ellips that tries to encase the polygon most effectivly

def rotate_points(x, y, angle_deg):
- rotates and mirrors points to match internal axis system with the external

def find_core_intersection(polygons, min_area=0.0001):
- hard overlaps polygons

def estimate_location_with_uncertainty(polygons,
    prev_polygon=None,
    grid_step=0.05,      # 5 cm
    vote_fraction=0.8    # uncertainty contour
):
- soft overlaps the polygons to get 1 final polygon 

def process_measurements(tag_id, measurements):
- fully process all measurements for one tag filters data to prevent unrealistic values