#!/usr/bin/env python3

# convert_map_for_dash.py
# Converts a ROS/SLAM map (.pgm + .yaml) into a Dash-ready PNG
# and prints the correct Plotly/Dash add_layout_image parameters.

import os
import yaml
from PIL import Image

def convert_pgm_and_get_bounds(map_yaml_path: str, output_dir: str = "assets"):
    # --- read yaml ---
    with open(map_yaml_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    image_rel = cfg["image"]                      # e.g. "map.pgm"
    resolution = float(cfg["resolution"])         # meters per pixel
    origin = cfg["origin"]                        # [origin_x, origin_y, yaw]
    origin_x, origin_y = float(origin[0]), float(origin[1])

    yaml_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    image_path = os.path.join(yaml_dir, image_rel)

    # --- load pgm and save as png into Dash assets/ ---
    os.makedirs(output_dir, exist_ok=True)
    png_name = os.path.splitext(os.path.basename(image_rel))[0] + ".png"
    png_path = os.path.join(output_dir, png_name)

    img = Image.open(image_path)                  # reads .pgm fine
    width_px, height_px = img.size
    img.save(png_path, format="PNG")

    # --- compute bounds in map/world coordinates ---
    # In ROS map_server convention:
    # origin = world coords of the map's bottom-left pixel.
    xmin = origin_x
    ymin = origin_y
    xmax = origin_x + width_px * resolution
    ymax = origin_y + height_px * resolution

    # For plotly layout_image: use top-left corner (x=xmin, y=ymax)
    # and sizex/sizey in axis units (meters)
    plotly_params = dict(
        source=f"/assets/{png_name}",
        xref="x",
        yref="y",
        x=xmin,
        y=ymax,
        sizex=(xmax - xmin),
        sizey=(ymax - ymin),
        sizing="stretch",
        layer="below",
        opacity=0.8,
    )

    return png_path, (xmin, xmax, ymin, ymax), plotly_params


if __name__ == "__main__":
    # CHANGE THIS to your yaml path
    MAP_YAML = "lokaal.yaml"  # e.g. "C:/path/to/map.yaml"

    png_path, bounds, params = convert_pgm_and_get_bounds(MAP_YAML, output_dir="assets")

    xmin, xmax, ymin, ymax = bounds
    print("✅ Saved PNG for Dash:", png_path)
    print("\nBounds (meters):")
    print("xmin =", xmin, "xmax =", xmax, "ymin =", ymin, "ymax =", ymax)

    print("\nUse this in your Dash figure:")
    print("fig.add_layout_image(", params, ")\n")

    print("Optional: lock axes to map bounds:")
    print(f"fig.update_xaxes(range=[{xmin}, {xmax}])")
    print(f"fig.update_yaxes(range=[{ymin}, {ymax}])")
    print('fig.update_layout(xaxis_scaleanchor="y")')
