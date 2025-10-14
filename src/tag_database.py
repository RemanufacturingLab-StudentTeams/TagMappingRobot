# tag_database.py
import sqlite3
from datetime import datetime
from shapely import wkt
from shapely.geometry import Polygon, MultiPolygon

DB_FILE = "tag_data.db"

# -----------------------------------------------------------
# Initialization
# -----------------------------------------------------------

def init_db():
    """Create the tables if they don't exist."""
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()

    c.execute("""
        CREATE TABLE IF NOT EXISTS measurements (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            tag_id TEXT,
            rssi REAL,
            x REAL,
            y REAL,
            rotation REAL,
            timestamp TEXT
        )
    """)

    c.execute("""
        CREATE TABLE IF NOT EXISTS polygons (
            tag_id TEXT PRIMARY KEY,
            wkt TEXT,
            updated_at TEXT
        )
    """)

    conn.commit()
    conn.close()

# -----------------------------------------------------------
# Measurement Functions
# -----------------------------------------------------------

def add_measurement(tag_id, rssi, x, y, rotation):
    """Add a single RSSI measurement."""
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute(
        "INSERT INTO measurements (tag_id, rssi, x, y, rotation, timestamp) VALUES (?, ?, ?, ?, ?, ?)",
        (tag_id, rssi, x, y, rotation, datetime.now().isoformat())
    )
    conn.commit()
    conn.close()


def add_measurements_bulk(measurements):
    """Add multiple RSSI measurements at once."""
    if not measurements:
        return

    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    timestamp = datetime.now().isoformat()
    data = [(m["tag_id"], m["rssi"], m["x"], m["y"], m["rotation"], timestamp) for m in measurements]
    c.executemany(
        "INSERT INTO measurements (tag_id, rssi, x, y, rotation, timestamp) VALUES (?, ?, ?, ?, ?, ?)",
        data
    )
    conn.commit()
    conn.close()


def get_measurements(tag_id, limit=None):
    """Get all (or last N) RSSI measurements for a tag."""
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    if limit:
        c.execute("""
            SELECT rssi, x, y, rotation, timestamp
            FROM measurements
            WHERE tag_id=?
            ORDER BY id DESC
            LIMIT ?
        """, (tag_id, limit))
    else:
        c.execute("""
            SELECT rssi, x, y, rotation, timestamp
            FROM measurements
            WHERE tag_id=?
            ORDER BY id
        """, (tag_id,))
    rows = c.fetchall()
    conn.close()
    return [
        {"rssi": rssi, "x": x, "y": y, "rotation": rotation, "timestamp": ts}
        for rssi, x, y, rotation, ts in rows
    ]


def delete_measurements(tag_id):
    """Delete all RSSI measurements for a tag."""
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("DELETE FROM measurements WHERE tag_id=?", (tag_id,))
    conn.commit()
    conn.close()

# -----------------------------------------------------------
# Polygon Functions
# -----------------------------------------------------------

def save_polygon(tag_id, polygon):
    """
    Save a Polygon or MultiPolygon for a tag.
    Automatically deletes previous measurements for that tag.
    """
    if not isinstance(polygon, (Polygon, MultiPolygon)):
        raise TypeError("polygon must be a Shapely Polygon or MultiPolygon")

    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()

    wkt_data = polygon.wkt
    updated_at = datetime.now().isoformat()

    c.execute("""
        INSERT INTO polygons (tag_id, wkt, updated_at)
        VALUES (?, ?, ?)
        ON CONFLICT(tag_id) DO UPDATE SET
            wkt=excluded.wkt,
            updated_at=excluded.updated_at
    """, (tag_id, wkt_data, updated_at))

    # Delete previous measurements
    c.execute("DELETE FROM measurements WHERE tag_id=?", (tag_id,))

    conn.commit()
    conn.close()


def get_polygon(tag_id):
    """Return stored polygon for tag_id, or None."""
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("SELECT wkt FROM polygons WHERE tag_id=?", (tag_id,))
    row = c.fetchone()
    conn.close()

    return wkt.loads(row[0]) if row else None


def get_all_polygons():
    """Return all stored polygons as dict {tag_id: shapely.geometry}"""
    conn = sqlite3.connect(DB_FILE)
    c = conn.cursor()
    c.execute("SELECT tag_id, wkt FROM polygons")
    rows = c.fetchall()
    conn.close()

    return {tag_id: wkt.loads(wkt_str) for tag_id, wkt_str in rows}
