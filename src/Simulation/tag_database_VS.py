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
    """Create the polygons tables if it doesn't exist."""
    with sqlite3.connect(DB_FILE) as conn:
        c = conn.cursor()
        c.execute("""
            CREATE TABLE IF NOT EXISTS polygons (
                tag_id TEXT PRIMARY KEY,
                wkt TEXT,
                updated_at TEXT
            )
        """)
        conn.commit()

# -----------------------------------------------------------
# Polygon Functions
# -----------------------------------------------------------

def save_polygon(tag_id: str, polygon: Polygon | MultiPolygon) -> None:
    """
    Save a Polygon or MultiPolygon for a tag.
    Creates a new row if it does not exist, or replaces the existing polygon if it does.
    """
    if not isinstance(polygon, (Polygon, MultiPolygon)):
        raise TypeError("polygon must be a Shapely Polygon or MultiPolygon")

    wkt_data = polygon.wkt
    updated_at = datetime.now().isoformat()

    with sqlite3.connect(DB_FILE) as conn:
        c = conn.cursor()
        c.execute("""
            INSERT INTO polygons (tag_id, wkt, updated_at)
            VALUES (?, ?, ?)
            ON CONFLICT(tag_id) DO UPDATE SET
                wkt=excluded.wkt,
                updated_at=excluded.updated_at
        """, (tag_id, wkt_data, updated_at))
        conn.commit()



def get_polygon(tag_id):
    """Return stored polygon for tag_id, or None."""
    with sqlite3.connect(DB_FILE) as conn:
        c = conn.cursor()
        c.execute("SELECT wkt FROM polygons WHERE tag_id=?", (tag_id,))
        row = c.fetchone()
    if not row:
        return None
    try:
        return wkt.loads(row[0]) 
    except Exception as e:
        raise ValueError(f"Invalid WKT data for tag_id {tag_id}: {e}")

def delete_polygon(tag_id):
    """"Delete stored polygon for tag_id"""
    with sqlite3.connect(DB_FILE) as conn:
        conn.execute("DELETE FROM polygons WHERE tag_id=?", (tag_id,))
        conn.commit()
