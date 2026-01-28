import sqlite3
import time
from shapely import wkt
from shapely.geometry import Polygon, MultiPolygon

DB_FILE = "tag_data_lok.db"

def init_db(File_name = "tag_data_lok.db" ):
    """Creats data base file if it does not exist"""
    global DB_FILE
    DB_FILE = File_name
    with sqlite3.connect(DB_FILE) as conn:
        c = conn.cursor()
        c.execute("""
            CREATE TABLE IF NOT EXISTS polygons (
                tag_id TEXT PRIMARY KEY,
                wkt TEXT,
                updated_at INTEGER
            )
        """)
        c.execute("""
            CREATE TABLE IF NOT EXISTS latest_tags (
                tag_id TEXT PRIMARY KEY,
                rssi REAL,
                x REAL,
                y REAL,
                r REAL,
                w REAL,
                h REAL,
                updated_at INTEGER
            )
        """)
        conn.commit()

def save_polygon(tag_id: str, polygon: Polygon or MultiPolygon):
    """

    Parameters
    ----------
    tag_id : str
    polygon : Polygon or MultiPolygon

    Saves polygon in database, overwrites if one already exist for ID

    """
    if polygon is None:
        return
    wkt_text = polygon.wkt
    ts = int(time.time())
    with sqlite3.connect(DB_FILE) as conn:
        c = conn.cursor()
        c.execute("""
            INSERT INTO polygons (tag_id, wkt, updated_at) VALUES (?, ?, ?)
            ON CONFLICT(tag_id) DO UPDATE SET wkt=excluded.wkt, updated_at=excluded.updated_at
        """, (tag_id, wkt_text, ts))
        conn.commit()

def get_polygon(tag_id: str):
    """
    Parameters
    ----------
    tag_id : str

    Returns
    -------
    polygon : ShapelyPolygon
    """
    with sqlite3.connect(DB_FILE) as conn:
        c = conn.cursor()
        c.execute("SELECT wkt FROM polygons WHERE tag_id=?", (tag_id,))
        row = c.fetchone()
    if not row:
        return None
    try:
        polygon = wkt.loads(row[0])
        return (polygon)
    except Exception:
        return None
