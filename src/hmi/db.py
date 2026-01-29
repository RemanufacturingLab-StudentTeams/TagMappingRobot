import sqlite3
import pandas as pd
from config import DB_PATH

  
def init_db():
    #Makes a database to store data, this data can be read when the mecabot is offiline

    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()

    cur.execute("""
                CREATE TABLE IF NOT EXISTS readings (
                    tag_id      TEXT PRIMARY KEY,
                    x           REAL,
                    y           REAL,
                    r           REAL,
                    w           REAL,
                    h           REAL
                    )
                """)

    conn.commit()
    conn.close()


def save_readings_to_db(data_list):
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()

    for item in data_list:
        cur.execute("""
                    INSERT INTO readings (tag_id, x, y, r, w, h)
                    VALUES (?, ?, ?, ?, ?, ?)
                    ON CONFLICT(tag_id) DO UPDATE SET
                        x = excluded.x,
                        y = excluded.y,
                        r = excluded.r,
                        w = excluded.w,
                        h = excluded.h
                    """, (
                    item["ID"],
                    item["X"],
                    item["Y"],
                    item["r"],
                    item["w"],
                    item["h"],
                    )
            )
    conn.commit()
    conn.close()

def get_latest_db():
    conn = sqlite3.connect(DB_PATH)
    df = pd.read_sql_query("""
         SELECT tag_id AS ID, x as X, y AS Y, r, w, h
         FROM readings
         """, conn)
    conn.close()
    return df    