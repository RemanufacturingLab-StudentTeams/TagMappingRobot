#!/usr/bin/env python3
"""
Created on Thu Jan 15 15:29:27 2026

@author: renyb
"""

from dash import Dash, html, dash_table, dcc, Output, Input, State
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
import os

import json
import threading
import paho.mqtt.client as mqtt
import sqlite3

current_df = pd.DataFrame()
name_file = "tag_names.csv"     #Tag names file
DB_PATH = "tag_data_HMI.db"         #Database file
start_data = []



#Called when MQTT client connects to the broker
def on_connect_function(client, userdata, flags, rc):
    print("Connected with result code" + str(rc))
    client.subscribe("HMI/topic") ##dit veranderen!!!!

#Called when new MQTT message arrive
#Message in msg.payload
def on_message_function(client, userdata, msg):

    global current_df
    message = msg.payload.decode()
    data_list = json.loads(message)

    save_readings_to_db(data_list)

    message_df = pd.DataFrame(
        data_list,
        columns=["ID", "X", "Y", "r", "w", "h"])

    if message_df.empty:
        current_df = message_df
        return

    if current_df is None or current_df.empty:
        current_df = message_df
        return

    # --- UPSERT op ID ---
    # zet ID als index zodat update makkelijk is
    current_df = current_df.set_index("ID")
    message_df = message_df.set_index("ID")

    # update bestaande IDs (overschrijft X,Y,r,w,h)
    current_df.update(message_df)

    # voeg nieuwe IDs toe die nog niet bestonden
    new_ids = message_df.index.difference(current_df.index)
    if not new_ids.empty:
        current_df = pd.concat([current_df, message_df.loc[new_ids]])

    # terug naar normale kolom
    current_df = current_df.reset_index()



def mqtt_thread():
    client = mqtt.Client()
    #Event functions for when client connects to broker
    client.on_connect = on_connect_function
    client.on_message = on_message_function
    #Connect to broker
    client.connect("10.35.4.195", 1883, 60)
    client.loop_forever()






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



#Als name bestand niet bestaat dan eentje aanmaken
if not os.path.exists(name_file):
    pd.DataFrame(columns=["ID", "name"]).to_csv(name_file, index=False)



#Create object of Dash app
app = Dash()


#Function that creates figure
def create_figure(df):
    # Zorg dat df een DataFrame is
    if isinstance(df, list):
        df = pd.DataFrame(df)
    elif df is None:
        df = pd.DataFrame()

    # Geen of lege data → lege figuur met assen
    if df.empty:
        fig = go.Figure()
    else:
        fig = px.scatter(df, x='X', y='Y', hover_name="name")
        fig.update_traces(marker=dict(size=15, color='red'))

        theta = np.linspace(0, 2*np.pi, 100)

        for _, row in df.iterrows():
            x0, y0 = row["X"], row["Y"]
            w = row["w"]
            h = row["h"]
            angle = np.deg2rad(row["r"])  # rotatiehoek in radialen

            x_ellipse = x0 + (w/2) * np.cos(theta) * np.cos(angle) - (h/2) * np.sin(theta) * np.sin(angle)
            y_ellipse = y0 + (w/2) * np.cos(theta) * np.sin(angle) + (h/2) * np.sin(theta) * np.cos(angle)

            fig.add_trace(go.Scatter(
                x=x_ellipse,
                y=y_ellipse,
                mode="lines",
                line=dict(color="blue", width=1),
                hoverinfo="skip"
            ))


    fig.add_layout_image( {'source': '/assets/WHEELTEC.png', 'xref': 'x', 'yref': 'y', 'x': -6.26, 'y': 12.23, 'sizex': 19.400000000000002, 'sizey': 17.5, 'sizing': 'stretch', 'layer': 'below', 'opacity': 0.8} )
    fig.update_layout(
        xaxis_scaleanchor="y",  # equal aspect ratio (1m = 1m)
        xaxis_title="X [m]",
        yaxis_title="Y [m]",
        margin=dict(l=20, r=20, t=30, b=20),
        uirevision="constant",
        plot_bgcolor="#cdcdcd"
    )
    fig.update_xaxes(range=[-6.26, 13.140000000000002])
    fig.update_yaxes(range=[-5.27, 12.23])

    return fig





empty_df = pd.DataFrame(columns=["ID", "X", "Y", "r", "w", "h", "name"])
fig = create_figure(empty_df)

# Requires Dash 2.17.0 or later
app.layout = html.Div(
    style={
        "backgroundColor": "#9FA700",  # groene achtergrond
        "minHeight": "100vh",          # zorgt dat het hele scherm wordt gevuld
        "padding": "20px",
        "textAlign": "center"
    },
    children=[
        html.H3(
        "Tag Information",
        style={

            "color": "white",
            "fontFamily": "Frutiger, Segoe UI, Helvetica, Arial, sans-serif",
            "fontSize": "42px",
            "fontWeight": "700",
            "marginBottom": "5px"
        }
        ),

        html.H5(
            "Tag Mapping Group",
            style={
                "color": "#4D4D4D",
                "fontFamily": "Frutiger, Segoe UI, Helvetica, Arial, sans-serif",
                "fontSize": "20px",
                "fontStyle": "italic",
                "marginTop": "0"
            }
        ),

        #Create data table with no data in it. We can also delete it
        html.Div(
            style={
                "display": "flex",
                "justifyContent": "space-between",
                "alignItems": "flex-start",
                "gap": "20px",
                "flexWrap": "wrap",
            },
            children=[
                # Links: tabel (niet full width)
                html.Div(
                    style={"flex": "0 0 520px", "maxWidth": "1200px"},
                    children=[
                        dash_table.DataTable(
                            id="data-table1",
                            data=start_data,
                            columns=[
                                {"name": "ID", "id": "ID"},
                                {"name": "X", "id": "X"},
                                {"name": "Y", "id": "Y"},
                                {"name": "r", "id": "r"},
                                {"name": "w", "id": "w"},
                                {"name": "h", "id": "h"},
                                {"name": "name", "id": "name"},
                            ],
                            page_size=10,
                            style_table={"width": "1200px", "overflowX": "auto"},
                            style_cell={"textAlign": "left", "padding": "6px"},
                        )
                    ],
                ),

                # Rechts: knoppen + status (komt zo)
                # Rechts: knoppen + status (onder elkaar)
                html.Div(
                    style={"flex": "1", "minWidth": "260px", "textAlign": "right"},
                    children=[
                        html.Div(
                            style={
                                "display": "flex",
                                "flexDirection": "column",   # <-- onder elkaar
                                "alignItems": "flex-end",    # rechts uitlijnen
                                "gap": "10px"
                            },
                            children=[
                                html.Button(
                                    "STOP SCANNING",
                                    id="btn-red",
                                    n_clicks=0,
                                    style={
                                        "backgroundColor": "#d11a2a",
                                        "color": "white",
                                        "border": "none",
                                        "padding": "10px 14px",
                                        "borderRadius": "10px",
                                        "fontWeight": "700",
                                        "cursor": "pointer",
                                        "width": "300px",
                                        "height": "100px"
                                    },
                                ),
                                html.Button(
                                    "START SCANNING",
                                    id="btn-green",
                                    n_clicks=0,
                                    style={
                                        "backgroundColor": "#1aa64a",
                                        "color": "white",
                                        "border": "none",
                                        "padding": "10px 14px",
                                        "borderRadius": "10px",
                                        "fontWeight": "700",
                                        "cursor": "pointer",
                                        "width": "300px",
                                        "height": "100px"
                                    },
                                ),
                            ],
                        ),
                        html.Div(
                            id="mqtt-status",
                            style={"marginTop": "12px", "color": "white", "textAlign": "right"}
                        ),
                    ],
                ),

            ],
        ),
        html.Br(),


        html.Div(
            style={
                "display": "flex",
                "justifyContent": "center",
                "alignItems": "flex-start",
                "gap": "40px",
                "flexWrap": "wrap",
                "maxWidth": "95vw",
                "margin": "auto",
                "marginTop": "20px"

            },
            children=[
                # Linkerkant: scatter plot
                dcc.Graph(id="scatter-plot", figure=fig, style={"flex": "3", "width": "400px", "height":"500px"}),

                # ➡️ Rechterkant: melding en input
                html.Div(
                    id="name-input-div",
                    children=[
                        html.P(
                            id="unknown-tag-text",
                            children="",
                            style={
                                "fontWeight": "bold",
                                "color": "white",
                                "backgroundColor": "#4D4D4D",
                                "padding": "10px",
                                "borderRadius": "8px"
                            }
                        ),
                        dcc.Input(
                            id="new-name-input",
                            type="text",
                            placeholder="Naam...",
                            style={"marginTop": "10px"}
                        ),
                        html.Button(
                            "Opslaan",
                            id="save-name-btn",
                            n_clicks=0,
                            style={"marginTop": "10px"}
                        ),
                    ],
                    style={
                        "flex": "1",
                        "minWidth": "250px",
                        "display": "none"  # wordt dynamisch “block” door de callback
                    },
                ),
            ],
        ),
        html.Br(),
        dcc.Interval(id='interval1', interval = 2000, n_intervals=0)


        ]

    )

#Update data each two seconds
@app.callback(
    [Output("data-table1", "data"),
     Output("scatter-plot", "figure"),
     Output("name-input-div", "style"),
     Output("unknown-tag-text", "children")],
    [Input("interval1", "n_intervals")]
)
def update_data(n_intervals):
    global current_df

    # Als er nog geen MQTT-data is, maak een lege df met de juiste kolommen
    if current_df is None or current_df.empty:
        current_df = get_latest_db()

    if current_df is None or current_df.empty:
        empty_df = pd.DataFrame(columns=["ID", "X", "Y", "r", "w", "h", "name"])
        fig = create_figure(empty_df)
        return [], fig, {"display": "none"}, ""

    # namen inlezen
    names = pd.read_csv(name_file)      # kolommen: ID, name

    # both ID's from the csv file and data base same type
    current_df["ID"] = current_df["ID"].astype(str)
    names["ID"] = names["ID"].astype(str)

    # merge op ID de bijbehorende namen, ** wat is how= left?
    df_view = current_df.merge(names, on="ID", how="left")


    # onbekende namen opvullen
    df_view["name"] = df_view["name"].fillna("Onbekend")
    table_records = df_view.to_dict("records")


    # onbekende tags zoeken
    onbekende_tags = df_view[df_view["name"] == "Onbekend"]["ID"].unique()

    # figuur maken met deze df
    fig = create_figure(df_view)



    if len(onbekende_tags) > 0:
        tag_id = onbekende_tags[0]
        text = [
            "Nieuwe tag gevonden!",
            html.Br(),
            f"Geef een naam voor tag ID: {tag_id}"
        ]
        return (
            table_records,
            fig,
            {"display": "block"},
            text
        )
    else:
        return (
            table_records,
            fig,
            {"display": "none"},
            ""
        )



@app.callback(
    Output("new-name-input", "value"),
    Input("save-name-btn", "n_clicks"),
    State("new-name-input", "value")
)
def save_new_name(n_clicks, new_name):
    global current_df

    if n_clicks > 0 and new_name:
        if current_df is None or current_df.empty:
            data = get_latest_db()
            print("error 1")
        else:
            data = current_df
        print("okay 1")
        # namen inlezen
        names = pd.read_csv(name_file)      # kolommen: ID, name

        #Both ID as string
        data["ID"] = data["ID"].astype(str)
        names["ID"] = names["ID"].astype(str)

        # merge op ID
        df_check = data.merge(names, on="ID", how="left")

        df_check["name"] = df_check["name"].fillna("Onbekend")

        unknown = df_check[df_check["name"] == "Onbekend"]["ID"]

        if unknown.empty:
            print("unkown empty")
            return""
        tag_id = unknown.iloc[0]


        names.loc[len(names)] = {"ID": tag_id, "name": new_name}
        names.to_csv(name_file, index=False)
        print(f"Nieuwe naam toegevoegd: {tag_id} → {new_name}")
    # Leeg inputveld na opslaan
    return ""


if __name__ == '__main__':
    init_db()
    #MQTT subscriber in a separate thread
    threading.Thread(target=mqtt_thread, daemon=True).start()
    app.run(debug=False)
