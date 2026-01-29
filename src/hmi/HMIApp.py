#!/usr/bin/env python3
"""
Created on Thu Jan 15 15:29:27 2026

@author: renyb
"""

from dash import Dash, html, dash_table, dcc, Output, Input, State
import pandas as pd

from config import UPDATE_INTERVAL_MS
from db import init_db, get_latest_db
from figure import create_figure
from mqtt_client import start_mqtt_in_thread, current_df
from utils import load_names, append_name
start_data = []


#Create object of Dash app
app = Dash(__name__)


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
                                "flexDirection": "column",  
                                "alignItems": "flex-end",    
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
        dcc.Interval(id='interval1', interval = UPDATE_INTERVAL_MS, n_intervals=0)


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
        df_live = get_latest_db()
    else:
        df_live = current_df.copy()
        

    if df_live is None or df_live.empty:
        empty_df = pd.DataFrame(columns=["ID", "X", "Y", "r", "w", "h", "name"])
        fig = create_figure(empty_df)
        return [], fig, {"display": "none"}, ""

    # namen inlezen
    names = load_names()
    
    # both ID's from the csv file and data base same type
    df_live["ID"] = df_live["ID"].astype(str)

    # merge op ID de bijbehorende namen, ** wat is how= left?
    df_view = df_live.merge(names, on="ID", how="left")


    # onbekende namen opvullen
    df_view["name"] = df_view["name"].fillna("Unknown")
    table_records = df_view.to_dict("records")


    # onbekende tags zoeken
    unknown_tags = df_view[df_view["name"] == "Unknown"]["ID"].unique()

    # figuur maken met deze df
    fig = create_figure(df_view)



    if len(unknown_tags) > 0:
        tag_id = unknown_tags[0]
        text = ["New tag found!", html.Br(), f"Enter a name for tag ID: {tag_id}"]
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
            df_live = get_latest_db()
        if df_live is None or df_live.empty:
            return ""
        
        names = load_names()    
        #Both ID as string
        df_live["ID"] = df_live["ID"].astype(str)
 
        # merge op ID
        df_check = df_live.merge(names, on="ID", how="left")

        df_check["name"] = df_check["name"].fillna("Unknwon")

        unknown = df_check[df_check["name"] == "Unknwon"]["ID"]

        if unknown.empty:

            return""
        tag_id = unknown.iloc[0]
        append_name(tag_id, new_name)
        print(f"Added new name: {tag_id} → {new_name}")
    # Leeg inputveld na opslaan
    return ""


if __name__ == '__main__':
    init_db()
    start_mqtt_in_thread()
    app.run(debug=False)
