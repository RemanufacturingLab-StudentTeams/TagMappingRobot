import numpy as np
import plotly.express as px
import plotly.graph_objects as go
from config import LAYOUT_IMAGE, AXIS_RANGE_CONFIG


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

    fig.add_layout_image(LAYOUT_IMAGE)
    fig.update_layout(
        xaxis_scaleanchor="y",  # equal aspect ratio (1m = 1m)
        xaxis_title="X [m]",
        yaxis_title="Y [m]",
        margin=dict(l=20, r=20, t=30, b=20),
        uirevision="constant",
        plot_bgcolor='#cdcdcd'
    )
    fig.update_xaxes(range=AXIS_RANGE_CONFIG["x"])
    fig.update_yaxes(range=AXIS_RANGE_CONFIG["y"])

    return fig