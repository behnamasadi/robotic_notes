import plotly.graph_objects as go
import g2o

def plot_slam2d(optimizer, title):
    def edges_coord(edges, dim):
        for e in edges:
            yield e.vertices()[0].estimate()[dim]
            yield e.vertices()[1].estimate()[dim]
            yield None

    fig = go.Figure()

    # edges
    edges = optimizer.edges()  # get set once to have same order
    se2_edges = [e for e in edges if type(e) == g2o.EdgeSE2]
    se2_pointxy_edges = [e for e in edges if type(e) == g2o.EdgeSE2PointXY]
    
    
    fig.add_trace(
        go.Scatter(
            x=list(edges_coord(se2_pointxy_edges, 0)),
            y=list(edges_coord(se2_pointxy_edges, 1)),
            mode="lines",
            line=dict(
                color='firebrick', 
                width=2,
                dash='dash'),
            name="Measurement edges",
            legendgroup="Measurements"
            
        )
    )
    
    fig.add_trace(
        go.Scatter(
            x=list(edges_coord(se2_edges, 0)),
            y=list(edges_coord(se2_edges, 1)),
            mode="lines",
            line=dict(
                color='midnightblue', 
                width=4),
            name="Pose edges",
            legendgroup="Poses"
        )
    )
    
    # poses of the vertices
    vertices = optimizer.vertices()
    poses = [v.estimate() for v in vertices.values() if type(v) == g2o.VertexSE2]
    measurements = [v.estimate() for v in vertices.values() if type(v) == g2o.VertexPointXY]
    
    
    fig.add_trace(
        go.Scatter(
            x=[v[0] for v in poses],
            y=[v[1] for v in poses],
            mode="markers",
            marker_line_color="midnightblue", 
            marker_color="lightskyblue",
            marker_line_width=2, marker_size=15,
            name="Poses",
            legendgroup="Poses"
        )
    )
    
    fig.add_trace(
        go.Scatter(
            x=[v[0] for v in measurements],
            y=[v[1] for v in measurements],
            mode="markers",
            marker_symbol="star",
            marker_line_color="firebrick",
            marker_color="firebrick",
            marker_line_width=2, marker_size=15,
            name="Measurements",
            legendgroup="Measurements"
        )
    )

    fig.update_yaxes(
        scaleanchor="x",
        scaleratio=1,
    )
    fig.update_layout(go.Layout({"title": title}))

    return fig