#!/usr/bin/env python3
"""Visualize TLAS/BLAS node residency and primitive visibility.

This tool animates top-level and bottom-level acceleration structure
nodes along with individual primitives to show how data is loaded or
unloaded as the camera moves.  Each frame log is expected to be a
JSON object with the following structure:

{
    "frame": 0,
    "camera": {"min": [x,y,z], "max": [x,y,z]},  # optional view box
    "tlas": [
        {"id": 0, "min": [x,y,z], "max": [x,y,z], "loaded": true},
        ...
    ],
    "blas": [
        {"id": 0, "min": [x,y,z], "max": [x,y,z], "loaded": true},
        ...
    ],
    "primitives": [
        {"id": 0, "position": [x,y,z], "loaded": true},
        ...
    ]
}

The script accepts a directory containing one JSON file per frame or a
single JSON file that stores a list of frames.  TLAS nodes are drawn in
blue (loaded) or light blue (unloaded), BLAS nodes in green (loaded) or
light green (unloaded) and primitives as black (loaded) or grey
(unloaded) points.  The camera bounding box, if supplied, is drawn in
red.  Use the slider or Play button in the rendered browser window to
animate through frames and watch nodes/primitives move into and out of
the view.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import plotly.graph_objects as go
import plotly.io as pio

# Always try to pop open a browser window
pio.renderers.default = "browser"


def _load_frames(path: Path):
    frames = []
    if path.is_file():
        with path.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
        if isinstance(data, list):
            frames.extend(data)
        else:
            frames.append(data)
    else:
        for p in sorted(path.glob("*.json")):
            with p.open("r", encoding="utf-8") as fh:
                frames.append(json.load(fh))
    return frames


def _box_traces(node: dict, color: str):
    mnx, mny, mnz = node["min"]
    mxx, mxy, mxz = node["max"]
    x = [mnx, mxx, mxx, mnx, mnx, mxx, mxx, mnx]
    y = [mny, mny, mxy, mxy, mny, mny, mxy, mxy]
    z = [mnz, mnz, mnz, mnz, mxz, mxz, mxz, mxz]
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    traces = []
    for i, j in edges:
        traces.append(
            go.Scatter3d(
                x=[x[i], x[j]],
                y=[y[i], y[j]],
                z=[z[i], z[j]],
                mode="lines",
                line=dict(color=color, width=2),
                showlegend=False,
            )
        )
    return traces


def _camera_trace(frame: dict):
    cam = frame.get("camera")
    if not cam:
        return []
    return _box_traces(cam, "red")


def _tlas_traces(frame: dict):
    traces = []
    for n in frame.get("tlas", []):
        color = "blue" if n.get("loaded", True) else "lightblue"
        traces.extend(_box_traces(n, color))
    return traces


def _blas_traces(frame: dict):
    traces = []
    for n in frame.get("blas", []):
        color = "green" if n.get("loaded", True) else "lightgreen"
        traces.extend(_box_traces(n, color))
    return traces


def _primitive_traces(frame: dict):
    pts = frame.get("primitives", [])
    if not pts:
        return []
    x, y, z, color = [], [], [], []
    for p in pts:
        px, py, pz = p["position"]
        x.append(px)
        y.append(py)
        z.append(pz)
        color.append("black" if p.get("loaded", True) else "gray")
    return [
        go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode="markers",
            marker=dict(color=color, size=2),
            showlegend=False,
        )
    ]


def _frame_data(frame: dict):
    data = []
    data.extend(_camera_trace(frame))
    data.extend(_tlas_traces(frame))
    data.extend(_blas_traces(frame))
    data.extend(_primitive_traces(frame))
    return data


def _visualize(frames):
    if not frames:
        raise SystemExit("no frames loaded")

    fig = go.Figure(
        data=_frame_data(frames[0]),
        frames=[go.Frame(data=_frame_data(f), name=str(f.get("frame", i)))
                for i, f in enumerate(frames)],
    )

    fig.update_layout(
        title="Acceleration Structure Residency",
        scene=dict(aspectmode="data"),
        updatemenus=[{
            "type": "buttons",
            "buttons": [
                {
                    "label": "Play",
                    "method": "animate",
                    "args": [None, {"frame": {"duration": 500, "redraw": True},
                                      "fromcurrent": True}],
                },
                {
                    "label": "Pause",
                    "method": "animate",
                    "args": [[None], {"frame": {"duration": 0}, "mode": "immediate"}],
                },
            ],
        }],
        sliders=[{
            "steps": [
                {
                    "args": [[str(f.get("frame", i))],
                              {"frame": {"duration": 0}, "mode": "immediate"}],
                    "label": str(f.get("frame", i)),
                    "method": "animate",
                }
                for i, f in enumerate(frames)
            ]
        }],
        height=700,
    )

    fig.show()


def main():
    parser = argparse.ArgumentParser(
        description="Animate TLAS/BLAS node residency and primitive visibility")
    parser.add_argument("path", type=Path,
                        help="Directory or JSON file containing frame logs")
    args = parser.parse_args()

    frames = _load_frames(args.path)
    _visualize(frames)


if __name__ == "__main__":
    main()

