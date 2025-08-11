#!/usr/bin/env python3
"""Animate BVH node residency across frames.

This tool visualises when BVH nodes are loaded or unloaded by animating
frame-by-frame logs.  Each frame is expected to contain an array of
nodes with bounding boxes and a boolean ``loaded`` flag:

{
    "frame": 0,
    "nodes": [
        {"id": 42, "min": [x, y, z], "max": [x, y, z], "loaded": true},
        ...
    ]
}

The script accepts either a directory containing one JSON file per
frame or a single JSON file holding a list of frame objects.
Loaded nodes are drawn in green while unloaded nodes are drawn in red.

Usage::

    python visualize_bvh.py /path/to/frame_logs

A browser window will open showing an animatable 3D view of the BVH.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import plotly.graph_objects as go
import plotly.io as pio

# Force Plotly to open a browser window regardless of environment
pio.renderers.default = "browser"


def _load_frames(path: Path):
    """Return a list of frame dictionaries from ``path``."""
    frames = []
    if path.is_file():
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list):
            frames.extend(data)
        else:
            frames.append(data)
    else:
        for p in sorted(path.glob("*.json")):
            with p.open("r", encoding="utf-8") as f:
                frames.append(json.load(f))
    return frames


def _box_edges(node, color: str):
    """Return Plotly line traces for a single bounding box."""
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


def _frame_traces(frame):
    traces = []
    for node in frame.get("nodes", []):
        color = "green" if node.get("loaded", True) else "red"
        traces.extend(_box_edges(node, color))
    return traces


def _visualize(frames):
    if not frames:
        raise SystemExit("No frames were loaded")

    fig = go.Figure(
        data=_frame_traces(frames[0]),
        frames=[
            go.Frame(data=_frame_traces(f), name=str(f.get("frame", i)))
            for i, f in enumerate(frames)
        ],
    )

    fig.update_layout(
        title="BVH Node Residency",
        scene=dict(
            xaxis_title="X",
            yaxis_title="Y",
            zaxis_title="Z",
            aspectmode="data",
        ),
        updatemenus=[{
            "type": "buttons",
            "buttons": [
                {
                    "label": "Play",
                    "method": "animate",
                    "args": [None, {"frame": {"duration": 500, "redraw": True}, "fromcurrent": True}],
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
                    "args": [[str(f.get("frame", i))], {"frame": {"duration": 0}, "mode": "immediate"}],
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
    parser = argparse.ArgumentParser(description="Animate BVH node residency over time")
    parser.add_argument("path", type=Path, help="Directory or JSON file containing frame logs")
    args = parser.parse_args()

    frames = _load_frames(args.path)
    _visualize(frames)


if __name__ == "__main__":
    main()
