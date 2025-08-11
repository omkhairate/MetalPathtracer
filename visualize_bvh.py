#!/usr/bin/env python3
"""Visualize TLAS/BLAS bounding boxes from JSON dumps.

This script recursively scans a directory for `.json` files. Each file may
contain either a recursive TLAS/BLAS tree with `"bounds"` and `"children"`
entries, **or** flat `"tlas"` / `"blas"` arrays where each node stores
`"min"` and `"max"` coordinates.

Usage:
    python visualize_bvh.py /path/to/json/dir

An interactive Plotly window will open showing wireframe axis-aligned bounding
boxes for all nodes across all JSON files in the directory.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import plotly.graph_objects as go
import plotly.io as pio

# Ensure Plotly opens in a browser window rather than relying on notebook
# integrations.  This makes the script behave the same whether it is executed
# from an IDE or a terminal.
pio.renderers.default = "browser"


def load_json_files(base_dir: Path):
    """Load all JSON files under `base_dir` recursively."""
    trees = []
    for root, _dirs, files in os.walk(base_dir):
        for name in files:
            if name.endswith(".json"):
                path = Path(root) / name
                with path.open("r", encoding="utf-8") as f:
                    trees.append(json.load(f))
    return trees


def gather_boxes(tree):
    """Collect TLAS and BLAS bounding boxes from various BVH JSON layouts."""
    boxes: dict[str, list[list[float]]] = {"tlas": [], "blas": []}

    if isinstance(tree, dict) and ("tlas" in tree or "blas" in tree):
        # Current format: flat lists of TLAS/BLAS nodes with "min"/"max" bounds.
        for node in tree.get("tlas", []):
            mn = node.get("min")
            mx = node.get("max")
            if mn and mx:
                boxes["tlas"].append(mn + mx)
        for node in tree.get("blas", []):
            mn = node.get("min")
            mx = node.get("max")
            if mn and mx:
                boxes["blas"].append(mn + mx)
        return boxes

    # Fallback: recursive traversal expecting "bounds" and optional "children".
    stack = [tree]
    while stack:
        n = stack.pop()
        box = n.get("bounds")
        if box:
            boxes.setdefault("unknown", []).append(box)
        stack.extend(n.get("children", []))
    return boxes


def add_box_edges(fig, box, color="blue"):
    """Add a wireframe axis-aligned bounding box to a Plotly figure."""
    mnx, mny, mnz, mxx, mxy, mxz = box
    x = [mnx, mxx, mxx, mnx, mnx, mxx, mxx, mnx]
    y = [mny, mny, mxy, mxy, mny, mny, mxy, mxy]
    z = [mnz, mnz, mnz, mnz, mxz, mxz, mxz, mxz]
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom
        (4, 5), (5, 6), (6, 7), (7, 4),  # top
        (0, 4), (1, 5), (2, 6), (3, 7),  # verticals
    ]
    for i, j in edges:
        fig.add_trace(
            go.Scatter3d(
                x=[x[i], x[j]],
                y=[y[i], y[j]],
                z=[z[i], z[j]],
                mode="lines",
                line=dict(color=color, width=2),
                showlegend=False,
            )
        )


def visualize_boxes(boxes: dict[str, list[list[float]]]):
    """Render TLAS and BLAS boxes with different colors."""
    fig = go.Figure()
    color_map = {"tlas": "red", "blas": "blue", "unknown": "green"}
    for kind, box_list in boxes.items():
        for box in box_list:
            add_box_edges(fig, box, color_map.get(kind, "blue"))

    fig.update_layout(
        title="TLAS / BLAS Bounding Boxes",
        scene=dict(
            xaxis_title="X", yaxis_title="Y", zaxis_title="Z", aspectmode="data"
        ),
        height=700,
    )
    fig.show()


def main():
    parser = argparse.ArgumentParser(description="Visualize TLAS/BLAS bounding boxes from JSON files")
    parser.add_argument("directory", type=Path, help="Directory containing JSON dumps")
    args = parser.parse_args()

    trees = load_json_files(args.directory)
    if not trees:
        raise SystemExit(f"No JSON files found in {args.directory}")

    all_boxes: dict[str, list[list[float]]] = {"tlas": [], "blas": []}
    for tree in trees:
        gathered = gather_boxes(tree)
        for key, box_list in gathered.items():
            all_boxes.setdefault(key, []).extend(box_list)

    visualize_boxes(all_boxes)


if __name__ == "__main__":
    main()
