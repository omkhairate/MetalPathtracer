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
    """Collect bounding boxes from various BVH JSON layouts."""
    boxes: list[list[float]] = []

    if isinstance(tree, dict) and ("tlas" in tree or "blas" in tree):
        # New format: flat lists of TLAS/BLAS nodes with "min"/"max" bounds.
        for key in ("tlas", "blas"):
            for node in tree.get(key, []):
                mn = node.get("min")
                mx = node.get("max")
                if mn and mx:
                    boxes.append(mn + mx)
        return boxes

    # Fallback: recursive traversal expecting "bounds" and optional "children".
    stack = [tree]
    while stack:
        n = stack.pop()
        box = n.get("bounds")
        if box:
            boxes.append(box)
        stack.extend(n.get("children", []))
    return boxes


def add_box_edges(fig, box, **line_kwargs):
    """Add a wireframe AABB to a Plotly figure."""
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
                line=dict(color="blue", width=2, **line_kwargs),
                showlegend=False,
            )
        )


def visualize_boxes(boxes):
    fig = go.Figure()
    for box in boxes:
        add_box_edges(fig, box)
    fig.update_layout(
        title="TLAS / BLAS Bounding Boxes",
        scene=dict(xaxis_title="X", yaxis_title="Y", zaxis_title="Z", aspectmode="data"),
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

    all_boxes = []
    for tree in trees:
        all_boxes.extend(gather_boxes(tree))

    visualize_boxes(all_boxes)


if __name__ == "__main__":
    main()
