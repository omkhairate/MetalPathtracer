#!/usr/bin/env python3
"""Visualize BLAS residency over frames as a simple HTML grid.

This script reads TLAS/BLAS dumps produced by the renderer (either a directory
containing per-frame JSON files or a single JSON file with a list of frames).
For each frame, BLAS nodes that are resident are shown in green while offloaded
nodes are shown in red.  The result is written to ``residency.html`` in the
current directory and can be viewed in any browser.

This approach avoids heavy dependencies and produces a compact, fast-to-render
visualisation suitable for quick inspection.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, Dict, Any


def _nodes_from_dump(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Convert BLAS dump data into a list of node dictionaries."""
    nodes = data.get("blas", [])
    prims = data.get("primitives", [])
    status: List[Dict[str, Any]] = [None] * len(nodes)  # type: ignore

    def compute(idx: int) -> bool:
        node = nodes[idx]
        count = node.get("count", 0)
        if count > 0:  # Leaf
            start = node.get("leftFirst", 0)
            end = start + count
            active = any(p.get("active", True) for p in prims[start:end])
        else:  # Internal
            left = node.get("leftFirst", 0)
            right = -count
            l_active = compute(left)
            r_active = compute(right)
            active = l_active or r_active
        status[idx] = {"loaded": active}
        return active

    if nodes:
        compute(0)
    return status


def _process_frame(data: Dict[str, Any], frame_index: int) -> Dict[str, Any]:
    """Normalise various frame dump formats."""
    if "tlas" in data and "blas" in data:
        return {
            "frame": frame_index,
            "nodes": _nodes_from_dump(data),
        }
    if "nodes" in data:
        data.setdefault("frame", frame_index)
        return data
    raise ValueError("Unsupported frame format")


def _load_frames(path: Path) -> List[Dict[str, Any]]:
    """Return a list of frame dictionaries from ``path``."""
    frames: List[Dict[str, Any]] = []
    if path.is_file():
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, list):
            for d in data:
                frames.append(_process_frame(d, len(frames)))
        else:
            frames.append(_process_frame(data, 0))
    else:
        for p in sorted(path.glob("*.json")):
            with p.open("r", encoding="utf-8") as f:
                frames.append(_process_frame(json.load(f), len(frames)))
    return frames


def _write_html(frames: List[Dict[str, Any]], output: Path) -> None:
    """Write residency information to ``output`` as an HTML file."""
    if not frames:
        raise SystemExit("No frames were loaded")

    max_nodes = max(len(f.get("nodes", [])) for f in frames)

    # Header row with frame indices
    header_cells = ["<th></th>"]
    for idx, frame in enumerate(frames):
        header_cells.append(f"<th>{frame.get('frame', idx)}</th>")
    rows: List[str] = ["<tr>" + "".join(header_cells) + "</tr>"]

    # Body rows for each BLAS node
    for node_idx in range(max_nodes):
        cells: List[str] = [f"<th>{node_idx}</th>"]
        for frame_idx, frame in enumerate(frames):
            nodes = frame.get("nodes", [])
            loaded = nodes[node_idx]["loaded"] if node_idx < len(nodes) else False
            cls = "loaded" if loaded else "offloaded"
            title = f"Node {node_idx}, Frame {frame_idx}: {'loaded' if loaded else 'offloaded'}"
            cells.append(f'<td class="{cls}" title="{title}"></td>')
        rows.append("<tr>" + "".join(cells) + "</tr>")

    html = f"""<!DOCTYPE html>
<html>
<head>
<meta charset='utf-8'>
<style>
  table.residency {{ border-collapse: collapse; }}
  table.residency th, table.residency td {{ width: 12px; height: 12px; padding: 0; text-align: center; }}
  table.residency th {{ font-size: 10px; background: #fafafa; }}
  .loaded {{ background: #4caf50; }}
  .offloaded {{ background: #f44336; }}
  .legend {{ margin-bottom: 1em; }}
  .legend .swatch {{ display: inline-block; width: 12px; height: 12px; margin-right: 4px; vertical-align: middle; }}
</style>
</head>
<body>
<div class='legend'>
  <span class='swatch loaded'></span> Loaded
  <span class='swatch offloaded' style='margin-left:1em;'></span> Offloaded
</div>
<table class='residency'>
<tbody>
{'\n'.join(rows)}
</tbody>
</table>
</body>
</html>
"""
    output.write_text(html, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a BLAS residency heatmap")
    parser.add_argument("path", type=Path, help="Directory or JSON file containing frame dumps")
    parser.add_argument("--output", type=Path, default=Path("residency.html"), help="Output HTML file")
    args = parser.parse_args()

    frames = _load_frames(args.path)
    _write_html(frames, args.output)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
