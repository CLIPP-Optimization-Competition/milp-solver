#!/usr/bin/env python3
"""
Generate an interactive HTML visualization for a CLIPP instance.

Usage:
    python visualize_instance.py "1. Instances/toy.txt"
    python visualize_instance.py "1. Instances/A_instance.txt" --open
"""

import argparse
import json
import math
import random
import webbrowser
from pathlib import Path

def parse_instance(filepath):
    """Parse CLIPP instance files (same format as solver)."""
    with open(filepath, encoding="utf-8") as fh:
        raw = [ln.strip() for ln in fh if ln.strip()]

    idx = 0
    header = raw[idx].split()
    idx += 1

    n = int(header[0])
    m = int(header[1])
    t_limit = int(header[2])
    c = int(header[3])
    depot = int(header[4])
    w_penalty = float(header[5]) if len(header) >= 6 else 0.0

    junction_coords = []
    if idx < len(raw):
        tokens = raw[idx].split()
        if len(tokens) == 2:
            try:
                float(tokens[0])
                float(tokens[1])
                for _ in range(n):
                    parts = raw[idx].split()
                    junction_coords.append((float(parts[0]), float(parts[1])))
                    idx += 1
            except ValueError:
                pass

    streets = []
    for j in range(m):
        parts = raw[idx].split()
        idx += 1
        a = int(parts[0])
        b = int(parts[1])
        direction = int(parts[2])
        cost = int(parts[3])
        length = int(parts[4])
        cat = parts[5]
        req = int(parts[6])
        streets.append(
            {
                "id": j,
                "a": a,
                "b": b,
                "dir": direction,
                "cost": cost,
                "length": length,
                "cat": cat,
                "req": req,
            }
        )

    vehicle_tokens = raw[idx].split()
    cap = {"S": 10, "M": 20, "L": 30}
    vehicles = [{"type": tok, "cap": cap[tok]} for tok in vehicle_tokens]

    return {
        "N": n,
        "M": m,
        "T": t_limit,
        "C": c,
        "S": depot,
        "W": w_penalty,
        "streets": streets,
        "vehicles": vehicles,
        "junction_coords": junction_coords,
    }


def _normalize_points(points, width, height, pad):
    """Scale points to fit inside [pad, width-pad] x [pad, height-pad]."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    span_x = max(max_x - min_x, 1e-9)
    span_y = max(max_y - min_y, 1e-9)

    sx = (width - 2 * pad) / span_x
    sy = (height - 2 * pad) / span_y
    scale = min(sx, sy)

    out = []
    for x, y in points:
        nx = pad + (x - min_x) * scale
        ny = pad + (y - min_y) * scale
        out.append((nx, ny))
    return out


def _force_layout(instance, width, height, seed=42, iterations=240):
    """
    Deterministic Fruchterman-Reingold style layout for instances without coords.
    Complexity is O(iterations * N^2), reasonable for the provided instance sizes.
    """
    n = instance["N"]
    rng = random.Random(seed)
    pad = 60.0

    if n == 1:
        return [(width / 2.0, height / 2.0)]

    # Initial circle + tiny jitter for stable yet non-uniform starts
    cx = width / 2.0
    cy = height / 2.0
    radius = min(width, height) * 0.35
    pos = []
    for i in range(n):
        angle = 2.0 * math.pi * i / n
        x = cx + radius * math.cos(angle) + rng.uniform(-8.0, 8.0)
        y = cy + radius * math.sin(angle) + rng.uniform(-8.0, 8.0)
        pos.append([x, y])

    # Use one undirected spring per street
    springs = [(s["a"], s["b"]) for s in instance["streets"]]
    area = max((width - 2 * pad) * (height - 2 * pad), 1.0)
    k = math.sqrt(area / float(n))
    temperature = min(width, height) * 0.12

    for _ in range(iterations):
        disp = [[0.0, 0.0] for _ in range(n)]

        # Repulsive forces between all node pairs
        for i in range(n):
            xi, yi = pos[i]
            for j in range(i + 1, n):
                xj, yj = pos[j]
                dx = xi - xj
                dy = yi - yj
                dist = math.hypot(dx, dy) + 1e-9
                force = (k * k) / dist
                fx = (dx / dist) * force
                fy = (dy / dist) * force
                disp[i][0] += fx
                disp[i][1] += fy
                disp[j][0] -= fx
                disp[j][1] -= fy

        # Attractive forces along streets
        for u, v in springs:
            dx = pos[u][0] - pos[v][0]
            dy = pos[u][1] - pos[v][1]
            dist = math.hypot(dx, dy) + 1e-9
            force = (dist * dist) / k
            fx = (dx / dist) * force
            fy = (dy / dist) * force
            disp[u][0] -= fx
            disp[u][1] -= fy
            disp[v][0] += fx
            disp[v][1] += fy

        # Move nodes with temperature-limited step
        for i in range(n):
            dx, dy = disp[i]
            disp_len = math.hypot(dx, dy)
            if disp_len > 0:
                step = min(disp_len, temperature)
                pos[i][0] += (dx / disp_len) * step
                pos[i][1] += (dy / disp_len) * step
            pos[i][0] = min(width - pad, max(pad, pos[i][0]))
            pos[i][1] = min(height - pad, max(pad, pos[i][1]))

        temperature *= 0.96
        if temperature < 0.5:
            break

    return [(p[0], p[1]) for p in pos]


def build_layout(instance, width, height):
    coords = instance.get("junction_coords", [])
    if coords and len(coords) == instance["N"]:
        return _normalize_points(coords, width, height, pad=60.0)
    return _force_layout(instance, width, height)


def build_view_data(instance, points, width, height):
    depot = instance["S"]
    streets = instance["streets"]
    vehicles = instance["vehicles"]

    degree = [0] * instance["N"]
    for s in streets:
        degree[s["a"]] += 1
        degree[s["b"]] += 1

    nodes = []
    for i, (x, y) in enumerate(points):
        nodes.append(
            {
                "id": i,
                "x": round(x, 2),
                "y": round(y, 2),
                "isDepot": i == depot,
                "degree": degree[i],
            }
        )

    edges = []
    for s in streets:
        label = f"#{s['id']} | cost={s['cost']}s len={s['length']}m req={s['req']}"
        edges.append(
            {
                "id": s["id"],
                "a": s["a"],
                "b": s["b"],
                "dir": s["dir"],
                "cat": s["cat"],
                "cost": s["cost"],
                "length": s["length"],
                "req": s["req"],
                "label": label,
            }
        )

    cat_counts = {"M": 0, "O": 0, "C": 0}
    for s in streets:
        cat_counts[s["cat"]] += 1

    return {
        "meta": {
            "N": instance["N"],
            "M": instance["M"],
            "T": instance["T"],
            "C": instance["C"],
            "S": instance["S"],
            "W": instance["W"],
            "fleet": [v["type"] for v in vehicles],
            "categoryCounts": cat_counts,
            "viewWidth": width,
            "viewHeight": height,
        },
        "nodes": nodes,
        "edges": edges,
    }


def build_html(data):
    data_json = json.dumps(data, separators=(",", ":"))
    html = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CLIPP Instance Graph View</title>
  <style>
    :root {
      --bg: #f5f7fb;
      --panel: #ffffff;
      --ink: #18212f;
      --muted: #4a5972;
      --mandatory: #cc2a2a;
      --optional: #1564c0;
      --connector: #7e889a;
      --node: #2d5f5d;
      --depot: #ff9a2f;
      --grid: #e6ebf3;
      --sel: #111111;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Segoe UI", Tahoma, Geneva, Verdana, sans-serif;
      background:
        radial-gradient(circle at 85% 10%, #d9e6ff 0%, transparent 44%),
        radial-gradient(circle at 10% 90%, #e9f0ff 0%, transparent 42%),
        var(--bg);
      color: var(--ink);
    }
    .wrap {
      min-height: 100vh;
      display: grid;
      grid-template-columns: 320px 1fr;
      gap: 14px;
      padding: 14px;
    }
    .panel {
      background: var(--panel);
      border: 1px solid #d8dfeb;
      border-radius: 12px;
      padding: 14px;
      box-shadow: 0 3px 10px rgba(13, 29, 56, 0.08);
    }
    .title {
      margin: 0 0 8px;
      font-size: 20px;
      line-height: 1.1;
    }
    .sub {
      margin: 0 0 8px;
      color: var(--muted);
      font-size: 13px;
    }
    .stat {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 8px;
      font-size: 13px;
      padding: 4px 0;
      border-bottom: 1px solid #eef2f7;
    }
    .row {
      display: flex;
      align-items: center;
      gap: 8px;
      font-size: 14px;
      margin: 8px 0;
    }
    .legend-dot {
      width: 12px;
      height: 12px;
      border-radius: 3px;
      border: 1px solid rgba(0, 0, 0, 0.2);
    }
    .mandatory { background: var(--mandatory); }
    .optional { background: var(--optional); }
    .connector { background: var(--connector); }
    .depot { background: var(--depot); border-radius: 50%; }
    .graph-card {
      background: var(--panel);
      border: 1px solid #d8dfeb;
      border-radius: 12px;
      overflow: hidden;
      box-shadow: 0 3px 10px rgba(13, 29, 56, 0.08);
      position: relative;
    }
    .graph-head {
      padding: 10px 14px;
      border-bottom: 1px solid #e8eef8;
      font-size: 13px;
      color: var(--muted);
      background: #f9fbff;
    }
    svg {
      width: 100%;
      height: calc(100vh - 62px);
      display: block;
      background:
        linear-gradient(0deg, transparent 23px, var(--grid) 24px),
        linear-gradient(90deg, transparent 23px, var(--grid) 24px),
        #ffffff;
      background-size: 24px 24px;
    }
    .edge {
      stroke-linecap: round;
      opacity: 0.9;
      cursor: pointer;
    }
    .edge-cat-M { stroke: var(--mandatory); stroke-width: 4.4; }
    .edge-cat-O { stroke: var(--optional); stroke-width: 3.0; }
    .edge-cat-C { stroke: var(--connector); stroke-width: 2.2; stroke-dasharray: 6 4; }
    .edge.selected {
      stroke: var(--sel);
      stroke-width: 5.3;
      opacity: 1;
    }
    .edge-label {
      font-size: 11px;
      fill: #243149;
      paint-order: stroke;
      stroke: #ffffff;
      stroke-width: 3px;
      stroke-linejoin: round;
      pointer-events: none;
      user-select: none;
    }
    .node {
      fill: var(--node);
      stroke: #ffffff;
      stroke-width: 2px;
      cursor: pointer;
    }
    .node.depot {
      fill: var(--depot);
      stroke: #1f1f1f;
      stroke-width: 2.2px;
    }
    .node.selected {
      stroke: #101010;
      stroke-width: 3.6px;
    }
    .node-label {
      font-size: 12px;
      fill: #0f1f3f;
      font-weight: 600;
      paint-order: stroke;
      stroke: #ffffff;
      stroke-width: 3px;
      pointer-events: none;
      user-select: none;
    }
    .info {
      margin-top: 10px;
      font-size: 13px;
      color: #1f2d43;
      background: #f4f8ff;
      border: 1px solid #d9e4f8;
      border-radius: 8px;
      padding: 10px;
      min-height: 64px;
      line-height: 1.35;
      white-space: pre-wrap;
    }
    @media (max-width: 960px) {
      .wrap { grid-template-columns: 1fr; }
      svg { height: 62vh; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="panel">
      <h1 class="title">CLIPP Graph Viewer</h1>
      <p class="sub">Interactive graph with labels, colors, mandatory lines, and direction arrows.</p>
      <div id="stats"></div>
      <hr style="border:0;border-top:1px solid #e8eef8;margin:10px 0;">
      <div class="row"><input id="toggleM" type="checkbox" checked><label for="toggleM">Show Mandatory</label></div>
      <div class="row"><input id="toggleO" type="checkbox" checked><label for="toggleO">Show Optional</label></div>
      <div class="row"><input id="toggleC" type="checkbox" checked><label for="toggleC">Show Connector</label></div>
      <div class="row"><input id="toggleNodeLabels" type="checkbox" checked><label for="toggleNodeLabels">Node Labels</label></div>
      <div class="row"><input id="toggleEdgeLabels" type="checkbox" checked><label for="toggleEdgeLabels">Street Labels</label></div>
      <hr style="border:0;border-top:1px solid #e8eef8;margin:10px 0;">
      <div class="row"><span class="legend-dot mandatory"></span>Mandatory street</div>
      <div class="row"><span class="legend-dot optional"></span>Optional street</div>
      <div class="row"><span class="legend-dot connector"></span>Connector street</div>
      <div class="row"><span class="legend-dot depot"></span>Depot node</div>
      <div class="info" id="info">Click a node or street to inspect details.</div>
    </div>
    <div class="graph-card">
      <div class="graph-head" id="graphHead"></div>
      <svg id="graph" aria-label="Graph visualization">
        <defs>
          <marker id="arrow-end" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto" markerUnits="strokeWidth">
            <path d="M0,0 L8,4 L0,8 Z" fill="currentColor"></path>
          </marker>
          <marker id="arrow-start" markerWidth="8" markerHeight="8" refX="1" refY="4" orient="auto" markerUnits="strokeWidth">
            <path d="M8,0 L0,4 L8,8 Z" fill="currentColor"></path>
          </marker>
        </defs>
        <g id="edgeLayer"></g>
        <g id="edgeLabelLayer"></g>
        <g id="nodeLayer"></g>
        <g id="nodeLabelLayer"></g>
      </svg>
    </div>
  </div>
  <script>
    const data = __DATA_JSON__;
    const meta = data.meta;
    const graph = document.getElementById("graph");
    const edgeLayer = document.getElementById("edgeLayer");
    const edgeLabelLayer = document.getElementById("edgeLabelLayer");
    const nodeLayer = document.getElementById("nodeLayer");
    const nodeLabelLayer = document.getElementById("nodeLabelLayer");
    const info = document.getElementById("info");

    graph.setAttribute("viewBox", `0 0 ${meta.viewWidth} ${meta.viewHeight}`);
    document.getElementById("graphHead").textContent =
      `Nodes: ${meta.N} | Streets: ${meta.M} | Depot: ${meta.S} | Fleet: ${meta.fleet.join(", ")} | T=${meta.T}s | W=${meta.W}`;

    const stats = document.getElementById("stats");
    function statRow(label, value) {
      const row = document.createElement("div");
      row.className = "stat";
      const a = document.createElement("span");
      a.textContent = label;
      const b = document.createElement("strong");
      b.textContent = value;
      row.appendChild(a);
      row.appendChild(b);
      return row;
    }
    stats.appendChild(statRow("Junctions", meta.N));
    stats.appendChild(statRow("Streets", meta.M));
    stats.appendChild(statRow("Mandatory", meta.categoryCounts.M));
    stats.appendChild(statRow("Optional", meta.categoryCounts.O));
    stats.appendChild(statRow("Connector", meta.categoryCounts.C));
    stats.appendChild(statRow("Time Limit (T)", `${meta.T}s`));

    const nodeById = new Map();
    for (const n of data.nodes) nodeById.set(n.id, n);

    let selected = null;
    function clearSelection() {
      if (selected) selected.classList.remove("selected");
      selected = null;
    }
    function setInfo(text) {
      info.textContent = text;
    }

    const edgeEntries = [];
    for (const e of data.edges) {
      const a = nodeById.get(e.a);
      const b = nodeById.get(e.b);

      const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
      line.setAttribute("x1", a.x);
      line.setAttribute("y1", a.y);
      line.setAttribute("x2", b.x);
      line.setAttribute("y2", b.y);
      line.classList.add("edge", `edge-cat-${e.cat}`);

      if (e.dir === 1) {
        line.setAttribute("marker-end", "url(#arrow-end)");
      } else {
        line.setAttribute("marker-start", "url(#arrow-start)");
        line.setAttribute("marker-end", "url(#arrow-end)");
      }

      const lineTitle = document.createElementNS("http://www.w3.org/2000/svg", "title");
      lineTitle.textContent =
        `Street ${e.id}: ${e.a} -> ${e.b}, cat=${e.cat}, dir=${e.dir}, cost=${e.cost}s, len=${e.length}m, req=${e.req}`;
      line.appendChild(lineTitle);

      line.addEventListener("click", (ev) => {
        ev.stopPropagation();
        clearSelection();
        selected = line;
        selected.classList.add("selected");
        setInfo(
          `Street #${e.id}\\n` +
          `From ${e.a} to ${e.b}\\n` +
          `Category: ${e.cat} | Direction: ${e.dir === 1 ? "one-way" : "two-way"}\\n` +
          `Cost: ${e.cost}s | Length: ${e.length}m | Requirement: ${e.req}`
        );
      });

      edgeLayer.appendChild(line);

      const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
      const mx = (a.x + b.x) / 2;
      const my = (a.y + b.y) / 2;
      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const d = Math.hypot(dx, dy) || 1;
      const nx = -dy / d;
      const ny = dx / d;
      const labelOffset = 8 + (e.id % 5) * 2;
      text.setAttribute("x", mx + nx * labelOffset);
      text.setAttribute("y", my + ny * labelOffset);
      text.classList.add("edge-label");
      text.textContent = e.label;
      edgeLabelLayer.appendChild(text);

      edgeEntries.push({ edge: e, line, label: text });
    }

    const nodeEntries = [];
    for (const n of data.nodes) {
      const c = document.createElementNS("http://www.w3.org/2000/svg", "circle");
      c.setAttribute("cx", n.x);
      c.setAttribute("cy", n.y);
      c.setAttribute("r", n.isDepot ? 8.5 : 6.2);
      c.classList.add("node");
      if (n.isDepot) c.classList.add("depot");

      const title = document.createElementNS("http://www.w3.org/2000/svg", "title");
      title.textContent = `Node ${n.id}${n.isDepot ? " (Depot)" : ""}, degree=${n.degree}`;
      c.appendChild(title);

      c.addEventListener("click", (ev) => {
        ev.stopPropagation();
        clearSelection();
        selected = c;
        selected.classList.add("selected");
        setInfo(
          `Node ${n.id}${n.isDepot ? " (Depot)" : ""}\\n` +
          `Degree: ${n.degree}\\n` +
          `Position: (${n.x.toFixed(1)}, ${n.y.toFixed(1)})`
        );
      });

      nodeLayer.appendChild(c);

      const t = document.createElementNS("http://www.w3.org/2000/svg", "text");
      t.setAttribute("x", n.x + 8);
      t.setAttribute("y", n.y - 8);
      t.classList.add("node-label");
      t.textContent = String(n.id);
      nodeLabelLayer.appendChild(t);

      nodeEntries.push({ node: n, circle: c, label: t });
    }

    graph.addEventListener("click", () => {
      clearSelection();
      setInfo("Click a node or street to inspect details.");
    });

    const toggleM = document.getElementById("toggleM");
    const toggleO = document.getElementById("toggleO");
    const toggleC = document.getElementById("toggleC");
    const toggleNodeLabels = document.getElementById("toggleNodeLabels");
    const toggleEdgeLabels = document.getElementById("toggleEdgeLabels");

    function categoryVisible(cat) {
      if (cat === "M") return toggleM.checked;
      if (cat === "O") return toggleO.checked;
      if (cat === "C") return toggleC.checked;
      return true;
    }

    function applyFilters() {
      for (const item of edgeEntries) {
        const visible = categoryVisible(item.edge.cat);
        item.line.style.display = visible ? "" : "none";
        item.label.style.display = visible && toggleEdgeLabels.checked ? "" : "none";
      }
      for (const item of nodeEntries) {
        item.label.style.display = toggleNodeLabels.checked ? "" : "none";
      }
    }

    toggleM.addEventListener("change", applyFilters);
    toggleO.addEventListener("change", applyFilters);
    toggleC.addEventListener("change", applyFilters);
    toggleNodeLabels.addEventListener("change", applyFilters);
    toggleEdgeLabels.addEventListener("change", applyFilters);
    applyFilters();
  </script>
</body>
</html>
"""
    return html.replace("__DATA_JSON__", data_json)


def main():
    parser = argparse.ArgumentParser(
        description="Visualize CLIPP instances as an interactive HTML graph."
    )
    parser.add_argument("input_file", help="Path to CLIPP instance .txt file")
    parser.add_argument(
        "--output",
        default=None,
        help="Output HTML path (default: <input_stem>_graph.html in same folder)",
    )
    parser.add_argument("--width", type=int, default=1280, help="Graph width in px")
    parser.add_argument("--height", type=int, default=860, help="Graph height in px")
    parser.add_argument("--open", action="store_true", help="Open generated HTML in browser")
    args = parser.parse_args()

    input_path = Path(args.input_file)
    if args.output:
        output_path = Path(args.output)
    else:
        output_path = input_path.parent / f"{input_path.stem}_graph.html"

    instance = parse_instance(str(input_path))
    points = build_layout(instance, args.width, args.height)
    data = build_view_data(instance, points, args.width, args.height)
    html = build_html(data)

    output_path.write_text(html, encoding="utf-8")
    print(f"[OK] Graph view generated: {output_path}")

    if args.open:
        webbrowser.open(output_path.resolve().as_uri())


if __name__ == "__main__":
    main()
