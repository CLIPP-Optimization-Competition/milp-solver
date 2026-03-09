#!/usr/bin/env python3
"""
Generate an interactive HTML visualization for a CLIPP instance.
Optionally animate a solution showing vehicle routes.

Usage:
    python visualize_instance.py Instances/toy_version_3.txt
    python visualize_instance.py Instances/toy_version_3.txt --solution Instances/toy_version_3_solution.txt --open
"""

import argparse
import json
import math
import random
import webbrowser
from pathlib import Path


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

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
        streets.append({
            "id": j, "a": int(parts[0]), "b": int(parts[1]),
            "dir": int(parts[2]), "cost": int(parts[3]),
            "length": int(parts[4]), "cat": parts[5], "req": int(parts[6]),
        })

    vehicle_tokens = raw[idx].split()
    cap = {"S": 10, "M": 20, "L": 30}
    vehicles = [{"type": tok, "cap": cap[tok]} for tok in vehicle_tokens]

    return {
        "N": n, "M": m, "T": t_limit, "C": c, "S": depot, "W": w_penalty,
        "streets": streets, "vehicles": vehicles, "junction_coords": junction_coords,
    }


def parse_solution(filepath):
    """Parse a CLIPP solution file."""
    lines = Path(filepath).read_text(encoding="utf-8").splitlines()
    idx = 0
    c_sol = int(lines[idx].strip())
    idx += 1

    routes = []
    clean_per_vehicle = []

    for _k in range(c_sol):
        _n = int(lines[idx].strip())
        idx += 1
        route_line = lines[idx].strip() if idx < len(lines) else ""
        idx += 1
        route = [int(x) for x in route_line.split()] if route_line else []
        cleaned_line = lines[idx].strip() if idx < len(lines) else ""
        idx += 1
        cleaned = [int(x) for x in cleaned_line.split()] if cleaned_line else []
        routes.append(route)
        clean_per_vehicle.append(cleaned)

    return c_sol, routes, clean_per_vehicle


def build_solution_data(instance, routes, clean_per_vehicle):
    """Build animation segment data from a parsed solution."""
    streets = instance["streets"]
    arc_to_street = {}
    for s in streets:
        arc_to_street[(s["a"], s["b"])] = s["id"]
        if s["dir"] == 2:
            arc_to_street[(s["b"], s["a"])] = s["id"]

    vehicles_data = []
    max_time = 0

    for k in range(len(routes)):
        route = routes[k]
        cleaned_set = set(clean_per_vehicle[k])
        segments = []
        cum = 0

        for i in range(len(route) - 1):
            u, v = route[i], route[i + 1]
            sid = arc_to_street.get((u, v))
            if sid is None:
                continue
            cost = streets[sid]["cost"]
            segments.append({
                "f": u, "t": v, "s": sid,
                "c": sid in cleaned_set,
                "t0": cum, "t1": cum + cost,
            })
            cum += cost

        max_time = max(max_time, cum)
        vtype = instance["vehicles"][k]["type"] if k < len(instance["vehicles"]) else "?"
        vehicles_data.append({
            "id": k, "type": vtype,
            "segments": segments, "totalTime": cum,
        })

    return {"vehicles": vehicles_data, "maxTime": max_time}


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------

def _normalize_points(points, width, height, pad):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1e-9)
    span_y = max(max_y - min_y, 1e-9)
    sx = (width - 2 * pad) / span_x
    sy = (height - 2 * pad) / span_y
    scale = min(sx, sy)
    return [(pad + (x - min_x) * scale, pad + (y - min_y) * scale) for x, y in points]


def _force_layout(instance, width, height, seed=42, iterations=240):
    n = instance["N"]
    rng = random.Random(seed)
    pad = 60.0
    if n == 1:
        return [(width / 2.0, height / 2.0)]

    cx, cy = width / 2.0, height / 2.0
    radius = min(width, height) * 0.35
    pos = []
    for i in range(n):
        angle = 2.0 * math.pi * i / n
        pos.append([cx + radius * math.cos(angle) + rng.uniform(-8, 8),
                     cy + radius * math.sin(angle) + rng.uniform(-8, 8)])

    springs = [(s["a"], s["b"]) for s in instance["streets"]]
    area = max((width - 2 * pad) * (height - 2 * pad), 1.0)
    k = math.sqrt(area / float(n))
    temperature = min(width, height) * 0.12

    for _ in range(iterations):
        disp = [[0.0, 0.0] for _ in range(n)]
        for i in range(n):
            xi, yi = pos[i]
            for j in range(i + 1, n):
                dx = xi - pos[j][0]
                dy = yi - pos[j][1]
                dist = math.hypot(dx, dy) + 1e-9
                force = (k * k) / dist
                fx = (dx / dist) * force
                fy = (dy / dist) * force
                disp[i][0] += fx; disp[i][1] += fy
                disp[j][0] -= fx; disp[j][1] -= fy
        for u, v in springs:
            dx = pos[u][0] - pos[v][0]
            dy = pos[u][1] - pos[v][1]
            dist = math.hypot(dx, dy) + 1e-9
            force = (dist * dist) / k
            fx = (dx / dist) * force
            fy = (dy / dist) * force
            disp[u][0] -= fx; disp[u][1] -= fy
            disp[v][0] += fx; disp[v][1] += fy
        for i in range(n):
            dx, dy = disp[i]
            dl = math.hypot(dx, dy)
            if dl > 0:
                step = min(dl, temperature)
                pos[i][0] += (dx / dl) * step
                pos[i][1] += (dy / dl) * step
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
        nodes.append({"id": i, "x": round(x, 2), "y": round(y, 2),
                       "isDepot": i == depot, "degree": degree[i]})

    edges = []
    for s in streets:
        edges.append({
            "id": s["id"], "a": s["a"], "b": s["b"], "dir": s["dir"],
            "cat": s["cat"], "cost": s["cost"], "length": s["length"], "req": s["req"],
            "label": f"#{s['id']} | cost={s['cost']}s len={s['length']}m req={s['req']}",
        })

    cat_counts = {"M": 0, "O": 0, "C": 0}
    for s in streets:
        cat_counts[s["cat"]] += 1

    return {
        "meta": {
            "N": instance["N"], "M": instance["M"], "T": instance["T"],
            "C": instance["C"], "S": instance["S"], "W": instance["W"],
            "fleet": [v["type"] for v in vehicles],
            "categoryCounts": cat_counts,
            "viewWidth": width, "viewHeight": height,
        },
        "nodes": nodes, "edges": edges,
    }


# ---------------------------------------------------------------------------
# HTML Generation
# ---------------------------------------------------------------------------

def build_html(data, solution_data=None):
    data_json = json.dumps(data, separators=(",", ":"))
    sol_json = json.dumps(solution_data, separators=(",", ":")) if solution_data else "null"

    html = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>CLIPP Instance Viewer</title>
<style>
:root {
  --bg:#f5f7fb;--panel:#fff;--ink:#18212f;--muted:#4a5972;
  --mandatory:#cc2a2a;--optional:#1564c0;--connector:#7e889a;
  --node:#2d5f5d;--depot:#ff9a2f;--grid:#e6ebf3;--sel:#111;
  --clean:#22c55e;--deadhead:#f59e0b;--idle:#94a3b8;
  --tab-active:#1564c0;
}
*{box-sizing:border-box}
body{margin:0;font-family:"Segoe UI",Tahoma,Geneva,Verdana,sans-serif;
  background:radial-gradient(circle at 85% 10%,#d9e6ff 0%,transparent 44%),
    radial-gradient(circle at 10% 90%,#e9f0ff 0%,transparent 42%),var(--bg);
  color:var(--ink)}
.wrap{min-height:100vh;display:grid;grid-template-columns:320px 1fr;gap:14px;padding:14px}
.panel{background:var(--panel);border:1px solid #d8dfeb;border-radius:12px;
  padding:14px;box-shadow:0 3px 10px rgba(13,29,56,.08);overflow-y:auto;max-height:100vh}
.title{margin:0 0 8px;font-size:20px;line-height:1.1}
.sub{margin:0 0 8px;color:var(--muted);font-size:13px}
.stat{display:grid;grid-template-columns:1fr auto;gap:8px;font-size:13px;
  padding:4px 0;border-bottom:1px solid #eef2f7}
.row{display:flex;align-items:center;gap:8px;font-size:14px;margin:8px 0}
.legend-dot{width:12px;height:12px;border-radius:3px;border:1px solid rgba(0,0,0,.2)}
.mandatory{background:var(--mandatory)}.optional{background:var(--optional)}
.connector{background:var(--connector)}.depot-dot{background:var(--depot);border-radius:50%}
hr.sep{border:0;border-top:1px solid #e8eef8;margin:10px 0}
.info{margin-top:10px;font-size:13px;color:#1f2d43;background:#f4f8ff;
  border:1px solid #d9e4f8;border-radius:8px;padding:10px;min-height:64px;
  line-height:1.35;white-space:pre-wrap}
.graph-card{background:var(--panel);border:1px solid #d8dfeb;border-radius:12px;
  overflow:hidden;box-shadow:0 3px 10px rgba(13,29,56,.08);display:flex;
  flex-direction:column;position:relative}
.tab-bar{display:flex;gap:0;border-bottom:1px solid #e8eef8;background:#f9fbff}
.tab-btn{padding:9px 20px;border:none;border-bottom:3px solid transparent;
  background:transparent;color:var(--muted);cursor:pointer;font-size:13px;
  font-weight:500;transition:all .15s}
.tab-btn.active{color:var(--tab-active);border-bottom-color:var(--tab-active);font-weight:700}
.tab-btn:hover:not(.active){background:#eef3fc}
.tab-btn:disabled{opacity:.35;cursor:not-allowed}
svg{display:block;flex:1;min-height:0;
  background:linear-gradient(0deg,transparent 23px,var(--grid) 24px),
    linear-gradient(90deg,transparent 23px,var(--grid) 24px),#fff;
  background-size:24px 24px;cursor:grab}
svg.panning{cursor:grabbing}
.edge{stroke-linecap:round;opacity:.9;cursor:pointer}
.edge-cat-M{stroke:var(--mandatory);stroke-width:4.4}
.edge-cat-O{stroke:var(--optional);stroke-width:3}
.edge-cat-C{stroke:var(--connector);stroke-width:2.2;stroke-dasharray:6 4}
.edge.selected{stroke:var(--sel);stroke-width:5.3;opacity:1}
.edge-label{font-size:11px;fill:#243149;paint-order:stroke;stroke:#fff;
  stroke-width:3px;stroke-linejoin:round;pointer-events:none;user-select:none}
.node{fill:var(--node);stroke:#fff;stroke-width:2px;cursor:pointer}
.node.is-depot{fill:var(--depot);stroke:#1f1f1f;stroke-width:2.2px}
.node.selected{stroke:#101010;stroke-width:3.6px}
.node-label{font-size:12px;fill:#0f1f3f;font-weight:600;paint-order:stroke;
  stroke:#fff;stroke-width:3px;pointer-events:none;user-select:none}
svg.anim-mode .edge{opacity:.25}
svg.anim-mode .edge-label{opacity:.15}
.anim-bar{display:flex;align-items:center;gap:10px;padding:10px 14px;
  border-top:1px solid #e8eef8;background:#f9fbff;font-size:13px}
.anim-bar button{padding:5px 14px;border:1px solid #c5cfe0;border-radius:6px;
  background:var(--panel);cursor:pointer;font-size:12px;font-weight:600}
.anim-bar button:hover{background:#eef3fc}
#timeline{flex:1;height:6px;cursor:pointer;accent-color:var(--tab-active)}
.speed-sel{padding:3px 8px;border:1px solid #c5cfe0;border-radius:4px;font-size:12px}
.zoom-btns{position:absolute;top:48px;right:12px;display:flex;flex-direction:column;gap:4px;z-index:5}
.zoom-btns button{width:32px;height:32px;border:1px solid #d0d8e8;border-radius:6px;
  background:rgba(255,255,255,.92);cursor:pointer;font-size:16px;font-weight:700;
  color:var(--ink);display:flex;align-items:center;justify-content:center}
.zoom-btns button:hover{background:#e8eeff}
.vehicle-card{font-size:12px;margin:5px 0;padding:7px 9px;border-radius:8px;
  border-left:4px solid;display:flex;align-items:center;gap:8px;background:#f9fbff}
.v-dot{width:16px;height:16px;border-radius:50%;border:3px solid;flex-shrink:0}
.v-info{flex:1;line-height:1.35}
.v-name{font-weight:700}
.v-status{color:var(--muted)}
.no-sol{text-align:center;color:var(--muted);padding:40px 20px;font-size:14px}
.action-legend{display:flex;gap:14px;margin:8px 0;font-size:12px}
.action-legend span{display:flex;align-items:center;gap:4px}
.action-swatch{width:10px;height:10px;border-radius:50%;border:1px solid rgba(0,0,0,.15)}
.progress-bar{height:8px;border-radius:4px;background:#e8eef8;margin:6px 0;overflow:hidden}
.progress-fill{height:100%;border-radius:4px;background:var(--clean);transition:width .3s}
.hidden{display:none!important}
@media(max-width:960px){.wrap{grid-template-columns:1fr}svg{height:62vh}}
</style>
</head>
<body>
<div class="wrap">
  <!-- ===== SIDEBAR ===== -->
  <div class="panel">
    <h1 class="title">CLIPP Viewer</h1>
    <p class="sub" id="subtitle">Interactive graph &mdash; scroll to zoom, drag to pan.</p>
    <!-- Graph-mode sidebar -->
    <div id="sideGraph">
      <div id="stats"></div>
      <hr class="sep">
      <div class="row"><input id="toggleM" type="checkbox" checked><label for="toggleM">Mandatory</label></div>
      <div class="row"><input id="toggleO" type="checkbox" checked><label for="toggleO">Optional</label></div>
      <div class="row"><input id="toggleC" type="checkbox" checked><label for="toggleC">Connector</label></div>
      <div class="row"><input id="toggleNodeLabels" type="checkbox" checked><label for="toggleNodeLabels">Node Labels</label></div>
      <div class="row"><input id="toggleEdgeLabels" type="checkbox" checked><label for="toggleEdgeLabels">Street Labels</label></div>
      <hr class="sep">
      <div class="row"><span class="legend-dot mandatory"></span>Mandatory street</div>
      <div class="row"><span class="legend-dot optional"></span>Optional street</div>
      <div class="row"><span class="legend-dot connector"></span>Connector street</div>
      <div class="row"><span class="legend-dot depot-dot"></span>Depot node</div>
      <div class="info" id="info">Click a node or street to inspect details.</div>
    </div>
    <!-- Animation-mode sidebar -->
    <div id="sideAnim" class="hidden">
      <div class="action-legend">
        <span><span class="action-swatch" style="background:var(--clean)"></span>Cleaning</span>
        <span><span class="action-swatch" style="background:var(--deadhead)"></span>Deadheading</span>
        <span><span class="action-swatch" style="background:var(--idle)"></span>At depot</span>
      </div>
      <hr class="sep">
      <div id="vehicleList"></div>
      <hr class="sep">
      <div style="font-size:13px;color:var(--muted)">Streets cleaned</div>
      <div class="progress-bar"><div class="progress-fill" id="progressFill" style="width:0%"></div></div>
      <div style="font-size:12px;color:var(--muted)" id="progressText">0 / 0</div>
      <div class="info" id="animInfo">Press Play to start the animation.</div>
    </div>
  </div>
  <!-- ===== GRAPH CARD ===== -->
  <div class="graph-card">
    <div class="tab-bar">
      <button class="tab-btn active" data-tab="graph">Graph View</button>
      <button class="tab-btn" data-tab="anim" id="animTabBtn">Solution Animation</button>
    </div>
    <div class="zoom-btns">
      <button id="zoomIn" title="Zoom In">+</button>
      <button id="zoomOut" title="Zoom Out">&minus;</button>
      <button id="zoomReset" title="Reset Zoom" style="font-size:12px">&#8634;</button>
    </div>
    <svg id="graph" aria-label="Graph visualization">
      <defs>
        <marker id="arrow-end" markerWidth="8" markerHeight="8" refX="7" refY="4"
                orient="auto" markerUnits="strokeWidth">
          <path d="M0,0 L8,4 L0,8 Z" fill="currentColor"/>
        </marker>
        <marker id="arrow-start" markerWidth="8" markerHeight="8" refX="1" refY="4"
                orient="auto" markerUnits="strokeWidth">
          <path d="M8,0 L0,4 L8,8 Z" fill="currentColor"/>
        </marker>
      </defs>
      <g id="edgeLayer"></g>
      <g id="animEdgeOverlay"></g>
      <g id="edgeLabelLayer"></g>
      <g id="nodeLayer"></g>
      <g id="nodeLabelLayer"></g>
      <g id="vehicleLayer"></g>
    </svg>
    <div class="anim-bar hidden" id="animBar">
      <button id="playPause">&#9654; Play</button>
      <button id="resetBtn">&#8634;</button>
      <input type="range" id="timeline" min="0" max="10000" value="0" step="1">
      <span id="timeDisplay" style="min-width:100px;text-align:right">0s / 0s</span>
      <label style="white-space:nowrap">Speed:
        <select id="speedSel" class="speed-sel">
          <option value="0.25">0.25x</option>
          <option value="0.5">0.5x</option>
          <option value="1" selected>1x</option>
          <option value="2">2x</option>
          <option value="5">5x</option>
          <option value="10">10x</option>
          <option value="20">20x</option>
          <option value="50">50x</option>
        </select>
      </label>
    </div>
  </div>
</div>
<script>
// ====== DATA ======
const data = __DATA_JSON__;
const sol  = __SOLUTION_JSON__;
const meta = data.meta;
const hasSol = sol !== null;
const graph = document.getElementById("graph");

// ====== VIEWBOX / ZOOM / PAN ======
const vb = { x:0, y:0, w:meta.viewWidth, h:meta.viewHeight };
const VB_INIT = { x:0, y:0, w:meta.viewWidth, h:meta.viewHeight };
function setVB(){ graph.setAttribute("viewBox", vb.x+" "+vb.y+" "+vb.w+" "+vb.h); }
setVB();

let isPanning=false, didPan=false;
let panSt={x:0,y:0}, panVB={x:0,y:0};

graph.addEventListener("wheel", function(e){
  e.preventDefault();
  const f = e.deltaY>0?1.12:1/1.12;
  const nw=vb.w*f, nh=vb.h*f;
  if(nw/VB_INIT.w>8||nw/VB_INIT.w<0.05) return;
  const r=graph.getBoundingClientRect();
  const mx=(e.clientX-r.left)/r.width, my=(e.clientY-r.top)/r.height;
  vb.x+=(vb.w-nw)*mx; vb.y+=(vb.h-nh)*my; vb.w=nw; vb.h=nh;
  setVB();
}, {passive:false});

graph.addEventListener("mousedown", function(e){
  if(e.button!==0) return;
  isPanning=true; didPan=false;
  panSt={x:e.clientX,y:e.clientY}; panVB={x:vb.x,y:vb.y};
});
window.addEventListener("mousemove", function(e){
  if(!isPanning) return;
  const dx=e.clientX-panSt.x, dy=e.clientY-panSt.y;
  if(Math.hypot(dx,dy)>3) didPan=true;
  const r=graph.getBoundingClientRect();
  vb.x=panVB.x - dx/r.width*vb.w;
  vb.y=panVB.y - dy/r.height*vb.h;
  setVB();
  graph.classList.add("panning");
});
window.addEventListener("mouseup", function(){
  isPanning=false;
  graph.classList.remove("panning");
});

document.getElementById("zoomIn").addEventListener("click", function(){
  const f=1/1.3, nw=vb.w*f, nh=vb.h*f;
  vb.x+=(vb.w-nw)*0.5; vb.y+=(vb.h-nh)*0.5; vb.w=nw; vb.h=nh; setVB();
});
document.getElementById("zoomOut").addEventListener("click", function(){
  const f=1.3, nw=vb.w*f, nh=vb.h*f;
  vb.x+=(vb.w-nw)*0.5; vb.y+=(vb.h-nh)*0.5; vb.w=nw; vb.h=nh; setVB();
});
document.getElementById("zoomReset").addEventListener("click", function(){
  vb.x=VB_INIT.x; vb.y=VB_INIT.y; vb.w=VB_INIT.w; vb.h=VB_INIT.h; setVB();
});
graph.addEventListener("dblclick", function(){
  vb.x=VB_INIT.x; vb.y=VB_INIT.y; vb.w=VB_INIT.w; vb.h=VB_INIT.h; setVB();
});

// ====== STATS PANEL ======
const statsEl = document.getElementById("stats");
function addStat(label, value){
  const d=document.createElement("div"); d.className="stat";
  const a=document.createElement("span"); a.textContent=label;
  const b=document.createElement("strong"); b.textContent=value;
  d.appendChild(a); d.appendChild(b); statsEl.appendChild(d);
}
addStat("Junctions", meta.N); addStat("Streets", meta.M);
addStat("Mandatory", meta.categoryCounts.M);
addStat("Optional", meta.categoryCounts.O);
addStat("Connector", meta.categoryCounts.C);
addStat("Time Limit (T)", meta.T+"s");

// ====== BUILD GRAPH ======
const nodeById = new Map();
for(const n of data.nodes) nodeById.set(n.id, n);

let selected=null;
function clearSel(){ if(selected){ selected.classList.remove("selected"); selected=null; } }
function setInfo(t){ document.getElementById("info").textContent=t; }

const NS="http://www.w3.org/2000/svg";
const edgeEntries=[], edgeLabelEls={};

for(const e of data.edges){
  const a=nodeById.get(e.a), b=nodeById.get(e.b);
  const line=document.createElementNS(NS,"line");
  line.setAttribute("x1",a.x); line.setAttribute("y1",a.y);
  line.setAttribute("x2",b.x); line.setAttribute("y2",b.y);
  line.classList.add("edge","edge-cat-"+e.cat);
  if(e.dir===1) line.setAttribute("marker-end","url(#arrow-end)");
  else{ line.setAttribute("marker-start","url(#arrow-start)"); line.setAttribute("marker-end","url(#arrow-end)"); }

  line.addEventListener("click", function(ev){
    if(didPan) return; ev.stopPropagation(); clearSel();
    selected=line; selected.classList.add("selected");
    setInfo("Street #"+e.id+"\nFrom "+e.a+" to "+e.b+"\nCategory: "+e.cat+
      " | Direction: "+(e.dir===1?"one-way":"two-way")+"\nCost: "+e.cost+
      "s | Length: "+e.length+"m | Requirement: "+e.req);
  });
  document.getElementById("edgeLayer").appendChild(line);

  const txt=document.createElementNS(NS,"text");
  const mx=(a.x+b.x)/2, my=(a.y+b.y)/2;
  const dx=b.x-a.x, dy=b.y-a.y, d=Math.hypot(dx,dy)||1;
  const nx=-dy/d, ny=dx/d, off=8+(e.id%5)*2;
  txt.setAttribute("x",mx+nx*off); txt.setAttribute("y",my+ny*off);
  txt.classList.add("edge-label"); txt.textContent=e.label;
  document.getElementById("edgeLabelLayer").appendChild(txt);
  edgeLabelEls[e.id]=txt;

  edgeEntries.push({edge:e, line:line, label:txt});
}

const nodeEntries=[];
for(const n of data.nodes){
  const c=document.createElementNS(NS,"circle");
  c.setAttribute("cx",n.x); c.setAttribute("cy",n.y);
  c.setAttribute("r",n.isDepot?8.5:6.2);
  c.classList.add("node"); if(n.isDepot) c.classList.add("is-depot");

  c.addEventListener("click", function(ev){
    if(didPan) return; ev.stopPropagation(); clearSel();
    selected=c; selected.classList.add("selected");
    setInfo("Node "+n.id+(n.isDepot?" (Depot)":"")+"\nDegree: "+n.degree+
      "\nPosition: ("+n.x.toFixed(1)+", "+n.y.toFixed(1)+")");
  });
  document.getElementById("nodeLayer").appendChild(c);

  const t=document.createElementNS(NS,"text");
  t.setAttribute("x",n.x+8); t.setAttribute("y",n.y-8);
  t.classList.add("node-label"); t.textContent=String(n.id);
  document.getElementById("nodeLabelLayer").appendChild(t);

  nodeEntries.push({node:n, circle:c, label:t});
}

graph.addEventListener("click", function(){
  if(didPan) return; clearSel();
  setInfo("Click a node or street to inspect details.");
});

// ====== FILTERS ======
const toggleM=document.getElementById("toggleM"), toggleO=document.getElementById("toggleO"),
      toggleC=document.getElementById("toggleC"),
      toggleNL=document.getElementById("toggleNodeLabels"),
      toggleEL=document.getElementById("toggleEdgeLabels");
function catVis(cat){ return cat==="M"?toggleM.checked:cat==="O"?toggleO.checked:toggleC.checked; }
function applyFilters(){
  for(const it of edgeEntries){
    const vis=catVis(it.edge.cat);
    it.line.style.display=vis?"":"none";
    it.label.style.display=vis&&toggleEL.checked?"":"none";
  }
  for(const it of nodeEntries) it.label.style.display=toggleNL.checked?"":"none";
}
[toggleM,toggleO,toggleC,toggleNL,toggleEL].forEach(function(el){ el.addEventListener("change",applyFilters); });
applyFilters();

// ====== TABS ======
let activeTab="graph";
const animTabBtn=document.getElementById("animTabBtn");
if(!hasSol) animTabBtn.disabled=true;

document.querySelectorAll(".tab-btn").forEach(function(btn){
  btn.addEventListener("click", function(){
    const tab=btn.dataset.tab;
    if(tab==="anim"&&!hasSol) return;
    activeTab=tab;
    document.querySelectorAll(".tab-btn").forEach(function(b){b.classList.remove("active");});
    btn.classList.add("active");
    switchTab(tab);
  });
});

function switchTab(tab){
  const sg=document.getElementById("sideGraph"), sa=document.getElementById("sideAnim"),
        ab=document.getElementById("animBar"), vl=document.getElementById("vehicleLayer"),
        ao=document.getElementById("animEdgeOverlay");
  if(tab==="graph"){
    sg.classList.remove("hidden"); sa.classList.add("hidden");
    ab.classList.add("hidden");
    graph.classList.remove("anim-mode");
    vl.style.display="none"; ao.style.display="none";
    if(animPlaying) togglePlay();
    document.getElementById("subtitle").textContent="Interactive graph \u2014 scroll to zoom, drag to pan.";
  } else {
    sg.classList.add("hidden"); sa.classList.remove("hidden");
    ab.classList.remove("hidden");
    graph.classList.add("anim-mode");
    vl.style.display=""; ao.style.display="";
    updateAnim(simTime);
    document.getElementById("subtitle").textContent="Solution animation \u2014 watch vehicles clean streets.";
  }
}

// ====== ANIMATION ENGINE ======
const VCOLS=["#e63946","#457b9d","#2a9d8f","#e9c46a","#264653","#f4a261","#6a4c93","#1982c4"];
const ACT_COL={cleaning:"#22c55e",deadheading:"#f59e0b",depot:"#94a3b8"};

let simTime=0, animPlaying=false, lastFrame=null, animId=null;
let speed=1;

// Create edge overlays
const edgeOvEls={};
for(const e of data.edges){
  const a=nodeById.get(e.a), b=nodeById.get(e.b);
  const line=document.createElementNS(NS,"line");
  line.setAttribute("x1",a.x); line.setAttribute("y1",a.y);
  line.setAttribute("x2",b.x); line.setAttribute("y2",b.y);
  line.setAttribute("stroke-linecap","round"); line.setAttribute("stroke-width","7");
  line.setAttribute("opacity","0"); line.setAttribute("pointer-events","none");
  document.getElementById("animEdgeOverlay").appendChild(line);
  edgeOvEls[e.id]=line;
}

// Create vehicle dots
const vDots=[];
if(hasSol){
  for(const v of sol.vehicles){
    const g=document.createElementNS(NS,"g");
    const bg=document.createElementNS(NS,"circle");
    bg.setAttribute("r","13"); bg.setAttribute("fill","rgba(255,255,255,0.85)");
    bg.setAttribute("stroke","none");
    const circ=document.createElementNS(NS,"circle");
    circ.setAttribute("r","10");
    circ.setAttribute("stroke",VCOLS[v.id%VCOLS.length]);
    circ.setAttribute("stroke-width","3"); circ.setAttribute("fill",ACT_COL.depot);
    const txt=document.createElementNS(NS,"text");
    txt.setAttribute("text-anchor","middle"); txt.setAttribute("dy","4");
    txt.setAttribute("font-size","10"); txt.setAttribute("font-weight","bold");
    txt.setAttribute("fill","#fff"); txt.setAttribute("pointer-events","none");
    txt.textContent=String(v.id);
    g.appendChild(bg); g.appendChild(circ); g.appendChild(txt);
    g.style.display="none";
    document.getElementById("vehicleLayer").appendChild(g);
    vDots.push({g:g, bg:bg, circ:circ, txt:txt, v:v});
  }
}

// Vehicle status cards in sidebar
const vehicleListEl=document.getElementById("vehicleList");
const vCards=[];
if(hasSol){
  for(const v of sol.vehicles){
    const card=document.createElement("div");
    card.className="vehicle-card";
    card.style.borderLeftColor=VCOLS[v.id%VCOLS.length];
    const dot=document.createElement("div");
    dot.className="v-dot";
    dot.style.borderColor=VCOLS[v.id%VCOLS.length];
    dot.style.background=ACT_COL.depot;
    const info=document.createElement("div");
    info.className="v-info";
    info.innerHTML="<span class='v-name'>Vehicle "+v.id+" ("+v.type+")</span><br><span class='v-status'>At depot</span>";
    card.appendChild(dot); card.appendChild(info);
    vehicleListEl.appendChild(card);
    vCards.push({card:card, dot:dot, info:info, v:v});
  }
}

function getVState(v, t){
  const dp=nodeById.get(meta.S);
  if(!v.segments.length||t<=0) return {x:dp.x,y:dp.y,status:"depot",seg:null};
  const last=v.segments[v.segments.length-1];
  if(t>=last.t1) return {x:dp.x,y:dp.y,status:"depot",seg:null};
  for(const seg of v.segments){
    if(t>=seg.t0 && t<seg.t1){
      const frac=Math.max(0,Math.min(1,(t-seg.t0)/Math.max(seg.t1-seg.t0,0.001)));
      const fn=nodeById.get(seg.f), tn=nodeById.get(seg.t);
      return {x:fn.x+(tn.x-fn.x)*frac, y:fn.y+(tn.y-fn.y)*frac,
              status:seg.c?"cleaning":"deadheading", seg:seg};
    }
  }
  return {x:dp.x,y:dp.y,status:"depot",seg:null};
}

function computeEdgeStates(t){
  const st={};
  if(!hasSol) return st;
  for(const v of sol.vehicles){
    for(const seg of v.segments){
      if(seg.t1<=t){
        if(seg.c) st[seg.s]="cleaned";
        else if(st[seg.s]!=="cleaned") st[seg.s]="traversed";
      }
    }
  }
  return st;
}

function countCleaned(t){
  const st=computeEdgeStates(t);
  let c=0, total=0;
  for(const e of data.edges){ if(e.cat==="M"||e.cat==="O") total++; }
  for(const k in st){ if(st[k]==="cleaned") c++; }
  return {cleaned:c, total:total};
}

function updateAnim(t){
  if(!hasSol) return;
  // Vehicle dots
  for(const vd of vDots){
    const st=getVState(vd.v, t);
    vd.g.setAttribute("transform","translate("+st.x.toFixed(1)+","+st.y.toFixed(1)+")");
    vd.g.style.display="";
    vd.circ.setAttribute("fill", ACT_COL[st.status]);
    vd.bg.setAttribute("fill", st.status==="cleaning"?"rgba(34,197,94,0.18)":
                                st.status==="deadheading"?"rgba(245,158,11,0.15)":"rgba(148,163,184,0.12)");
  }
  // Edge overlays
  const es=computeEdgeStates(t);
  for(const eid in edgeOvEls){
    const s=es[eid], line=edgeOvEls[eid];
    if(s==="cleaned"){line.setAttribute("stroke","#22c55e");line.setAttribute("opacity","0.55");}
    else if(s==="traversed"){line.setAttribute("stroke","#94a3b8");line.setAttribute("opacity","0.25");}
    else{line.setAttribute("opacity","0");}
  }
  // Vehicle status cards
  for(const vc of vCards){
    const st=getVState(vc.v, t);
    vc.dot.style.background=ACT_COL[st.status];
    const statusTxt=st.status==="cleaning"?"Cleaning street #"+st.seg.s:
                    st.status==="deadheading"?"Deadheading via street #"+st.seg.s:
                    "At depot";
    vc.info.innerHTML="<span class='v-name'>Vehicle "+vc.v.id+" ("+vc.v.type+")</span><br><span class='v-status'>"+statusTxt+"</span>";
  }
  // Timeline
  const slider=document.getElementById("timeline");
  slider.value=Math.round(t/sol.maxTime*10000)||0;
  document.getElementById("timeDisplay").textContent=Math.round(t)+"s / "+sol.maxTime+"s";
  // Progress
  const p=countCleaned(t);
  const pct=p.total?Math.round(p.cleaned/p.total*100):0;
  document.getElementById("progressFill").style.width=pct+"%";
  document.getElementById("progressText").textContent=p.cleaned+" / "+p.total+" streets ("+pct+"%)";
  // Anim info
  document.getElementById("animInfo").textContent=
    t<=0?"Press Play to start the animation.":
    t>=sol.maxTime?"Animation complete. All vehicles returned to depot.":
    "Simulation running at "+speed+"x speed...";
}

function animLoop(ts){
  if(!animPlaying) return;
  if(lastFrame===null) lastFrame=ts;
  const dt=(ts-lastFrame)/1000*speed;
  lastFrame=ts;
  simTime=Math.min(simTime+dt, sol.maxTime);
  updateAnim(simTime);
  if(simTime>=sol.maxTime){
    animPlaying=false;
    document.getElementById("playPause").innerHTML="&#9654; Play";
    return;
  }
  animId=requestAnimationFrame(animLoop);
}

function togglePlay(){
  if(!hasSol) return;
  if(animPlaying){
    animPlaying=false; lastFrame=null;
    document.getElementById("playPause").innerHTML="&#9654; Play";
    cancelAnimationFrame(animId);
  } else {
    if(simTime>=sol.maxTime) simTime=0;
    animPlaying=true; lastFrame=null;
    document.getElementById("playPause").innerHTML="&#9646;&#9646; Pause";
    animId=requestAnimationFrame(animLoop);
  }
}

function resetAnim(){
  animPlaying=false; simTime=0; lastFrame=null;
  document.getElementById("playPause").innerHTML="&#9654; Play";
  cancelAnimationFrame(animId);
  updateAnim(0);
}

document.getElementById("playPause").addEventListener("click", togglePlay);
document.getElementById("resetBtn").addEventListener("click", resetAnim);
document.getElementById("speedSel").addEventListener("change", function(){
  speed=parseFloat(this.value);
});
document.getElementById("timeline").addEventListener("input", function(){
  if(!hasSol) return;
  const t=parseFloat(this.value)/10000*sol.maxTime;
  simTime=t;
  if(!animPlaying) updateAnim(t);
});

// Keyboard shortcuts
document.addEventListener("keydown", function(e){
  if(activeTab!=="anim"||!hasSol) return;
  if(e.code==="Space"){ e.preventDefault(); togglePlay(); }
  if(e.code==="KeyR"){ resetAnim(); }
  if(e.code==="ArrowRight"){
    simTime=Math.min(simTime+sol.maxTime*0.02, sol.maxTime);
    if(!animPlaying) updateAnim(simTime);
  }
  if(e.code==="ArrowLeft"){
    simTime=Math.max(simTime-sol.maxTime*0.02, 0);
    if(!animPlaying) updateAnim(simTime);
  }
});

// Initialize animation overlay as hidden
document.getElementById("vehicleLayer").style.display="none";
document.getElementById("animEdgeOverlay").style.display="none";
document.getElementById("animBar").classList.add("hidden");

// If no solution, show message in anim sidebar
if(!hasSol){
  document.getElementById("sideAnim").innerHTML=
    "<div class='no-sol'><p style='font-size:28px;margin:0 0 8px'>&#128722;</p>"+
    "<p>No solution file loaded.</p>"+
    "<p style='font-size:12px'>Use <code>--solution &lt;file&gt;</code> to animate a solution.</p></div>";
}
</script>
</body>
</html>
"""
    return html.replace("__DATA_JSON__", data_json).replace("__SOLUTION_JSON__", sol_json)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Visualize CLIPP instances as an interactive HTML graph, optionally with animated solution."
    )
    parser.add_argument("input_file", help="Path to CLIPP instance .txt file")
    parser.add_argument(
        "--solution", default=None,
        help="Path to solution .txt file (enables animation tab)",
    )
    parser.add_argument(
        "--output", default=None,
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

    solution_data = None
    if args.solution:
        _, routes, clean_per_vehicle = parse_solution(args.solution)
        solution_data = build_solution_data(instance, routes, clean_per_vehicle)

    points = build_layout(instance, args.width, args.height)
    view_data = build_view_data(instance, points, args.width, args.height)
    html = build_html(view_data, solution_data)

    output_path.write_text(html, encoding="utf-8")
    print(f"[OK] Graph view generated: {output_path}")

    if args.open:
        webbrowser.open(output_path.resolve().as_uri())


if __name__ == "__main__":
    main()
