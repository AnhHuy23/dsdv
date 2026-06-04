"""WSN + DSDV environment simulator.

This script simulates a simplified DSDV routing environment using a discrete-time
loop (`dt`) rather than an event-driven simulator such as simpy.

Usage example:

    python dsdv_env_sim.py --nodes 30 --duration 300 --dt 1.0 --seed 7
    python dsdv_env_sim.py --nodes 30 --placement uniform   # legacy spread; clustered pins gateway at hub
    python dsdv_env_sim.py --compare-three --nodes 30       # Baseline / +Backbone-Leaf / +Gradient → subfolders + metrics_chart each
    python dsdv_env_sim.py --list-scenarios                 # Print preset S1..S12 (Vietnamese labels)
    python dsdv_env_sim.py --scenario S4                    # One preset → <out-dir>/scenarios/S4/ (override --scenarios-dir)
    python dsdv_env_sim.py --all-scenarios                  # Run S1..S12 sequentially into separate folders

Outputs are written under `<out-dir>/n{N}/` and may include CSV summaries,
PNG plots (``topology.png``, ``metrics_chart.png``, ``metrics_chart_bars.png``), and an optional GIF/MP4 animation.

Dependencies:
    - matplotlib for plots/animation
    - pillow for GIF output (via matplotlib's PillowWriter)
    - ffmpeg for MP4 output (via matplotlib's FFMpegWriter)
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import random
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Deque, Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Tuple


INF_HOPS = 255
DEFAULT_OUT_DIR = Path("sim_outputs")
DEFAULT_AREA_W = 18.0
DEFAULT_AREA_H = 12.0
DEFAULT_GATEWAY_ID = -1
DEFAULT_MAX_DEGREE = 8
DEFAULT_EXTRA_EDGE_FACTOR = 0.8
DEFAULT_MAX_ATTEMPTS = 50
DEFAULT_RSSI_1M = -59.0
DEFAULT_PATH_LOSS_N = 1.9
DEFAULT_RSSI_THRESHOLD = -84.0
DEFAULT_NOISE_RSSI_AMP = 0.0
DEFAULT_HELLO_PERIOD = 12.0
DEFAULT_UPDATE_PERIOD = 12.0
DEFAULT_ROUTE_TIMEOUT = 180.0
DEFAULT_UPDATE_PAYLOAD_LIMIT = 32
DEFAULT_DATA_PERIOD = 0.38
DEFAULT_DATA_START_DELAY_S = 45.0
DEFAULT_INITIAL_ENERGY = 100.0
DEFAULT_ENERGY_PER_TX = 0.45
DEFAULT_ENERGY_PER_RX = 0.18
DEFAULT_FPS = 6
DEFAULT_VIDEO_STRIDE = 1
DEFAULT_VIDEO_MAX_FRAMES = 2000
DEFAULT_NODE_PLACEMENT = "clustered"
DEFAULT_CLUSTER_STD_FRAC = 0.22
HELLO_KEEPALIVE_PERIODS = 16
ROUTE_SETTLE_TIME_S = 45.0
UPDATE_MIN_INTERVAL_S = 3.0
LEAF_UPDATE_SUPPRESS_BACKOFF = 4.0
NEIGHBOR_RSSI_VALID_WINDOW_S = 90.0
HELLO_RSSI_MARGIN_NEW = 1.0
HELLO_RSSI_MARGIN_EXISTING = 4.0
BACKBONE_RSSI_THRESHOLD = -70.0
BACKBONE_RSSI_REJECT = -80.0
BACKBONE_MIN_DEGREE = 3
BACKBONE_PROMOTE_MIN_DEGREE = 2
BACKBONE_PROMOTE_RSSI_THRESHOLD = -80.0
BACKBONE_EVAL_INTERVAL_S = 30.0
BACKBONE_INITIAL_DELAY_S = 15.0
BACKBONE_PDR_REJECT = 50.0
BACKBONE_PDR_WINDOW_S = 60.0
BACKBONE_EXPECTED_HELLO_S = 12.0
RT_PROPAGATION_SPEED_MPS = 250000.0
RECV_RSSI_JITTER_DB = 2.0

ROLE_UNKNOWN = 0
ROLE_BACKBONE = 1
ROLE_LEAF = 2
BACKBONE_FORWARD_ONLY = True
HIGH_RELIABILITY_TRAFFIC = True
MAX_FORWARD_RETRIES = 3
FORWARD_RETRY_DELAY_S = 1.0
ACK_WAIT_TIMEOUT_S = 4.0
TRAFFIC_SOURCE_POOL = 24
TRAFFIC_SINK_POOL = 4
TRAFFIC_MAX_HOPS = 3
TRAFFIC_BURST_SIZE = 4

# (internal profile key, output subdirectory under n<N>/)
SIM_PROFILE_SEQUENCE: Tuple[Tuple[str, str], ...] = (
    ("baseline", "baseline_dsdv"),
    ("backbone_leaf", "dsdv_backbone_leaf"),
    ("full", "dsdv_backbone_leaf_gradient"),
)
SIM_PROFILE_LABELS = {
    "baseline": "Baseline DSDV",
    "backbone_leaf": "DSDV + Backbone-Leaf",
    "full": "DSDV + Backbone-Leaf + Gradient",
}

# Preset evaluation scenarios (thesis / benchmark S1..S12).
# Keys match argparse attribute names. ``description_vi`` is metadata only (written to scenario_spec.json).
# Load: larger ``data_period`` => lower traffic. Link quality: more negative ``rssi_threshold`` => larger
# nominal hop range (``dmax``); higher ``noise_rssi_amp`` / ``path_loss_n`` => harsher channel.
SCENARIO_ORDER: Tuple[str, ...] = (
    "S1",
    "S2",
    "S3",
    "S4",
    "S5",
    "S6",
    "S7",
    "S8",
    "S9",
    "S10",
    "S11",
    "S12",
)

SCENARIO_PRESETS: Dict[str, Dict[str, Any]] = {
    "S1": {
        "description_vi": "10 node, uniform, tải thấp, liên kết tốt",
        "nodes": 10,
        "placement": "uniform",
        "data_period": 1.35,
        "rssi_threshold": -93.0,
        "noise_rssi_amp": 0.0,
        "path_loss_n": 1.78,
        "area_w": 18.0,
        "area_h": 12.0,
        "cluster_std_frac": 0.22,
        "max_attempts": 80,
    },
    "S2": {
        "description_vi": "10 node, uniform, tải trung bình, liên kết tốt",
        "nodes": 10,
        "placement": "uniform",
        "data_period": 0.38,
        "rssi_threshold": -93.0,
        "noise_rssi_amp": 0.0,
        "path_loss_n": 1.78,
        "area_w": 18.0,
        "area_h": 12.0,
        "cluster_std_frac": 0.22,
        "max_attempts": 80,
    },
    "S3": {
        "description_vi": "10 node, clustered, tải cao, liên kết trung bình",
        "nodes": 10,
        "placement": "clustered",
        "data_period": 0.12,
        "rssi_threshold": -84.0,
        "noise_rssi_amp": 0.0,
        "path_loss_n": 1.9,
        "area_w": 18.0,
        "area_h": 12.0,
        "cluster_std_frac": 0.2,
        "max_attempts": 80,
    },
    "S4": {
        "description_vi": "30 node, uniform, tải thấp, liên kết tốt",
        "nodes": 30,
        "placement": "uniform",
        "data_period": 1.35,
        "rssi_threshold": -93.0,
        "noise_rssi_amp": 0.0,
        "path_loss_n": 1.78,
        "area_w": 18.0,
        "area_h": 12.0,
        "cluster_std_frac": 0.22,
        "max_attempts": 80,
    },
    "S5": {
        "description_vi": "30 node, uniform, tải cao, liên kết trung bình",
        "nodes": 30,
        "placement": "uniform",
        "data_period": 0.12,
        "rssi_threshold": -84.0,
        "noise_rssi_amp": 0.0,
        "path_loss_n": 1.9,
        "area_w": 18.0,
        "area_h": 12.0,
        "cluster_std_frac": 0.22,
        "max_attempts": 80,
    },
    "S6": {
        "description_vi": "30 node, clustered, tải cao, liên kết trung bình",
        "nodes": 30,
        "placement": "clustered",
        "data_period": 0.12,
        "rssi_threshold": -84.0,
        "noise_rssi_amp": 0.0,
        "path_loss_n": 1.9,
        "area_w": 18.0,
        "area_h": 12.0,
        "cluster_std_frac": 0.22,
        "max_attempts": 80,
    },
    "S7": {
        "description_vi": "50 node, uniform, tải trung bình, liên kết trung bình",
        "nodes": 50,
        "placement": "uniform",
        "data_period": 0.38,
        "rssi_threshold": -84.0,
        "noise_rssi_amp": 0.0,
        "path_loss_n": 1.9,
        "area_w": 22.0,
        "area_h": 15.0,
        "cluster_std_frac": 0.22,
        "max_attempts": 100,
        "max_degree": 10,
        "extra_edge_factor": 0.95,
    },
    "S8": {
        "description_vi": "50 node, clustered, tải cao, liên kết trung bình",
        "nodes": 50,
        "placement": "clustered",
        "data_period": 0.12,
        "rssi_threshold": -84.0,
        "noise_rssi_amp": 0.0,
        "path_loss_n": 1.9,
        "area_w": 22.0,
        "area_h": 15.0,
        "cluster_std_frac": 0.22,
        "max_attempts": 100,
        "max_degree": 10,
        "extra_edge_factor": 0.95,
    },
    "S9": {
        "description_vi": "50 node, uniform, tải trung bình, liên kết yếu",
        "nodes": 50,
        "placement": "uniform",
        "data_period": 0.38,
        "rssi_threshold": -79.0,
        "noise_rssi_amp": 1.8,
        "path_loss_n": 2.05,
        "area_w": 22.0,
        "area_h": 15.0,
        "cluster_std_frac": 0.22,
        "max_attempts": 120,
        "max_degree": 11,
        "extra_edge_factor": 1.0,
    },
    "S10": {
        "description_vi": "50 node, clustered, tải cao, liên kết yếu",
        "nodes": 50,
        "placement": "clustered",
        "data_period": 0.12,
        "rssi_threshold": -79.0,
        "noise_rssi_amp": 2.0,
        "path_loss_n": 2.08,
        "area_w": 22.0,
        "area_h": 15.0,
        "cluster_std_frac": 0.22,
        "max_attempts": 120,
        "max_degree": 11,
        "extra_edge_factor": 1.0,
    },
    "S11": {
        "description_vi": "100 node, uniform, tải cao, liên kết trung bình",
        "nodes": 100,
        "placement": "uniform",
        "data_period": 0.12,
        "rssi_threshold": -84.0,
        "noise_rssi_amp": 0.0,
        "path_loss_n": 1.9,
        "area_w": 32.0,
        "area_h": 22.0,
        "cluster_std_frac": 0.22,
        "max_attempts": 120,
        "max_degree": 12,
        "extra_edge_factor": 1.0,
    },
    "S12": {
        "description_vi": "100 node, clustered, tải cao, liên kết yếu (cụm dày, vùng nhỏ hơn S10)",
        "nodes": 100,
        "placement": "clustered",
        "data_period": 0.12,
        "rssi_threshold": -78.5,
        "noise_rssi_amp": 2.4,
        "path_loss_n": 2.12,
        "area_w": 24.0,
        "area_h": 16.0,
        "cluster_std_frac": 0.16,
        "max_attempts": 180,
        "max_degree": 12,
        "extra_edge_factor": 1.05,
    },
}


def _apply_scenario_to_args(base: argparse.Namespace, scenario_id: str) -> argparse.Namespace:
    """Return a copy of ``base`` with preset fields for ``scenario_id`` (sets ``scenario_id`` / description)."""
    if scenario_id not in SCENARIO_PRESETS:
        raise ValueError(f"unknown scenario {scenario_id!r}; expected one of {SCENARIO_ORDER}")
    preset = SCENARIO_PRESETS[scenario_id]
    out = copy.deepcopy(base)
    out.scenario_id = scenario_id
    out.scenario_description_vi = str(preset.get("description_vi", ""))
    for key, value in preset.items():
        if key == "description_vi":
            continue
        setattr(out, key, value)
    # Square ``--area`` would override ``area_w``/``area_h`` in :func:`_make_sim`; presets use explicit W×H.
    out.area = None
    return out


def _lazy_import_matplotlib():
    try:
        import matplotlib.pyplot as plt  # type: ignore
        from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter  # type: ignore
    except Exception as exc:  # pragma: no cover - dependency path
        return None, None, None, exc
    return plt, FuncAnimation, (FFMpegWriter, PillowWriter), None


def _role_color(role: int) -> str:
    if role == ROLE_BACKBONE:
        return "#d62728"
    if role == ROLE_LEAF:
        return "#1f77b4"
    return "#7f7f7f"


def plot_topology_png(
    positions: Sequence[Tuple[float, float]],
    adjacency: Mapping[int, Sequence[int]],
    gateway_id: int,
    roles: Optional[Sequence[int]],
    gradient_leaf_nodes: Optional[Sequence[int]],
    out_path: Path,
    title: str = "WSN topology",
    sim_profile: Optional[str] = None,
) -> Optional[Path]:
    plt, _, _, err = _lazy_import_matplotlib()
    if err is not None:
        print(f"[WARN] matplotlib not available, skipping topology plot: {err}")
        return None

    fig, ax = plt.subplots(figsize=(10, 7), dpi=150)
    for src, neighbors in adjacency.items():
        x0, y0 = positions[src]
        for dst in neighbors:
            if dst <= src:
                continue
            x1, y1 = positions[dst]
            ax.plot([x0, x1], [y0, y1], color="#8aa0b8", linewidth=0.8, alpha=0.55)

    gradient_leaf_set = set(gradient_leaf_nodes or [])
    is_baseline = (sim_profile == "baseline")
    if is_baseline:
        # Baseline DSDV is a flat proactive routing protocol with no backbone/leaf hierarchy.
        # Internally every node is marked ROLE_BACKBONE for relaying eligibility; show as a single node class instead.
        groups: Dict[str, Dict[str, Any]] = {
            "node": {"color": "#1f77b4", "label": "Node (Baseline DSDV)", "x": [], "y": []},
        }
        for idx, (x, y) in enumerate(positions):
            if idx == gateway_id:
                continue
            groups["node"]["x"].append(x)
            groups["node"]["y"].append(y)
    else:
        groups = {
            "backbone": {"color": "#d62728", "label": "Backbone", "x": [], "y": []},
            "leaf_t1": {"color": "#1f77b4", "label": "Leaf (kề backbone)", "x": [], "y": []},
            "leaf_grad": {"color": "#ff7f0e", "label": "Leaf qua Gradient", "x": [], "y": []},
            "unknown": {"color": "#7f7f7f", "label": "Chưa phân loại", "x": [], "y": []},
        }
        for idx, (x, y) in enumerate(positions):
            if idx == gateway_id:
                continue
            role = roles[idx] if roles is not None else ROLE_UNKNOWN
            if idx in gradient_leaf_set:
                key = "leaf_grad"
            elif role == ROLE_BACKBONE:
                key = "backbone"
            elif role == ROLE_LEAF:
                key = "leaf_t1"
            else:
                key = "unknown"
            groups[key]["x"].append(x)
            groups[key]["y"].append(y)

    for key, info in groups.items():
        if not info["x"]:
            continue
        ax.scatter(
            info["x"], info["y"],
            s=64, c=info["color"], edgecolors="white", linewidths=0.7,
            zorder=3, label=info["label"],
        )

    gw_x, gw_y = positions[gateway_id]
    ax.scatter([gw_x], [gw_y], s=220, c="#ffd400", marker="*",
               edgecolors="#8a2200", linewidths=1.2, zorder=4, label=f"Gateway (id={gateway_id})")

    for idx, (x, y) in enumerate(positions):
        ax.annotate(
            str(idx),
            xy=(x, y),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=8,
            color="#222222",
            ha="left",
            va="bottom",
            zorder=5,
        )

    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best", fontsize=9, framealpha=0.92)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def plot_metrics_chart(samples: Sequence[Mapping[str, Any]], out_path: Path, title: str = "Simulation metrics") -> Optional[Path]:
    plt, _, _, err = _lazy_import_matplotlib()
    if err is not None:
        print(f"[WARN] matplotlib not available, skipping metrics chart: {err}")
        return None

    if not samples:
        return None

    t = [float(row["t"]) for row in samples]
    pdr = [float(row["pdr"]) for row in samples]
    latency = [float(row["avg_latency_s"]) for row in samples]
    hops = [float(row["avg_hops"]) for row in samples]
    hello = [float(row["hello_packets"]) for row in samples]
    update = [float(row["update_packets"]) for row in samples]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), dpi=150)
    ax = axes[0, 0]
    ax.plot(t, pdr, color="#2ca02c")
    ax.set_title("PDR (%)")
    ax.set_ylabel("%")
    ax.grid(True, alpha=0.2)

    ax = axes[0, 1]
    ax.plot(t, latency, color="#9467bd")
    ax.set_title("Average latency (s)")
    ax.grid(True, alpha=0.2)

    ax = axes[1, 0]
    ax.plot(t, hops, color="#ff7f0e")
    ax.set_title("Average hop count")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.2)

    ax = axes[1, 1]
    ax.plot(t, hello, label="HELLO", color="#1f77b4")
    ax.plot(t, update, label="UPDATE", color="#d62728")
    ax.set_title("Cumulative control packets")
    ax.set_xlabel("Time (s)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.2)

    fig.suptitle(title)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def _bin_metrics_by_time(
    samples: Sequence[Mapping[str, Any]],
    n_bins: int,
) -> Tuple[
    List[float],
    List[float],
    List[float],
    List[float],
    List[float],
    List[float],
]:
    """Aggregate samples into ``n_bins`` equal-width time buckets.

    PDR, latency, and hops use the **mean** inside each bucket. HELLO and UPDATE
    use the **last** value in the bucket (cumulative counters are monotone).
    """
    t = [float(row["t"]) for row in samples]
    pdr = [float(row["pdr"]) for row in samples]
    latency = [float(row["avg_latency_s"]) for row in samples]
    hops = [float(row["avg_hops"]) for row in samples]
    hello = [float(row["hello_packets"]) for row in samples]
    update = [float(row["update_packets"]) for row in samples]
    n = len(t)
    n_bins = max(1, min(n_bins, n))
    t0, t1 = t[0], t[-1]
    span = max(t1 - t0, 1e-12)

    buckets_pdr: List[List[float]] = [[] for _ in range(n_bins)]
    buckets_lat: List[List[float]] = [[] for _ in range(n_bins)]
    buckets_hops: List[List[float]] = [[] for _ in range(n_bins)]
    buckets_hello: List[List[float]] = [[] for _ in range(n_bins)]
    buckets_upd: List[List[float]] = [[] for _ in range(n_bins)]

    for i in range(n):
        ti = t[i]
        b = int((ti - t0) / span * n_bins) if span > 0 else 0
        if b >= n_bins:
            b = n_bins - 1
        buckets_pdr[b].append(pdr[i])
        buckets_lat[b].append(latency[i])
        buckets_hops[b].append(hops[i])
        buckets_hello[b].append(hello[i])
        buckets_upd[b].append(update[i])

    def _mean(vals: List[float]) -> float:
        return sum(vals) / len(vals) if vals else 0.0

    def _last(vals: List[float]) -> float:
        return float(vals[-1]) if vals else 0.0

    centers: List[float] = []
    out_pdr: List[float] = []
    out_lat: List[float] = []
    out_hops: List[float] = []
    out_hello: List[float] = []
    out_update: List[float] = []
    for b in range(n_bins):
        lo = t0 + (b / n_bins) * span
        hi = t0 + ((b + 1) / n_bins) * span
        centers.append(0.5 * (lo + hi))
        out_pdr.append(_mean(buckets_pdr[b]))
        out_lat.append(_mean(buckets_lat[b]))
        out_hops.append(_mean(buckets_hops[b]))
        out_hello.append(_last(buckets_hello[b]))
        out_update.append(_last(buckets_upd[b]))

    return centers, out_pdr, out_lat, out_hops, out_hello, out_update


def _sparse_time_xticks(tc: Sequence[float], max_labels: int = 12) -> Tuple[List[int], List[str]]:
    """Pick bar indices and short time strings so x-axis stays readable."""
    n = len(tc)
    if n <= 0:
        return [], []
    if n <= max_labels:
        ticks = list(range(n))
    else:
        # Evenly sample ~max_labels positions, always include first and last bar.
        step = max(1, (n - 1) // (max_labels - 1))
        ticks = list(range(0, n, step))
        if ticks[-1] != n - 1:
            ticks.append(n - 1)
    span = float(tc[-1] - tc[0]) if n > 1 else 0.0
    fmt = ".0f" if span > 90.0 else ".1f"
    labels = [format(float(tc[i]), fmt) for i in ticks]
    return ticks, labels


def plot_metrics_bar_chart(
    samples: Sequence[Mapping[str, Any]],
    out_path: Path,
    title: str = "Chỉ số mô phỏng (biểu đồ cột)",
    max_bins: int = 56,
) -> Optional[Path]:
    """Same four panels as :func:`plot_metrics_chart`, using column (bar) charts.

    Time axis is split into at most ``max_bins`` equal-width intervals; each bar
    summarizes samples in that interval (mean for rates, end-of-interval value
    for cumulative HELLO/UPDATE).
    """
    plt, _, _, err = _lazy_import_matplotlib()
    if err is not None:
        print(f"[WARN] matplotlib not available, skipping metrics bar chart: {err}")
        return None

    if not samples:
        return None

    n = len(samples)
    n_bins = min(max_bins, max(1, n))
    tc, pdr, latency, hops, hello, update = _bin_metrics_by_time(samples, n_bins)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5), dpi=150)
    x = list(range(len(tc)))
    tick_idx, tick_lbl = _sparse_time_xticks(tc, max_labels=12)

    def _style_time_xaxis(ax: Any) -> None:
        ax.set_xticks(tick_idx)
        ax.set_xticklabels(tick_lbl, rotation=30, ha="right", fontsize=9)
        ax.set_xlabel("Thời gian mô phỏng (s)")
        ax.margins(x=0.01)
        ax.tick_params(axis="x", pad=2)

    ax = axes[0, 0]
    ax.bar(x, pdr, width=0.85, color="#2ca02c", edgecolor="#1a6b1a", linewidth=0.4, alpha=0.88)
    ax.set_title("PDR (%)")
    ax.set_ylabel("%")
    _style_time_xaxis(ax)
    ax.grid(True, axis="y", alpha=0.2)

    ax = axes[0, 1]
    ax.bar(x, latency, width=0.85, color="#9467bd", edgecolor="#5c3d7a", linewidth=0.4, alpha=0.88)
    ax.set_title("Độ trễ trung bình (s)")
    _style_time_xaxis(ax)
    ax.grid(True, axis="y", alpha=0.2)

    ax = axes[1, 0]
    ax.bar(x, hops, width=0.85, color="#ff7f0e", edgecolor="#b35900", linewidth=0.4, alpha=0.88)
    ax.set_title("Hop trung bình")
    ax.set_ylabel("Số hop")
    _style_time_xaxis(ax)
    ax.grid(True, axis="y", alpha=0.2)

    ax = axes[1, 1]
    w = 0.38
    x0 = [xi - w / 2 for xi in x]
    x1 = [xi + w / 2 for xi in x]
    ax.bar(x0, hello, width=w, label="HELLO (lũy kế)", color="#1f77b4", edgecolor="#0d4a73", linewidth=0.35, alpha=0.9)
    ax.bar(x1, update, width=w, label="UPDATE (lũy kế)", color="#d62728", edgecolor="#8b1a1a", linewidth=0.35, alpha=0.9)
    ax.set_title("HELLO và UPDATE (lũy kế)")
    ax.set_ylabel("Gói tin (lũy kế)")
    _style_time_xaxis(ax)
    ax.legend(loc="best")
    ax.grid(True, axis="y", alpha=0.2)

    fig.suptitle(title)
    fig.tight_layout(rect=[0.02, 0.11, 0.98, 0.94], h_pad=1.9, w_pad=1.15)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def plot_compare_control_overhead(summaries: Sequence[Mapping[str, Any]], out_path: Path) -> Optional[Path]:
    """Grouped control-overhead bars for compare-three runs."""
    if not summaries:
        return None

    labels = [SIM_PROFILE_LABELS.get(str(row.get("sim_profile", "")), str(row.get("sim_profile", ""))) for row in summaries]
    hello = [float(row.get("hello_packets", 0.0)) for row in summaries]
    update = [float(row.get("update_packets", 0.0)) for row in summaries]
    total = [float(row.get("total_control_packets", 0.0)) for row in summaries]

    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception as err:  # pragma: no cover
        print(f"[WARN] matplotlib not available, skipping control overhead chart: {err}")
        return _plot_compare_control_overhead_pillow(labels, hello, update, total, out_path)

    x = list(range(len(summaries)))
    w = 0.26
    fig, ax = plt.subplots(figsize=(10.5, 5.6), dpi=160)
    bars_hello = ax.bar([i - w for i in x], hello, width=w, label="HELLO", color="#2f80ed", edgecolor="#1f5faa", linewidth=0.5)
    bars_update = ax.bar(x, update, width=w, label="UPDATE", color="#d64545", edgecolor="#963030", linewidth=0.5)
    bars_total = ax.bar([i + w for i in x], total, width=w, label="Total control", color="#333333", edgecolor="#111111", linewidth=0.5)

    ax.set_title("Control overhead comparison")
    ax.set_ylabel("Packets")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=10, ha="right")
    ax.grid(True, axis="y", alpha=0.22)
    ax.legend(loc="upper right", ncol=3, fontsize=8)
    for bars in (bars_hello, bars_update, bars_total):
        ax.bar_label(bars, fmt="%.0f", padding=2, fontsize=8)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def _plot_compare_control_overhead_pillow(
    labels: Sequence[str],
    hello: Sequence[float],
    update: Sequence[float],
    total: Sequence[float],
    out_path: Path,
) -> Optional[Path]:
    try:
        from PIL import Image, ImageDraw, ImageFont  # type: ignore
    except Exception as err:  # pragma: no cover
        print(f"[WARN] pillow not available, skipping control overhead chart: {err}")
        return None

    width, height = 1280, 720
    margin_l, margin_r, margin_t, margin_b = 110, 50, 80, 165
    plot_w = width - margin_l - margin_r
    plot_h = height - margin_t - margin_b
    ymax = max([1.0, *hello, *update, *total])
    step = max(100.0, math.ceil(ymax / 5.0 / 100.0) * 100.0)
    ytop = math.ceil(ymax / step) * step

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    def sx(group: int, offset: float) -> float:
        group_w = plot_w / max(1, len(labels))
        return margin_l + group_w * group + group_w * 0.5 + offset

    def sy(value: float) -> float:
        return margin_t + plot_h - (value / ytop) * plot_h

    draw.text((margin_l, 30), "Control overhead comparison", fill="#111111", font=font)
    draw.line((margin_l, margin_t, margin_l, margin_t + plot_h), fill="#222222", width=2)
    draw.line((margin_l, margin_t + plot_h, margin_l + plot_w, margin_t + plot_h), fill="#222222", width=2)

    tick = 0.0
    while tick <= ytop + 1e-9:
        y = sy(tick)
        draw.line((margin_l, y, margin_l + plot_w, y), fill="#e3e3e3", width=1)
        draw.text((25, y - 6), f"{int(tick)}", fill="#333333", font=font)
        tick += step

    series = (
        ("HELLO", hello, "#2f80ed"),
        ("UPDATE", update, "#d64545"),
        ("Total control", total, "#333333"),
    )
    bar_w = min(70, plot_w / max(1, len(labels)) * 0.18)
    offsets = (-bar_w * 1.15, 0.0, bar_w * 1.15)
    for group_idx, label in enumerate(labels):
        for (name, values, color), offset in zip(series, offsets):
            value = float(values[group_idx])
            x = sx(group_idx, offset)
            y = sy(value)
            draw.rectangle((x - bar_w / 2, y, x + bar_w / 2, margin_t + plot_h), fill=color)
            draw.text((x - bar_w / 2, y - 18), f"{int(value)}", fill="#111111", font=font)
        wrapped = label.replace(" + ", "\n+ ")
        draw.multiline_text((sx(group_idx, 0.0) - 95, margin_t + plot_h + 20), wrapped, fill="#111111", font=font, align="center", spacing=4)

    legend_x = margin_l + plot_w - 360
    for i, (name, _values, color) in enumerate(series):
        x = legend_x + i * 125
        draw.rectangle((x, 48, x + 18, 66), fill=color)
        draw.text((x + 24, 51), name, fill="#111111", font=font)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(out_path)
    return out_path


def export_all_scenario_control_summary(rows: Sequence[Mapping[str, Any]], out_path: Path) -> Path:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "scenario_id",
        "sim_profile",
        "hello_packets",
        "update_packets",
        "total_control_packets",
        "pdr",
        "avg_latency_s",
        "avg_hops",
        "route_changes",
        "backbone_nodes",
        "leaf_nodes",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            summary = row.get("summary", {})
            if not isinstance(summary, Mapping):
                summary = {}
            writer.writerow(
                {
                    "scenario_id": row.get("scenario_id", ""),
                    "sim_profile": summary.get("sim_profile", row.get("sim_profile", "")),
                    "hello_packets": summary.get("hello_packets", 0),
                    "update_packets": summary.get("update_packets", 0),
                    "total_control_packets": summary.get("total_control_packets", 0),
                    "pdr": summary.get("pdr", 0.0),
                    "avg_latency_s": summary.get("avg_latency_s", 0.0),
                    "avg_hops": summary.get("avg_hops", 0.0),
                    "route_changes": summary.get("route_changes", 0),
                    "backbone_nodes": summary.get("backbone_nodes", 0),
                    "leaf_nodes": summary.get("leaf_nodes", 0),
                }
            )
    return out_path


def plot_all_scenarios_control_metric(
    rows: Sequence[Mapping[str, Any]],
    out_path: Path,
    metric: str,
    title: str,
    ylabel: str = "Packets",
) -> Optional[Path]:
    try:
        from PIL import Image, ImageDraw, ImageFont  # type: ignore
    except Exception as err:  # pragma: no cover
        print(f"[WARN] pillow not available, skipping all-scenario chart: {err}")
        return None

    scenario_ids = [sid for sid in SCENARIO_ORDER if any(row.get("scenario_id") == sid for row in rows)]
    profiles = [key for key, _ in SIM_PROFILE_SEQUENCE]
    values: Dict[Tuple[str, str], float] = {}
    for row in rows:
        sid = str(row.get("scenario_id", ""))
        summary = row.get("summary", {})
        if not isinstance(summary, Mapping):
            continue
        profile = str(summary.get("sim_profile", row.get("sim_profile", "")))
        values[(sid, profile)] = float(summary.get(metric, 0.0))

    if not scenario_ids:
        return None

    width, height = 1680, 820
    margin_l, margin_r, margin_t, margin_b = 115, 50, 90, 130
    plot_w = width - margin_l - margin_r
    plot_h = height - margin_t - margin_b
    ymax = max([1.0, *values.values()])
    step = max(100.0, math.ceil(ymax / 5.0 / 100.0) * 100.0)
    ytop = math.ceil(ymax / step) * step

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    profile_labels = {
        "baseline": "Baseline",
        "backbone_leaf": "Backbone-Leaf",
        "full": "Backbone-Leaf + Gradient",
    }
    colors = {
        "baseline": "#2f80ed",
        "backbone_leaf": "#d64545",
        "full": "#333333",
    }

    def sy(value: float) -> float:
        return margin_t + plot_h - (value / ytop) * plot_h

    draw.text((margin_l, 34), title, fill="#111111", font=font)
    draw.text((20, 38), ylabel, fill="#333333", font=font)
    draw.line((margin_l, margin_t, margin_l, margin_t + plot_h), fill="#222222", width=2)
    draw.line((margin_l, margin_t + plot_h, margin_l + plot_w, margin_t + plot_h), fill="#222222", width=2)

    tick = 0.0
    while tick <= ytop + 1e-9:
        y = sy(tick)
        draw.line((margin_l, y, margin_l + plot_w, y), fill="#e3e3e3", width=1)
        draw.text((32, y - 6), f"{int(tick)}", fill="#333333", font=font)
        tick += step

    group_w = plot_w / len(scenario_ids)
    bar_w = min(32.0, group_w * 0.2)
    offsets = (-bar_w * 1.2, 0.0, bar_w * 1.2)
    for idx, sid in enumerate(scenario_ids):
        center = margin_l + group_w * idx + group_w * 0.5
        for profile, offset in zip(profiles, offsets):
            value = values.get((sid, profile), 0.0)
            x = center + offset
            y = sy(value)
            draw.rectangle((x - bar_w / 2, y, x + bar_w / 2, margin_t + plot_h), fill=colors[profile])
            if len(scenario_ids) <= 12:
                draw.text((x - bar_w / 2, y - 15), f"{int(value)}", fill="#111111", font=font)
        draw.text((center - 9, margin_t + plot_h + 18), sid, fill="#111111", font=font)

    legend_x = margin_l + plot_w - 520
    for i, profile in enumerate(profiles):
        x = legend_x + i * 170
        draw.rectangle((x, 58, x + 18, 76), fill=colors[profile])
        draw.text((x + 24, 61), profile_labels[profile], fill="#111111", font=font)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(out_path)
    return out_path


def save_topology_animation(
    frames: Sequence[Mapping[str, Any]],
    positions: Sequence[Tuple[float, float]],
    adjacency: Mapping[int, Sequence[int]],
    gateway_id: int,
    roles: Optional[Sequence[int]],
    out_path: Path,
    fps: int = DEFAULT_FPS,
    title: str = "Topology energy animation",
) -> Optional[Path]:
    plt, FuncAnimation, writer_types, err = _lazy_import_matplotlib()
    if err is not None:
        print(f"[WARN] matplotlib not available, skipping animation: {err}")
        return None

    if not frames:
        return None

    FFMpegWriter, PillowWriter = writer_types
    fig, ax = plt.subplots(figsize=(10, 7), dpi=150)

    xs = [pos[0] for pos in positions]
    ys = [pos[1] for pos in positions]
    node_colors = ["#1f77b4" for _ in positions] if roles is None else [_role_color(role) for role in roles]
    scatter = ax.scatter(xs, ys, s=58, c=node_colors, edgecolors="white", linewidths=0.7, zorder=3)
    gateway_point = ax.scatter([positions[gateway_id][0]], [positions[gateway_id][1]], s=140, c="#d62728",
                               marker="*", edgecolors="white", linewidths=1.0, zorder=4)
    labels = []
    for idx, (x, y) in enumerate(positions):
        labels.append(
            ax.annotate(
                str(idx),
                xy=(x, y),
                xytext=(4, 4),
                textcoords="offset points",
                fontsize=8,
                color="#222222",
                ha="left",
                va="bottom",
                zorder=5,
            )
        )

    edge_lines = []
    for src, neighbors in adjacency.items():
        x0, y0 = positions[src]
        for dst in neighbors:
            if dst <= src:
                continue
            x1, y1 = positions[dst]
            line, = ax.plot([x0, x1], [y0, y1], color="#8aa0b8", linewidth=0.8, alpha=0.45)
            edge_lines.append(line)

    title_artist = ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)

    def _update(frame: Mapping[str, Any]):
        energy = frame["energy"]
        norm = [max(0.0, min(1.0, float(value))) for value in energy]
        scatter.set_array(norm)
        title_artist.set_text(f"{title} | t={frame['t']:.1f}s")
        return [scatter, gateway_point, title_artist, *labels, *edge_lines]

    anim = FuncAnimation(fig, _update, frames=frames, interval=max(1, int(1000 / max(1, fps))), blit=False)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    suffix = out_path.suffix.lower()

    try:
        if suffix == ".mp4":
            anim.save(out_path, writer=FFMpegWriter(fps=fps))
        elif suffix == ".gif":
            anim.save(out_path, writer=PillowWriter(fps=fps))
        else:
            anim.save(out_path.with_suffix(".mp4"), writer=FFMpegWriter(fps=fps))
            out_path = out_path.with_suffix(".mp4")
    except Exception as mp4_error:
        fallback = out_path.with_suffix(".gif")
        try:
            anim.save(fallback, writer=PillowWriter(fps=fps))
            print(f"[WARN] MP4 export failed, wrote GIF instead: {mp4_error}")
            out_path = fallback
        except Exception as gif_error:  # pragma: no cover - dependency path
            print(f"[WARN] animation export failed: {gif_error}")
            plt.close(fig)
            return None

    plt.close(fig)
    return out_path


def _rssi_from_dist(dist_m: float, rssi_1m: float, path_loss_n: float, noise_rssi_amp: float = 0.0, rng: Optional[random.Random] = None) -> float:
    dist_m = max(1.0, float(dist_m))
    rssi = rssi_1m - 10.0 * path_loss_n * math.log10(dist_m)
    if noise_rssi_amp > 0.0:
        rnd = rng or random
        rssi += rnd.uniform(-noise_rssi_amp, noise_rssi_amp)
    return rssi


def _max_distance_from_rssi_threshold(rssi_1m: float, path_loss_n: float, rssi_threshold: float, noise_rssi_amp: float = 0.0) -> float:
    effective_threshold = rssi_threshold - abs(noise_rssi_amp)
    if path_loss_n <= 0:
        raise ValueError("path_loss_n must be positive")
    exponent = (rssi_1m - effective_threshold) / (10.0 * path_loss_n)
    return 10.0 ** exponent


def generate_positions(
    node_count: int,
    area_w: float,
    area_h: float,
    rng: random.Random,
    *,
    placement: str = DEFAULT_NODE_PLACEMENT,
    cluster_std_frac: float = DEFAULT_CLUSTER_STD_FRAC,
) -> Tuple[List[Tuple[float, float]], Optional[Tuple[float, float]]]:
    """Place nodes in the deployment rectangle.

    ``uniform``: independent uniform samples (legacy, spread across the area).
    ``clustered``: 2D Gaussian cloud around one hub, clipped to the rectangle.
    Returns ``(positions, hub)`` where ``hub`` is ``(cx, cy)`` for clustered mode
    (caller should pin the gateway on/near this hub); ``hub`` is ``None`` for uniform.

    The hub is kept away from edges by a few sigma so clipping does not scatter
    many nodes along the border away from the dense core.
    """
    mode = (placement or DEFAULT_NODE_PLACEMENT).strip().lower()
    if mode == "uniform":
        return (
            [(rng.uniform(0.0, area_w), rng.uniform(0.0, area_h)) for _ in range(node_count)],
            None,
        )
    if mode != "clustered":
        raise ValueError(f"unknown placement mode {placement!r}; use 'clustered' or 'uniform'")

    span = min(float(area_w), float(area_h))
    sigma = max(1e-6, float(cluster_std_frac) * span)
    inset = min(3.0 * sigma, 0.45 * float(area_w), 0.45 * float(area_h))
    cx = rng.uniform(inset, max(inset, area_w - inset))
    cy = rng.uniform(inset, max(inset, area_h - inset))
    out: List[Tuple[float, float]] = []
    for _ in range(node_count):
        x = cx + rng.gauss(0.0, sigma)
        y = cy + rng.gauss(0.0, sigma)
        x = max(0.0, min(area_w, x))
        y = max(0.0, min(area_h, y))
        out.append((x, y))
    return out, (cx, cy)


def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def constrained_connectivity_graph(
    positions: Sequence[Tuple[float, float]],
    gateway_id: int,
    dmax: float,
    max_degree: int,
    extra_edge_factor: float,
    rng: random.Random,
) -> Optional[Dict[int, set[int]]]:
    node_count = len(positions)
    adjacency: Dict[int, set[int]] = {idx: set() for idx in range(node_count)}
    degree = [0 for _ in range(node_count)]
    unconnected = set(range(node_count))
    connected = {gateway_id}
    unconnected.remove(gateway_id)

    candidate_edges: List[Tuple[float, int, int]] = []
    for i in range(node_count):
        for j in range(i + 1, node_count):
            distance = _dist(positions[i], positions[j])
            if distance <= dmax:
                candidate_edges.append((distance, i, j))
    candidate_edges.sort(key=lambda item: item[0])

    while unconnected:
        best: Optional[Tuple[float, int, int]] = None
        for distance, i, j in candidate_edges:
            if i in connected and j in unconnected and degree[i] < max_degree and degree[j] < max_degree:
                best = (distance, i, j)
                break
            if j in connected and i in unconnected and degree[i] < max_degree and degree[j] < max_degree:
                best = (distance, j, i)
                break
        if best is None:
            return None
        _, src, dst = best
        adjacency[src].add(dst)
        adjacency[dst].add(src)
        degree[src] += 1
        degree[dst] += 1
        connected.add(dst)
        unconnected.remove(dst)

    extra_target = max(0, int(round(extra_edge_factor * node_count)))
    # Prefer short extra edges so the rendered topology matches intuitive local connectivity.
    extra_candidates = sorted(candidate_edges, key=lambda item: item[0])
    added = 0
    for _, i, j in extra_candidates:
        if added >= extra_target:
            break
        if j in adjacency[i]:
            continue
        if degree[i] >= max_degree or degree[j] >= max_degree:
            continue
        adjacency[i].add(j)
        adjacency[j].add(i)
        degree[i] += 1
        degree[j] += 1
        added += 1

    return adjacency


def generate_random_topology(
    node_count: int,
    area_w: float,
    area_h: float,
    gateway_id: int,
    max_degree: int,
    extra_edge_factor: float,
    max_attempts: int,
    rssi_1m: float,
    path_loss_n: float,
    rssi_threshold: float,
    noise_rssi_amp: float,
    seed: int,
    placement: str = DEFAULT_NODE_PLACEMENT,
    cluster_std_frac: float = DEFAULT_CLUSTER_STD_FRAC,
) -> Tuple[List[Tuple[float, float]], Dict[int, set[int]], float, int]:
    dmax = _max_distance_from_rssi_threshold(rssi_1m, path_loss_n, rssi_threshold, noise_rssi_amp)
    last_error: Optional[str] = None
    for attempt in range(max_attempts):
        rng = random.Random(seed + attempt * 9973)
        positions, cluster_hub = generate_positions(
            node_count,
            area_w,
            area_h,
            rng,
            placement=placement,
            cluster_std_frac=cluster_std_frac,
        )
        root_id = gateway_id
        if cluster_hub is not None:
            # Pin gateway at the cluster hub so the dense region surrounds the sink.
            hub_x, hub_y = cluster_hub
            if 0 <= gateway_id < node_count:
                root_id = gateway_id
            else:
                root_id = min(range(node_count), key=lambda idx: _dist(positions[idx], cluster_hub))
            jitter = max(0.02, 0.02 * min(area_w, area_h))
            gx = max(0.0, min(area_w, hub_x + rng.uniform(-jitter, jitter)))
            gy = max(0.0, min(area_h, hub_y + rng.uniform(-jitter, jitter)))
            positions[root_id] = (gx, gy)
        elif root_id < 0 or root_id >= node_count:
            center_x = area_w / 2.0
            center_y = area_h / 2.0
            root_id = min(range(node_count), key=lambda idx: _dist(positions[idx], (center_x, center_y)))
        graph = constrained_connectivity_graph(positions, root_id, dmax, max_degree, extra_edge_factor, rng)
        if graph is not None:
            return positions, graph, dmax, root_id
        last_error = (
            f"failed attempt {attempt + 1}/{max_attempts} with dmax={dmax:.2f}; "
            f"try increasing --rssi-threshold, --area, --max-degree, or --max-attempts"
        )
    raise RuntimeError(last_error or "unable to generate topology")


@dataclass
class RouteEntry:
    dest: int
    next_hop: int
    hop_count: int
    seq_num: int
    last_update: float
    changed: bool = True


@dataclass
class Message:
    kind: str
    src: int
    sender: int
    dst: Optional[int]
    payload: Dict[str, Any]
    deliver_time: float


@dataclass
class BackbonePeerInfo:
    addr: int
    degree: int = 0
    role: int = ROLE_UNKNOWN
    avg_rssi: float = -127.0
    last_seen: float = 0.0
    hello_rx_count: int = 0
    pdr_window_start: float = 0.0


@dataclass
class Node:
    node_id: int
    x: float
    y: float
    routes: Dict[int, RouteEntry] = field(default_factory=dict)
    neighbor_rssi: Dict[int, float] = field(default_factory=dict)
    neighbor_last_seen: Dict[int, float] = field(default_factory=dict)
    backbone_peers: Dict[int, BackbonePeerInfo] = field(default_factory=dict)
    next_hello: float = 0.0
    next_update: float = 0.0
    next_data: float = 0.0
    next_backbone_eval: float = 0.0
    energy: float = DEFAULT_INITIAL_ENERGY
    hello_seq: int = 0
    data_seq: int = 0
    hello_tx_count: int = 0
    role: int = ROLE_UNKNOWN
    backbone_score: int = 0
    degree: int = 0
    route_changed: bool = False
    my_info_changed: bool = False
    tx_count: int = 0
    rx_count: int = 0
    dropped_count: int = 0
    delivered_count: int = 0
    dup_cache: Dict[int, int] = field(default_factory=dict)
    pending_data: Deque[Dict[str, Any]] = field(default_factory=deque)
    pending_ack_seq: Optional[int] = None
    pending_ack_deadline: float = 0.0
    attached_backbone: Optional[int] = None
    gradient_level: Optional[int] = None
    gradient_next_hop: Optional[int] = None
    gradient_anchor_leaf: Optional[int] = None


class DsdvEnvSim:
    def __init__(
        self,
        *,
        nodes: int,
        duration: float,
        dt: float,
        seed: int,
        out_dir: Path,
        focus_node: int,
        area: Optional[float] = None,
        area_w: float = DEFAULT_AREA_W,
        area_h: float = DEFAULT_AREA_H,
        gateway_id: int = DEFAULT_GATEWAY_ID,
        max_degree: int = DEFAULT_MAX_DEGREE,
        extra_edge_factor: float = DEFAULT_EXTRA_EDGE_FACTOR,
        max_attempts: int = DEFAULT_MAX_ATTEMPTS,
        rssi_1m: float = DEFAULT_RSSI_1M,
        path_loss_n: float = DEFAULT_PATH_LOSS_N,
        rssi_threshold: float = DEFAULT_RSSI_THRESHOLD,
        noise_rssi_amp: float = DEFAULT_NOISE_RSSI_AMP,
        node_placement: str = DEFAULT_NODE_PLACEMENT,
        cluster_std_frac: float = DEFAULT_CLUSTER_STD_FRAC,
        hello: float = DEFAULT_HELLO_PERIOD,
        update: float = DEFAULT_UPDATE_PERIOD,
        timeout: float = DEFAULT_ROUTE_TIMEOUT,
        update_payload_limit: int = DEFAULT_UPDATE_PAYLOAD_LIMIT,
        data_period: float = DEFAULT_DATA_PERIOD,
        data_start_delay: float = DEFAULT_DATA_START_DELAY_S,
        initial_energy: float = DEFAULT_INITIAL_ENERGY,
        energy_per_tx: float = DEFAULT_ENERGY_PER_TX,
        energy_per_rx: float = DEFAULT_ENERGY_PER_RX,
        no_plots: bool = False,
        video: bool = False,
        video_format: str = "mp4",
        fps: int = DEFAULT_FPS,
        video_stride: int = DEFAULT_VIDEO_STRIDE,
        video_max_frames: int = DEFAULT_VIDEO_MAX_FRAMES,
        backbone_forward_only: bool = BACKBONE_FORWARD_ONLY,
        sim_profile: str = "full",
    ) -> None:
        self.node_count = int(nodes)
        self.duration = float(duration)
        self.dt = float(dt)
        self.seed = int(seed)
        self.out_dir = Path(out_dir)
        self.focus_node = int(focus_node)
        self.gateway_id = int(gateway_id)
        self.area_w = float(area if area is not None else area_w)
        self.area_h = float(area if area is not None else area_h)
        self.max_degree = int(max_degree)
        self.extra_edge_factor = float(extra_edge_factor)
        self.max_attempts = int(max_attempts)
        self.rssi_1m = float(rssi_1m)
        self.path_loss_n = float(path_loss_n)
        self.rssi_threshold = float(rssi_threshold)
        self.noise_rssi_amp = float(noise_rssi_amp)
        self.node_placement = str(node_placement)
        self.cluster_std_frac = float(cluster_std_frac)
        self.hello_period = float(hello)
        self.update_period = float(update)
        self.route_timeout = float(timeout)
        self.update_payload_limit = int(update_payload_limit)
        self.data_period = float(data_period)
        self.data_start_delay = max(0.0, float(data_start_delay))
        self.initial_energy = float(initial_energy)
        self.energy_per_tx = float(energy_per_tx)
        self.energy_per_rx = float(energy_per_rx)
        self.no_plots = bool(no_plots)
        self.video = bool(video)
        self.video_format = video_format.lower().lstrip(".")
        self.fps = int(fps)
        self.video_stride = max(1, int(video_stride))
        self.video_max_frames = int(video_max_frames)
        self.sim_profile = str(sim_profile)
        if self.sim_profile not in ("baseline", "backbone_leaf", "full"):
            raise ValueError("sim_profile must be 'baseline', 'backbone_leaf', or 'full'")
        # Baseline DSDV: flat proactive DSDV (no role hierarchy, no gradient, any hop may relay data/ACK).
        self.hierarchy_enabled = self.sim_profile != "baseline"
        self.gradient_enabled = self.sim_profile == "full"
        self.backbone_forward_only = bool(backbone_forward_only)
        if self.sim_profile == "baseline":
            self.backbone_forward_only = False
        self.rng = random.Random(self.seed)
        self.positions, self.adjacency, self.dmax, self.gateway_id = generate_random_topology(
            self.node_count,
            self.area_w,
            self.area_h,
            self.gateway_id,
            self.max_degree,
            self.extra_edge_factor,
            self.max_attempts,
            self.rssi_1m,
            self.path_loss_n,
            self.rssi_threshold,
            self.noise_rssi_amp,
            self.seed,
            placement=self.node_placement,
            cluster_std_frac=self.cluster_std_frac,
        )
        if not (0 <= self.focus_node < self.node_count):
            self.focus_node = self.gateway_id

        self.time = 0.0
        self.nodes: List[Node] = []
        for node_id, (x, y) in enumerate(self.positions):
            node = Node(node_id=node_id, x=x, y=y, energy=self.initial_energy)
            node.routes[node_id] = RouteEntry(dest=node_id, next_hop=node_id, hop_count=0, seq_num=0, last_update=0.0, changed=False)
            node.next_hello = self._initial_schedule(self.hello_period)
            node.next_update = self._initial_schedule(self.update_period)
            node.next_data = self.data_start_delay + self._initial_schedule(self.data_period) if self.data_period > 0 else math.inf
            node.next_backbone_eval = BACKBONE_INITIAL_DELAY_S + self.rng.uniform(0.0, 5.0)
            self.nodes.append(node)

        if self.sim_profile == "baseline":
            for n in self.nodes:
                n.role = ROLE_BACKBONE
                n.attached_backbone = None
                n.gradient_level = None
                n.gradient_next_hop = None
                n.gradient_anchor_leaf = None

        self.in_flight: Deque[Message] = deque()
        self.samples: List[Dict[str, Any]] = []
        self.video_frames: List[Dict[str, Any]] = []
        self.counters: Dict[str, int] = {
            "hello_packets": 0,
            "update_packets": 0,
            "data_packets": 0,
            "data_delivered": 0,
            "data_dropped": 0,
            "control_dropped": 0,
            "route_changes": 0,
            "acks_delivered": 0,
        }
        self.latency_samples: List[float] = []
        self.hop_samples: List[int] = []
        self._last_update_sent: Dict[int, float] = {node_id: -math.inf for node_id in range(self.node_count)}
        self.next_global_data: float = self.data_start_delay + self._initial_schedule(self.data_period) if self.data_period > 0 else math.inf

    def _initial_schedule(self, base: float) -> float:
        if base <= 0:
            return math.inf
        return self.rng.uniform(0.4 * base, 1.0 * base)

    def _active_neighbors(self, node: Node) -> List[int]:
        return [
            neighbor_id
            for neighbor_id, last_seen in node.neighbor_last_seen.items()
            if self.time - last_seen < NEIGHBOR_RSSI_VALID_WINDOW_S
        ]

    def _calc_avg_neighbor_rssi(self, node: Node) -> float:
        values = [node.neighbor_rssi[neighbor_id] for neighbor_id in self._active_neighbors(node) if neighbor_id in node.neighbor_rssi]
        if not values:
            return -127.0
        return sum(values) / len(values)

    def _calc_avg_neighbor_pdr(self, node: Node) -> float:
        total = 0.0
        count = 0
        for info in node.backbone_peers.values():
            if self.time - info.last_seen > NEIGHBOR_RSSI_VALID_WINDOW_S:
                continue
            window = self.time - info.pdr_window_start
            if window < 5.0:
                total += 100.0
                count += 1
                continue
            expected = max(1.0, window / BACKBONE_EXPECTED_HELLO_S)
            pdr = min(100.0, (info.hello_rx_count * 100.0) / expected)
            total += pdr
            count += 1
        return total / count if count else 0.0

    def _calc_backbone_score(self, degree: int, avg_rssi: float, pdr: float) -> int:
        rssi_score = int(max(0.0, min(70.0, avg_rssi + 100.0)))
        pdr_bonus = int(pdr * 30.0 / 100.0)
        return degree * 100 + rssi_score + pdr_bonus

    def _select_leaf_backbone(self, node: Node) -> Optional[int]:
        candidates: List[Tuple[int, float, int, int]] = []
        for peer in node.backbone_peers.values():
            if peer.role != ROLE_BACKBONE:
                continue
            if self.time - peer.last_seen > NEIGHBOR_RSSI_VALID_WINDOW_S:
                continue
            if peer.addr not in self.adjacency[node.node_id]:
                continue
            link_rssi = float(node.neighbor_rssi.get(peer.addr, peer.avg_rssi))
            if link_rssi < BACKBONE_RSSI_REJECT:
                continue
            candidates.append((peer.addr, link_rssi, int(peer.degree), int(peer.hello_rx_count)))

        # Fallback: if HELLO peer cache is stale/missing, still allow direct adjacent backbone neighbors.
        for nb in sorted(self.adjacency[node.node_id]):
            if self.nodes[nb].role != ROLE_BACKBONE:
                continue
            if any(existing[0] == nb for existing in candidates):
                continue
            link_rssi = float(node.neighbor_rssi.get(nb, self._link_rssi(node.node_id, nb)))
            if link_rssi < BACKBONE_RSSI_REJECT:
                continue
            candidates.append((nb, link_rssi, len(self.adjacency[nb]), 0))

        if not candidates:
            return None

        candidates.sort(key=lambda item: (-item[1], -item[2], -item[3], item[0]))
        best_id, best_rssi, _, _ = candidates[0]

        # Keep current parent unless a new one is clearly better, reducing parent flaps.
        if node.attached_backbone is not None:
            for cand_id, cand_rssi, _, _ in candidates:
                if cand_id == node.attached_backbone:
                    if (best_rssi - cand_rssi) < 3.0:
                        return node.attached_backbone
                    break
        return best_id

    def _enforce_leaf_single_parent_routes(self, node: Node) -> None:
        if node.role != ROLE_LEAF:
            return

        parent = node.attached_backbone
        if parent is not None and (parent < 0 or parent >= self.node_count or self.nodes[parent].role != ROLE_BACKBONE):
            node.attached_backbone = None
            parent = None
        drop_dests: List[int] = []
        for dest, entry in node.routes.items():
            if dest == node.node_id:
                continue
            if parent is None:
                drop_dests.append(dest)
                continue
            if entry.next_hop != parent:
                drop_dests.append(dest)
                continue
            if entry.hop_count == 1 and dest != parent:
                drop_dests.append(dest)

        for dest in drop_dests:
            node.routes.pop(dest, None)

        if parent is not None and parent in self._active_neighbors(node):
            parent_entry = node.routes.get(parent)
            if parent_entry is None or parent_entry.hop_count >= INF_HOPS:
                seq_num = int(parent_entry.seq_num) if parent_entry is not None else 0
                node.routes[parent] = RouteEntry(parent, parent, 1, seq_num, self.time, True)
                node.route_changed = True
                self.counters["route_changes"] += 1

    def _is_valid_leaf_parent(self, node: Node, parent: Optional[int]) -> bool:
        if parent is None:
            return False
        if not (0 <= parent < self.node_count):
            return False
        if self.nodes[parent].role != ROLE_BACKBONE:
            return False
        if parent not in self.adjacency[node.node_id]:
            return False
        return True

    def _has_backbone_neighbor(self, node: Node) -> bool:
        for nb in self.adjacency[node.node_id]:
            if self.nodes[nb].role == ROLE_BACKBONE:
                return True
        return False

    def _recompute_leaf_gradients(self) -> None:
        for node in self.nodes:
            node.gradient_level = None
            node.gradient_next_hop = None
            node.gradient_anchor_leaf = None
        if not self.gradient_enabled:
            return

        queue: Deque[int] = deque()
        for node in self.nodes:
            if node.role != ROLE_LEAF:
                continue
            # Type-1 leaf: has direct backbone neighbor (no gradient forwarding).
            if not self._has_backbone_neighbor(node):
                continue
            node.gradient_level = 0
            node.gradient_anchor_leaf = node.node_id
            queue.append(node.node_id)

        while queue:
            cur_id = queue.popleft()
            cur = self.nodes[cur_id]
            cur_level = int(cur.gradient_level) if cur.gradient_level is not None else 0
            anchor = cur.gradient_anchor_leaf
            for nb in self.adjacency[cur_id]:
                peer = self.nodes[nb]
                if peer.role != ROLE_LEAF:
                    continue
                if self._has_backbone_neighbor(peer):
                    if peer.gradient_level is None:
                        peer.gradient_level = 0
                        peer.gradient_anchor_leaf = peer.node_id
                    continue
                if peer.gradient_level is not None and peer.gradient_level <= cur_level + 1:
                    continue
                peer.gradient_level = cur_level + 1
                peer.gradient_next_hop = cur_id
                peer.gradient_anchor_leaf = anchor
                queue.append(nb)

    def _refresh_leaf_parents(self) -> None:
        for node in self.nodes:
            if node.role != ROLE_LEAF:
                node.attached_backbone = None
                continue
            prev_parent = node.attached_backbone
            node.attached_backbone = self._select_leaf_backbone(node) if self._has_backbone_neighbor(node) else None
            if prev_parent != node.attached_backbone:
                node.my_info_changed = True

    def _gradient_next_hop(self, node: Node) -> Optional[int]:
        if node.role != ROLE_LEAF:
            return None
        if self._has_backbone_neighbor(node):
            return None
        next_hop = node.gradient_next_hop
        if next_hop is None:
            return None
        if next_hop not in self.adjacency[node.node_id]:
            return None
        me_level = node.gradient_level
        peer_level = self.nodes[next_hop].gradient_level
        if me_level is None or peer_level is None:
            return None
        if peer_level >= me_level:
            return None
        return next_hop

    def _next_hop_for_dest(self, node: Node, dest: int) -> Optional[int]:
        if dest in self.adjacency[node.node_id]:
            return dest
        route = self._nearest_route(node, dest)
        if route is not None:
            return route.next_hop
        if node.role == ROLE_LEAF:
            grad_hop = self._gradient_next_hop(node)
            if grad_hop is not None:
                return grad_hop
            parent = node.attached_backbone
            if self._is_valid_leaf_parent(node, parent):
                return parent
        return None

    def _update_backbone_peer(self, node: Node, neighbor_id: int, degree: int, role: int, avg_rssi: float) -> None:
        info = node.backbone_peers.get(neighbor_id)
        now = self.time
        if info is None:
            info = BackbonePeerInfo(addr=neighbor_id, degree=degree, role=role, avg_rssi=avg_rssi, last_seen=now, hello_rx_count=1, pdr_window_start=now)
            node.backbone_peers[neighbor_id] = info
            return
        info.degree = degree
        info.role = role
        info.avg_rssi = avg_rssi
        info.last_seen = now
        if now - info.pdr_window_start > BACKBONE_PDR_WINDOW_S:
            info.hello_rx_count = 1
            info.pdr_window_start = now
        else:
            info.hello_rx_count += 1

    def _evaluate_backbone(self, node: Node) -> None:
        old_role = node.role
        old_parent = node.attached_backbone
        degree = len(self._active_neighbors(node))
        node.degree = degree
        avg_rssi = self._calc_avg_neighbor_rssi(node)
        avg_pdr = self._calc_avg_neighbor_pdr(node)

        # Full-case role partitioning:
        # 1) Hard reject => LEAF.
        # 2) Strong winner => BACKBONE.
        # 3) Otherwise => LEAF (single-parent policy).
        # 4) If isolated from any backbone, constrained promote to BACKBONE.
        if degree == 0:
            node.role = ROLE_LEAF
            node.backbone_score = 0
            node.attached_backbone = None
        elif degree < 2 or avg_rssi < BACKBONE_RSSI_REJECT or avg_pdr < BACKBONE_PDR_REJECT:
            node.role = ROLE_LEAF
            node.backbone_score = 0
            node.attached_backbone = self._select_leaf_backbone(node)
        else:
            my_score = self._calc_backbone_score(degree, avg_rssi, avg_pdr)
            node.backbone_score = my_score
            i_am_highest = True
            for peer in node.backbone_peers.values():
                if self.time - peer.last_seen > NEIGHBOR_RSSI_VALID_WINDOW_S:
                    continue
                peer_pdr = 100.0
                window = self.time - peer.pdr_window_start
                if window > 5.0:
                    expected = max(1.0, window / BACKBONE_EXPECTED_HELLO_S)
                    peer_pdr = min(100.0, (peer.hello_rx_count * 100.0) / expected)
                peer_score = self._calc_backbone_score(peer.degree, peer.avg_rssi, peer_pdr)
                if peer_score > my_score or (peer_score == my_score and peer.addr < node.node_id):
                    i_am_highest = False
                    break

            if (
                i_am_highest
                and degree >= BACKBONE_MIN_DEGREE
                and avg_rssi >= BACKBONE_RSSI_THRESHOLD
                and avg_pdr >= BACKBONE_PDR_REJECT
            ):
                node.role = ROLE_BACKBONE
            else:
                node.role = ROLE_LEAF

            if node.role == ROLE_LEAF:
                node.attached_backbone = self._select_leaf_backbone(node)
                if (
                    node.attached_backbone is None
                    and degree >= BACKBONE_PROMOTE_MIN_DEGREE
                    and avg_rssi >= BACKBONE_PROMOTE_RSSI_THRESHOLD
                    and avg_pdr >= BACKBONE_PDR_REJECT
                ):
                    node.role = ROLE_BACKBONE
            if node.role == ROLE_BACKBONE:
                node.attached_backbone = None

        if node.role == ROLE_LEAF:
            # Strict policy: a leaf keeps at most one backbone uplink and no leaf->leaf chain.
            self._enforce_leaf_single_parent_routes(node)

        if node.role != old_role or node.attached_backbone != old_parent:
            node.my_info_changed = True

    def _ttl_cap(self, node: Node, kind: str) -> int:
        if kind == "UPDATE":
            return 4 if node.role == ROLE_BACKBONE else 2
        if kind == "BROADCAST":
            return 3 if node.role == ROLE_BACKBONE else 1
        return 10

    def _route_timeout_for(self, node: Node, entry: RouteEntry) -> float:
        is_direct_neighbor = entry.dest == entry.next_hop and entry.hop_count == 1
        return min(self.route_timeout, max(self.hello_period * 4.0, self.route_timeout if not is_direct_neighbor else self.hello_period * 4.0)) if is_direct_neighbor else self.route_timeout

    def _link_rssi(self, src: int, dst: int) -> float:
        dist = _dist(self.positions[src], self.positions[dst])
        return _rssi_from_dist(dist, self.rssi_1m, self.path_loss_n, self.noise_rssi_amp, self.rng)

    def _energy_ok(self, node_id: int) -> bool:
        return self.nodes[node_id].energy > 0.0

    def _consume_energy_tx(self, node_id: int) -> bool:
        node = self.nodes[node_id]
        if node.energy <= 0.0:
            return False
        node.energy = max(0.0, node.energy - self.energy_per_tx)
        node.tx_count += 1
        return True

    def _consume_energy_rx(self, node_id: int) -> bool:
        node = self.nodes[node_id]
        if node.energy <= 0.0:
            return False
        node.energy = max(0.0, node.energy - self.energy_per_rx)
        node.rx_count += 1
        return True

    def _schedule_message(self, msg: Message) -> None:
        self.in_flight.append(msg)

    def _broadcast(self, src: int, kind: str, payload: Dict[str, Any], ttl: int = 1) -> None:
        if not self._consume_energy_tx(src):
            self.counters["control_dropped"] += 1
            return
        for dst in sorted(self.adjacency[src]):
            if not self._energy_ok(dst):
                continue
            dist = _dist(self.positions[src], self.positions[dst])
            delay = max(self.dt * 0.25, dist / RT_PROPAGATION_SPEED_MPS + 0.01)
            self._schedule_message(
                Message(
                    kind=kind,
                    src=src,
                    sender=src,
                    dst=dst,
                    payload={**payload, "rssi": self._link_rssi(src, dst)},
                    deliver_time=self.time + delay,
                )
            )

    def _unicast(self, src: int, dst: int, kind: str, payload: Dict[str, Any], ttl: int = 1) -> bool:
        if dst not in self.adjacency[src]:
            self.counters["control_dropped"] += 1
            return False
        if not self._consume_energy_tx(src):
            self.counters["control_dropped"] += 1
            return False
        dist = _dist(self.positions[src], self.positions[dst])
        delay = max(self.dt * 0.25, dist / RT_PROPAGATION_SPEED_MPS + 0.01)
        self._schedule_message(
            Message(
                kind=kind,
                src=src,
                sender=src,
                dst=dst,
                payload={**payload, "rssi": self._link_rssi(src, dst)},
                deliver_time=self.time + delay,
            )
        )
        return True

    def _neighbor_rssi_recent(self, node: Node, neighbor_id: int) -> bool:
        value = node.neighbor_rssi.get(neighbor_id)
        return value is not None

    def _update_neighbor_rssi(self, node: Node, neighbor_id: int, rssi: float) -> None:
        prev = node.neighbor_rssi.get(neighbor_id, rssi)
        smoothed = (31.0 * prev + rssi) / 32.0
        node.neighbor_rssi[neighbor_id] = smoothed

    def _nearest_route(self, node: Node, dest: int) -> Optional[RouteEntry]:
        entry = node.routes.get(dest)
        if entry is None:
            return None
        if entry.hop_count >= INF_HOPS:
            return None
        age = self.time - entry.last_update
        timeout = self.route_timeout
        if entry.dest == entry.next_hop and entry.hop_count == 1:
            timeout = max(timeout, self.hello_period * 4.0)
        if age > timeout:
            return None
        if self.backbone_forward_only and entry.hop_count > 1:
            if not (0 <= entry.next_hop < self.node_count):
                return None
            if self.nodes[entry.next_hop].role != ROLE_BACKBONE:
                return None
        return entry

    def _upsert_route(self, node: Node, dest: int, next_hop: int, hop_count: int, seq_num: int) -> bool:
        if dest == node.node_id:
            return False
        if node.role == ROLE_LEAF:
            parent = node.attached_backbone
            if parent is None:
                return False
            if next_hop != parent:
                return False
            if hop_count == 1 and dest != parent:
                return False
        if self.backbone_forward_only and hop_count > 1:
            if not (0 <= next_hop < self.node_count):
                return False
            if self.nodes[next_hop].role != ROLE_BACKBONE:
                return False
        now = self.time
        current = node.routes.get(dest)
        is_direct_neighbor_new = dest == next_hop and hop_count == 1

        if current is not None:
            is_direct_neighbor = current.dest == current.next_hop and current.hop_count == 1
            timeout = self.route_timeout if not is_direct_neighbor else max(self.route_timeout, self.hello_period * 4.0)
            age = now - current.last_update

            if age > timeout:
                node.routes[dest] = RouteEntry(dest, next_hop, hop_count, seq_num, now, True)
                node.route_changed = True
                if is_direct_neighbor_new:
                    node.my_info_changed = True
                self.counters["route_changes"] += 1
                return True

            if seq_num > current.seq_num:
                node.routes[dest] = RouteEntry(dest, next_hop, hop_count, seq_num, now, True)
                node.route_changed = True
                if is_direct_neighbor_new:
                    node.my_info_changed = True
                self.counters["route_changes"] += 1
                return True

            if seq_num == current.seq_num:
                if hop_count < current.hop_count:
                    route_age = now - current.last_update
                    if route_age > ROUTE_SETTLE_TIME_S or (current.hop_count - hop_count) >= 2:
                        node.routes[dest] = RouteEntry(dest, next_hop, hop_count, seq_num, now, True)
                        node.route_changed = True
                        self.counters["route_changes"] += 1
                        return True
                    current.last_update = now
                    return False
                if hop_count == current.hop_count:
                    # Ideal room assumption: avoid equal-cost route switching to reduce control churn.
                    current.last_update = now
                    return False

            return False

        node.routes[dest] = RouteEntry(dest, next_hop, hop_count, seq_num, now, True)
        node.route_changed = True
        if is_direct_neighbor_new:
            node.my_info_changed = True
        self.counters["route_changes"] += 1
        return True

    def _expire_routes(self, node: Node) -> None:
        expired: List[int] = []
        for dest, entry in list(node.routes.items()):
            if dest == node.node_id:
                continue
            timeout = self.route_timeout
            if entry.dest == entry.next_hop and entry.hop_count == 1:
                timeout = max(timeout, self.hello_period * 4.0)
            age = self.time - entry.last_update
            if age <= timeout:
                continue
            if entry.hop_count != INF_HOPS:
                entry.hop_count = INF_HOPS
                if entry.seq_num % 2 == 0:
                    entry.seq_num += 1
                entry.last_update = self.time
                entry.changed = True
                node.route_changed = True
                self.counters["route_changes"] += 1
                if entry.dest == entry.next_hop and entry.hop_count == INF_HOPS:
                    node.my_info_changed = True
            else:
                expired.append(dest)
        for dest in expired:
            node.routes.pop(dest, None)

    def _send_hello(self, node: Node) -> None:
        if node.energy <= 0.0:
            return
        node.hello_tx_count += 1
        if node.hello_tx_count % HELLO_KEEPALIVE_PERIODS == 0:
            node.hello_seq += 2
            node.my_info_changed = True

        active_neighbors = len(self._active_neighbors(node))
        node.degree = active_neighbors
        payload = {
            "seq_num": node.hello_seq,
            "my_degree": active_neighbors,
            "my_role": node.role,
        }
        self._broadcast(node.node_id, "HELLO", payload, ttl=1)
        self.counters["hello_packets"] += 1
        node.next_hello = self.time + self.hello_period + self.rng.uniform(0.0, self.hello_period * 0.35)

    def _send_update(self, node: Node) -> None:
        if node.energy <= 0.0:
            return
        now = self.time

        if now - self._last_update_sent[node.node_id] < UPDATE_MIN_INTERVAL_S:
            node.next_update = self._last_update_sent[node.node_id] + UPDATE_MIN_INTERVAL_S + self.rng.uniform(0.0, 0.5)
            return

        if self.hierarchy_enabled and node.role == ROLE_LEAF and node.attached_backbone is not None:
            # In Backbone-Leaf mode, the backbone owns proactive route advertisements.
            # Attached leaves rely on HELLO for direct parent reachability and do not
            # flood DSDV UPDATEs for routes that the backbone will advertise.
            for entry in node.routes.values():
                entry.changed = False
            node.route_changed = False
            node.my_info_changed = False
            node.next_update = self.time + (self.update_period * LEAF_UPDATE_SUPPRESS_BACKOFF) + self.rng.uniform(0.0, self.update_period)
            return

        changed_entries = [entry for entry in node.routes.values() if entry.dest != node.node_id and entry.hop_count < INF_HOPS and entry.changed]
        full_entries = [entry for entry in node.routes.values() if entry.dest != node.node_id]
        is_incremental = node.route_changed
        entries = changed_entries if is_incremental else full_entries
        entries = sorted(entries, key=lambda item: item.dest)[: self.update_payload_limit]
        if not entries and is_incremental:
            node.route_changed = False
            node.next_update = self.time + 0.1
            return
        if not entries:
            node.next_update = self.time + self.update_period + self.rng.uniform(0.0, self.update_period * 0.3)
            return

        payload = {
            "src": node.node_id,
            "sender_role": node.role,
            "entries": [
                {"dest": entry.dest, "hop_count": entry.hop_count, "seq_num": entry.seq_num}
                for entry in entries
            ],
            "incremental": is_incremental,
        }
        self._broadcast(node.node_id, "UPDATE", payload, ttl=self._ttl_cap(node, "UPDATE"))
        for entry in entries:
            entry.changed = False
        self._last_update_sent[node.node_id] = self.time
        self.counters["update_packets"] += 1
        node.route_changed = False if not any(entry.changed for entry in node.routes.values()) else True
        node.my_info_changed = False
        node.next_update = self.time + self.update_period + self.rng.uniform(0.0, self.update_period * 0.4)

    def _gen_data_packet(self, node: Node, dest: int) -> Dict[str, Any]:
        node.data_seq += 1
        return {
            "src": node.node_id,
            "dest": dest,
            "seq_num": node.data_seq,
            "created_at": self.time,
            "hop_count": 1,
            "path": [node.node_id],
            "request_ack": 1,
        }

    def _duplicate_seen(self, node: Node, src: int, seq_num: int) -> bool:
        last = node.dup_cache.get(src)
        if last is not None and seq_num <= last:
            return True
        node.dup_cache[src] = seq_num
        return False

    def _forward_data(self, node: Node, packet: Dict[str, Any]) -> bool:
        strict_path = packet.get("strict_path")
        if self.backbone_forward_only and node.role != ROLE_BACKBONE and not (isinstance(strict_path, list) and strict_path):
            self._queue_data_retry(node, packet)
            return False

        next_hop: Optional[int] = None
        if isinstance(strict_path, list) and strict_path:
            idx = int(packet.get("strict_idx", 0))
            if idx < len(strict_path) and strict_path[idx] == node.node_id and idx + 1 < len(strict_path):
                next_hop = int(strict_path[idx + 1])
                packet["strict_idx"] = idx + 1

        if next_hop is None:
            next_hop = self._next_hop_for_dest(node, int(packet["dest"]))
            if next_hop is None:
                self._queue_data_retry(node, packet)
                return False

        if next_hop not in self.adjacency[node.node_id]:
            self._queue_data_retry(node, packet)
            return False

        packet["hop_count"] = int(packet["hop_count"]) + 1
        if len(packet["path"]) < 8:
            packet["path"].append(node.node_id)
        payload = packet.copy()
        sent = self._unicast(node.node_id, next_hop, "DATA", payload, ttl=min(INF_HOPS, 10))
        if not sent:
            self._queue_data_retry(node, packet)
            return False
        return True

    def _queue_data_retry(self, node: Node, packet: Dict[str, Any]) -> None:
        retries = int(packet.get("retries", 0))
        if retries >= MAX_FORWARD_RETRIES:
            self.counters["data_dropped"] += 1
            node.dropped_count += 1
            return
        retry_pkt = packet.copy()
        retry_pkt["retries"] = retries + 1
        retry_pkt["retry_at"] = self.time + FORWARD_RETRY_DELAY_S
        node.pending_data.append(retry_pkt)

    def _process_pending_data(self, node: Node) -> None:
        if not node.pending_data:
            return
        keep: Deque[Dict[str, Any]] = deque()
        while node.pending_data:
            pkt = node.pending_data.popleft()
            retry_at = float(pkt.get("retry_at", self.time))
            if self.time < retry_at:
                keep.append(pkt)
                continue
            if not self._forward_data(node, pkt):
                # _forward_data may requeue or drop; do not keep stale copy.
                continue
        node.pending_data = keep

    def _path_to_gateway_exists(self, src_node: Node, max_depth: int = 16) -> bool:
        path = self._resolve_path(src_node.node_id, self.gateway_id, max_depth=max_depth)
        return bool(path and path[-1] == self.gateway_id)

    def _rank_sink_candidate(self, node: Node) -> Tuple[int, int, int, int]:
        role_rank = 0 if node.node_id == self.gateway_id else 1 if node.role == ROLE_BACKBONE else 2
        degree_rank = -len(self._active_neighbors(node))
        score_rank = -node.backbone_score
        return (role_rank, degree_rank, score_rank, node.node_id)

    def _select_sink_pool(self) -> List[int]:
        candidates: List[Node] = []
        if 0 <= self.gateway_id < self.node_count and self._energy_ok(self.gateway_id):
            candidates.append(self.nodes[self.gateway_id])
        for node in self.nodes:
            if node.node_id == self.gateway_id or node.energy <= 0.0:
                continue
            if node.role != ROLE_BACKBONE:
                continue
            if len(self._active_neighbors(node)) < 2:
                continue
            candidates.append(node)

        if not candidates:
            return [self.gateway_id] if 0 <= self.gateway_id < self.node_count else []

        seen: set[int] = set()
        ordered: List[int] = []
        for node in sorted(candidates, key=self._rank_sink_candidate):
            if node.node_id in seen:
                continue
            seen.add(node.node_id)
            ordered.append(node.node_id)
            if len(ordered) >= TRAFFIC_SINK_POOL:
                break
        return ordered

    def _choose_sink_for_source(self, src_id: int, sink_pool: Sequence[int]) -> Tuple[Optional[int], Optional[List[int]]]:
        best_sink: Optional[int] = None
        best_path: Optional[List[int]] = None
        for sink_id in sink_pool:
            if sink_id == src_id:
                continue
            path = self._resolve_path(src_id, sink_id)
            if not path or len(path) < 2:
                continue
            if best_path is None or len(path) < len(best_path) or (len(path) == len(best_path) and sink_id < int(best_sink if best_sink is not None else sink_id)):
                best_sink = sink_id
                best_path = path
        return best_sink, best_path

    def _resolve_path(self, src_id: int, dest_id: int, max_depth: int = 16) -> Optional[List[int]]:
        if src_id == dest_id:
            return [src_id]
        cur = src_id
        visited = {cur}
        path = [cur]
        depth = 0
        while depth < max_depth and cur != dest_id:
            node = self.nodes[cur]
            nxt = self._next_hop_for_dest(node, dest_id)
            if nxt is None:
                return None
            if nxt not in self.adjacency[cur]:
                return None
            if nxt in visited:
                return None
            visited.add(nxt)
            path.append(nxt)
            cur = nxt
            depth += 1
        if cur != dest_id:
            return None
        return path

    def _handle_hello(self, node: Node, msg: Message) -> None:
        if msg.sender != msg.src:
            return
        src = msg.src
        if src == node.node_id:
            return
        rssi = float(msg.payload.get("rssi", self._link_rssi(msg.sender, node.node_id)))
        base_threshold = self.rssi_threshold - self.noise_rssi_amp
        threshold = base_threshold - HELLO_RSSI_MARGIN_EXISTING if self._nearest_route(node, src) else base_threshold - HELLO_RSSI_MARGIN_NEW
        if rssi < threshold:
            return
        self._update_neighbor_rssi(node, src, rssi)
        node.neighbor_last_seen[src] = self.time
        seq = int(msg.payload.get("seq_num", 0))
        peer_degree = int(msg.payload.get("my_degree", 0))
        peer_role = int(msg.payload.get("my_role", ROLE_UNKNOWN))
        self._update_backbone_peer(node, src, peer_degree, peer_role, rssi)

        if node.role == ROLE_LEAF:
            if peer_role != ROLE_BACKBONE:
                return
            if node.attached_backbone is None:
                node.attached_backbone = src
                node.my_info_changed = True
            elif src != node.attached_backbone:
                return
        self._upsert_route(node, src, src, 1, seq)

    def _handle_update(self, node: Node, msg: Message) -> None:
        if msg.src == node.node_id:
            return
        sender = msg.sender
        sender_role = int(msg.payload.get("sender_role", ROLE_UNKNOWN))
        if self.backbone_forward_only and sender_role != ROLE_BACKBONE:
            return
        if node.role == ROLE_LEAF:
            if node.attached_backbone is None and sender_role == ROLE_BACKBONE:
                node.attached_backbone = sender
                node.my_info_changed = True
            if node.attached_backbone is None or sender != node.attached_backbone:
                return
        rssi = float(msg.payload.get("rssi", self._link_rssi(sender, node.node_id)))
        self._update_neighbor_rssi(node, sender, rssi)
        entries = msg.payload.get("entries", [])
        for entry in entries[: self.update_payload_limit]:
            dest = int(entry["dest"])
            hop = int(entry["hop_count"])
            seq_num = int(entry["seq_num"])
            if dest in {node.node_id, sender}:
                continue
            actual_hops = INF_HOPS if hop >= INF_HOPS else min(INF_HOPS, hop + 1)
            self._upsert_route(node, dest, sender, actual_hops, seq_num)

    def _handle_ack(self, node: Node, msg: Message) -> None:
        original_src = int(msg.payload["src_addr"])
        if node.node_id == original_src:
            created_at = float(msg.payload["created_at"])
            latency = max(0.0, self.time - created_at)
            self.latency_samples.append(latency)
            self.counters["acks_delivered"] += 1
            node.delivered_count += 1
            ack_seq = msg.payload.get("seq_num")
            if node.pending_ack_seq is not None and ack_seq is not None and int(ack_seq) == int(node.pending_ack_seq):
                node.pending_ack_seq = None
                node.pending_ack_deadline = 0.0
            return

        if self.backbone_forward_only and node.role != ROLE_BACKBONE:
            self.counters["control_dropped"] += 1
            return

        next_hop = self._next_hop_for_dest(node, original_src)
        if next_hop is None:
            self.counters["control_dropped"] += 1
            return

        ack_payload = dict(msg.payload)
        self._unicast(node.node_id, next_hop, "ACK", ack_payload, ttl=min(INF_HOPS, 10))

    def _handle_data(self, node: Node, msg: Message) -> None:
        packet = dict(msg.payload)
        if packet["dest"] == node.node_id:
            self.counters["data_delivered"] += 1
            node.delivered_count += 1
            self.hop_samples.append(int(packet.get("hop_count", 0)))
            if packet.get("request_ack", 0):
                ack_payload = {
                    "src_addr": int(packet["src"]),
                    "created_at": float(packet["created_at"]),
                    "seq_num": int(packet["seq_num"]),
                }
                next_hop = self._next_hop_for_dest(node, int(packet["src"]))
                if next_hop is not None:
                    self._unicast(node.node_id, next_hop, "ACK", ack_payload, ttl=min(INF_HOPS, 10))
            return
        # For strict-path forwarding in ideal-room mode, allow retransmitted copies.
        if "strict_path" not in packet:
            if self._duplicate_seen(node, int(packet["src"]), int(packet["seq_num"])):
                return
        self._forward_data(node, packet)

    def _deliver_message(self, msg: Message) -> None:
        if msg.dst is None:
            return
        if msg.dst not in self.adjacency[msg.sender]:
            self.counters["control_dropped"] += 1
            return
        receiver = self.nodes[msg.dst]
        if not self._consume_energy_rx(receiver.node_id):
            self.counters["control_dropped"] += 1
            return
        if msg.kind == "HELLO":
            self._handle_hello(receiver, msg)
        elif msg.kind == "UPDATE":
            self._handle_update(receiver, msg)
        elif msg.kind == "DATA":
            self._handle_data(receiver, msg)
        elif msg.kind == "ACK":
            self._handle_ack(receiver, msg)

    def _process_in_flight(self) -> None:
        next_queue: Deque[Message] = deque()
        while self.in_flight:
            msg = self.in_flight.popleft()
            if msg.deliver_time <= self.time:
                self._deliver_message(msg)
            else:
                next_queue.append(msg)
        self.in_flight = next_queue

    def _schedule_global_data(self) -> None:
        if self.data_period <= 0 or self.time < self.next_global_data:
            return

        def _stable_hop_count(node: Node) -> Optional[int]:
            path = self._resolve_path(node.node_id, self.gateway_id, max_depth=24)
            if not path or len(path) < 2:
                return None
            hop_count = len(path) - 1
            if hop_count > 6:
                return None
            return hop_count

        candidate_hops: Dict[int, int] = {}
        for node in self.nodes:
            if node.node_id == self.gateway_id or node.energy <= 0.0:
                continue
            if node.pending_ack_seq is not None:
                if self.time >= node.pending_ack_deadline:
                    self.counters["data_dropped"] += 1
                    node.dropped_count += 1
                    node.pending_ack_seq = None
                    node.pending_ack_deadline = 0.0
                else:
                    continue
            hop_count = _stable_hop_count(node)
            if hop_count is None:
                continue
            if not self._path_to_gateway_exists(node):
                continue
            if HIGH_RELIABILITY_TRAFFIC and hop_count > 3:
                continue
            if HIGH_RELIABILITY_TRAFFIC and hop_count > TRAFFIC_MAX_HOPS:
                continue
            candidate_hops[node.node_id] = hop_count

        candidates = [self.nodes[node_id] for node_id in candidate_hops]
        if not candidates:
            self.next_global_data = self.time + self.data_period + self.rng.uniform(0.0, self.data_period * 0.4)
            return

        sink_pool = self._select_sink_pool()

        if HIGH_RELIABILITY_TRAFFIC and len(candidates) > TRAFFIC_SOURCE_POOL:
            candidates = sorted(
                candidates,
                key=lambda n: (
                    candidate_hops[n.node_id],
                    0 if n.role == ROLE_BACKBONE else 1,
                    n.node_id,
                ),
            )[:TRAFFIC_SOURCE_POOL]

        leaf_candidates = [node for node in candidates if node.role == ROLE_LEAF]
        send_pool = list(leaf_candidates or candidates)
        self.rng.shuffle(send_pool)
        sends = min(TRAFFIC_BURST_SIZE, len(send_pool))
        for src_node in send_pool[:sends]:
            if src_node.node_id not in candidate_hops:
                self.counters["data_dropped"] += 1
                src_node.dropped_count += 1
                continue

            sink_id, strict_path = self._choose_sink_for_source(src_node.node_id, sink_pool)
            if sink_id is None or not strict_path:
                self.counters["data_dropped"] += 1
                src_node.dropped_count += 1
                continue
            packet = self._gen_data_packet(src_node, sink_id)
            packet["strict_path"] = strict_path
            packet["strict_idx"] = 0
            payload = dict(packet)
            sent = self._unicast(src_node.node_id, strict_path[1], "DATA", payload, ttl=min(INF_HOPS, 10))
            if sent:
                self.counters["data_packets"] += 1
                src_node.pending_ack_seq = int(packet["seq_num"])
                src_node.pending_ack_deadline = self.time + ACK_WAIT_TIMEOUT_S
            else:
                self.counters["data_dropped"] += 1
                src_node.dropped_count += 1

        self.next_global_data = self.time + self.data_period + self.rng.uniform(0.0, self.data_period * 0.4)

    def _collect_sample(self) -> None:
        delivered = self.counters["data_delivered"]
        generated = self.counters["data_packets"]
        pdr = 100.0 * delivered / generated if generated else 0.0
        avg_latency = sum(self.latency_samples) / len(self.latency_samples) if self.latency_samples else 0.0
        avg_hops = sum(self.hop_samples) / len(self.hop_samples) if self.hop_samples else 0.0
        # Additional metrics can be collected here
        sample = {
            "t": round(self.time, 6),
            "pdr": round(pdr, 6),
            "avg_latency_s": round(avg_latency, 6),
            "avg_hops": round(avg_hops, 6),
            "hello_packets": int(self.counters["hello_packets"]),
            "update_packets": int(self.counters["update_packets"]),
            "route_changes": int(self.counters["route_changes"]),
            "data_packets": int(self.counters["data_packets"]),
            "data_delivered": int(self.counters["data_delivered"]),
            "data_dropped": int(self.counters["data_dropped"]),
        }
        self.samples.append(sample)
        if self.video and len(self.video_frames) < self.video_max_frames and int(round(self.time / self.dt)) % self.video_stride == 0:
            self.video_frames.append({
                "t": self.time,
                "energy": [node.energy / self.initial_energy if self.initial_energy > 0 else 0.0 for node in self.nodes],
            })

    def _backbone_components(self) -> List[set[int]]:
        backbone_nodes = {node.node_id for node in self.nodes if node.role == ROLE_BACKBONE}
        if 0 <= self.gateway_id < self.node_count:
            backbone_nodes.add(self.gateway_id)

        visited: set[int] = set()
        components: List[set[int]] = []
        for start in sorted(backbone_nodes):
            if start in visited:
                continue
            comp: set[int] = set()
            queue: Deque[int] = deque([start])
            visited.add(start)
            while queue:
                cur = queue.popleft()
                comp.add(cur)
                for nxt in self.adjacency[cur]:
                    if nxt in backbone_nodes and nxt not in visited:
                        visited.add(nxt)
                        queue.append(nxt)
            components.append(comp)
        return components

    def _shortest_path(self, src: int, dst: int) -> Optional[List[int]]:
        if src == dst:
            return [src]
        queue: Deque[int] = deque([src])
        parent: Dict[int, Optional[int]] = {src: None}
        while queue:
            cur = queue.popleft()
            for nxt in self.adjacency[cur]:
                if nxt in parent:
                    continue
                parent[nxt] = cur
                if nxt == dst:
                    path = [dst]
                    p = dst
                    while parent[p] is not None:
                        p = int(parent[p])
                        path.append(p)
                    path.reverse()
                    return path
                queue.append(nxt)
        return None

    def _enforce_backbone_connectivity(self) -> None:
        if not self.backbone_forward_only:
            return

        while True:
            components = self._backbone_components()
            if len(components) <= 1:
                return

            comp_index: Dict[int, int] = {}
            for idx, comp in enumerate(components):
                for node_id in comp:
                    comp_index[node_id] = idx

            connector_candidates: List[Tuple[int, int, int]] = []
            for node in self.nodes:
                if node.role == ROLE_BACKBONE:
                    continue
                touched = {comp_index[nb] for nb in self.adjacency[node.node_id] if nb in comp_index}
                if len(touched) >= 2:
                    connector_candidates.append((len(touched), node.backbone_score, node.node_id))

            if connector_candidates:
                connector_candidates.sort(key=lambda x: (-x[0], -x[1], x[2]))
                promote_id = connector_candidates[0][2]
                self.nodes[promote_id].role = ROLE_BACKBONE
                self.nodes[promote_id].my_info_changed = True
                continue

            best_path: Optional[List[int]] = None
            for i in range(len(components)):
                for j in range(i + 1, len(components)):
                    for a in components[i]:
                        for b in components[j]:
                            path = self._shortest_path(a, b)
                            if not path:
                                continue
                            if best_path is None or len(path) < len(best_path):
                                best_path = path

            if not best_path:
                return

            promoted_any = False
            for node_id in best_path[1:-1]:
                if self.nodes[node_id].role != ROLE_BACKBONE:
                    self.nodes[node_id].role = ROLE_BACKBONE
                    self.nodes[node_id].my_info_changed = True
                    promoted_any = True
            if not promoted_any:
                return

    def step(self) -> None:
        self._process_in_flight()
        backbone_eval_ran = False
        for node in self.nodes:
            self._expire_routes(node)
            if self.hierarchy_enabled and self.time >= node.next_backbone_eval:
                self._evaluate_backbone(node)
                node.next_backbone_eval = self.time + BACKBONE_EVAL_INTERVAL_S + self.rng.uniform(0.0, 5.0)
                backbone_eval_ran = True
            if self.time >= node.next_hello:
                self._send_hello(node)
            if self.time >= node.next_update:
                self._send_update(node)
            self._process_pending_data(node)
        self._refresh_leaf_parents()
        if self.hierarchy_enabled and backbone_eval_ran:
            self._enforce_backbone_connectivity()
            self._refresh_leaf_parents()
        self._recompute_leaf_gradients()
        self._schedule_global_data()
        self._collect_sample()
        self.time += self.dt

    def run(self, record_video: bool = False, video_stride: Optional[int] = None, video_max_frames: Optional[int] = None) -> "DsdvEnvSim":
        if record_video:
            self.video = True
        if video_stride is not None:
            self.video_stride = max(1, int(video_stride))
        if video_max_frames is not None:
            self.video_max_frames = int(video_max_frames)
        steps = max(1, int(math.ceil(self.duration / self.dt)))
        for _ in range(steps):
            self.step()
        return self

    def summary(self) -> Dict[str, Any]:
        delivered = self.counters["data_delivered"]
        generated = self.counters["data_packets"]
        pdr = 100.0 * delivered / generated if generated else 0.0
        avg_latency = sum(self.latency_samples) / len(self.latency_samples) if self.latency_samples else 0.0
        avg_hops = sum(self.hop_samples) / len(self.hop_samples) if self.hop_samples else 0.0
        hello_n = int(self.counters["hello_packets"])
        update_n = int(self.counters["update_packets"])
        return {
            "sim_profile": self.sim_profile,
            "nodes": self.node_count,
            "duration": self.duration,
            "dt": self.dt,
            "pdr": pdr,
            "avg_latency_s": avg_latency,
            "avg_hops": avg_hops,
            "hello_packets": hello_n,
            "update_packets": update_n,
            "total_control_packets": hello_n + update_n,
            "control_dropped": int(self.counters["control_dropped"]),
            "data_packets": self.counters["data_packets"],
            "data_delivered": delivered,
            "data_dropped": self.counters["data_dropped"],
            "route_changes": self.counters["route_changes"],
            "acks_delivered": self.counters["acks_delivered"],
            "backbone_nodes": sum(1 for node in self.nodes if node.role == ROLE_BACKBONE),
            "leaf_nodes": sum(1 for node in self.nodes if node.role == ROLE_LEAF),
            "remaining_energy_mean": sum(node.energy for node in self.nodes) / len(self.nodes),
        }

    def summary_text(self) -> str:
        s = self.summary()
        lines = [
            f"sim_profile: {self.sim_profile}",
            f"nodes: {s['nodes']}",
            f"duration_s: {s['duration']}",
            f"dt_s: {s['dt']}",
            f"pdr_pct: {s['pdr']:.2f}",
            f"avg_latency_s: {s['avg_latency_s']:.6f}",
            f"avg_hops: {s['avg_hops']:.3f}",
            f"hello_packets: {s['hello_packets']}",
            f"update_packets: {s['update_packets']}",
            f"total_control_packets: {s['total_control_packets']}",
            f"control_dropped: {s['control_dropped']}",
            f"data_packets: {s['data_packets']}",
            f"data_delivered: {s['data_delivered']}",
            f"data_dropped: {s['data_dropped']}",
            f"route_changes: {s['route_changes']}",
            f"acks_delivered: {s['acks_delivered']}",
            f"backbone_nodes: {s['backbone_nodes']}",
            f"leaf_nodes: {s['leaf_nodes']}",
            f"remaining_energy_mean: {s['remaining_energy_mean']:.3f}",
        ]
        return "\n".join(lines) + "\n"

    def export_metrics_csv(self, out_path: Path) -> Path:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = ["t", "pdr", "avg_latency_s", "avg_hops", "hello_packets", "update_packets", "route_changes", "data_packets", "data_delivered", "data_dropped"]
        with out_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for sample in self.samples:
                writer.writerow({name: sample.get(name, 0) for name in fieldnames})
        return out_path

    def export_routing_table_csv(self, out_path: Path, focus_node: Optional[int] = None) -> Path:
        focus_index = self.focus_node if focus_node is None or focus_node < 0 else focus_node
        focus = self.nodes[focus_index]
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = ["dest", "next_hop", "hop_count", "seq_num", "last_update"]
        with out_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for dest in sorted(focus.routes):
                entry = focus.routes[dest]
                if entry.hop_count >= INF_HOPS:
                    continue
                writer.writerow(
                    {
                        "dest": entry.dest,
                        "next_hop": entry.next_hop,
                        "hop_count": entry.hop_count,
                        "seq_num": entry.seq_num,
                        "last_update": f"{entry.last_update:.3f}",
                    }
                )
        return out_path

    def export_per_node_stats_csv(self, out_path: Path) -> Path:
        """Per-node TX/RX and role for forwarding-load / hotspot analysis (chapter 4.8.2)."""
        role_names = {
            ROLE_UNKNOWN: "unknown",
            ROLE_BACKBONE: "backbone",
            ROLE_LEAF: "leaf",
        }
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = ["node_id", "role", "tx_count", "rx_count", "hello_tx_count", "dropped_count", "degree"]
        with out_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for node in self.nodes:
                writer.writerow(
                    {
                        "node_id": node.node_id,
                        "role": role_names.get(node.role, "unknown"),
                        "tx_count": int(node.tx_count),
                        "rx_count": int(node.rx_count),
                        "hello_tx_count": int(node.hello_tx_count),
                        "dropped_count": int(node.dropped_count),
                        "degree": int(node.degree),
                    }
                )
        return out_path

    def _topology_frame_title(self) -> str:
        label = SIM_PROFILE_LABELS.get(self.sim_profile, self.sim_profile)
        return f"WSN topology | nodes={self.node_count} | {label}"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Discrete-time WSN + DSDV simulator")
    parser.add_argument("--nodes", type=int, default=30)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--dt", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--focus-node", type=int, default=DEFAULT_GATEWAY_ID)
    parser.add_argument("--area", type=float, default=None)
    parser.add_argument("--area-w", type=float, default=DEFAULT_AREA_W)
    parser.add_argument("--area-h", type=float, default=DEFAULT_AREA_H)
    parser.add_argument("--gateway-id", type=int, default=DEFAULT_GATEWAY_ID)
    parser.add_argument("--max-degree", type=int, default=DEFAULT_MAX_DEGREE)
    parser.add_argument("--extra-edge-factor", type=float, default=DEFAULT_EXTRA_EDGE_FACTOR)
    parser.add_argument("--max-attempts", type=int, default=DEFAULT_MAX_ATTEMPTS)
    parser.add_argument("--rssi-1m", type=float, default=DEFAULT_RSSI_1M)
    parser.add_argument("--path-loss-n", type=float, default=DEFAULT_PATH_LOSS_N)
    parser.add_argument("--rssi-threshold", type=float, default=DEFAULT_RSSI_THRESHOLD)
    parser.add_argument("--noise-rssi-amp", type=float, default=DEFAULT_NOISE_RSSI_AMP)
    parser.add_argument(
        "--placement",
        type=str,
        default=DEFAULT_NODE_PLACEMENT,
        choices=("clustered", "uniform"),
        help="Node layout: clustered (Gaussian cloud; gateway at cluster hub) or uniform (legacy).",
    )
    parser.add_argument(
        "--cluster-std-frac",
        type=float,
        default=DEFAULT_CLUSTER_STD_FRAC,
        help="For clustered placement, Gaussian sigma as a fraction of min(area_w, area_h). Smaller => tighter cluster.",
    )
    parser.add_argument("--hello", type=float, default=DEFAULT_HELLO_PERIOD)
    parser.add_argument("--update", type=float, default=DEFAULT_UPDATE_PERIOD)
    parser.add_argument("--timeout", type=float, default=DEFAULT_ROUTE_TIMEOUT)
    parser.add_argument("--update-payload-limit", type=int, default=DEFAULT_UPDATE_PAYLOAD_LIMIT)
    parser.add_argument("--data-period", type=float, default=DEFAULT_DATA_PERIOD)
    parser.add_argument("--data-start-delay", type=float, default=DEFAULT_DATA_START_DELAY_S)
    parser.add_argument("--initial-energy", type=float, default=DEFAULT_INITIAL_ENERGY)
    parser.add_argument("--energy-per-tx", type=float, default=DEFAULT_ENERGY_PER_TX)
    parser.add_argument("--energy-per-rx", type=float, default=DEFAULT_ENERGY_PER_RX)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--video-format", type=str, default="mp4")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--video-stride", type=int, default=DEFAULT_VIDEO_STRIDE)
    parser.add_argument("--video-max-frames", type=int, default=DEFAULT_VIDEO_MAX_FRAMES)

    backbone_group = parser.add_mutually_exclusive_group()
    backbone_group.add_argument(
        "--backbone-forward-only",
        dest="backbone_forward_only",
        action="store_true",
        help="Only allow multi-hop forwarding through backbone nodes.",
    )
    backbone_group.add_argument(
        "--allow-leaf-forward",
        dest="backbone_forward_only",
        action="store_false",
        help="Allow multi-hop forwarding through leaf nodes.",
    )
    parser.set_defaults(backbone_forward_only=BACKBONE_FORWARD_ONLY)

    parser.add_argument(
        "--sim-profile",
        type=str,
        choices=("baseline", "backbone_leaf", "full"),
        default="full",
        help="Single-run scenario: baseline DSDV, +Backbone-Leaf only, or +Gradient (full). Ignored if --compare-three.",
    )
    parser.add_argument(
        "--compare-three",
        action="store_true",
        help="Run all three scenarios into n<N>/baseline_dsdv, dsdv_backbone_leaf, dsdv_backbone_leaf_gradient.",
    )
    parser.add_argument(
        "--list-scenarios",
        action="store_true",
        help="Print preset evaluation scenarios S1..S12 (Vietnamese labels) and exit.",
    )
    scenario_cli = parser.add_mutually_exclusive_group()
    scenario_cli.add_argument(
        "--scenario",
        type=str,
        default=None,
        choices=SCENARIO_ORDER,
        metavar="S1",
        help="Run a single preset scenario S1..S12; outputs go under --scenarios-dir/<id>/ (see --list-scenarios).",
    )
    scenario_cli.add_argument(
        "--all-scenarios",
        action="store_true",
        help="Run all twelve preset scenarios S1..S12 sequentially (separate folder per id).",
    )
    parser.add_argument(
        "--scenarios-dir",
        type=Path,
        default=None,
        help="Root directory for --scenario / --all-scenarios outputs. Default: <out-dir>/scenarios",
    )

    return parser.parse_args(argv)


def _resolve_run_dir(args: argparse.Namespace, profile: str) -> Path:
    scenario_id = getattr(args, "scenario_id", None)
    if scenario_id:
        root = Path(getattr(args, "scenarios_dir", Path(args.out_dir) / "scenarios"))
        out_base = root / str(scenario_id)
        name_map = dict(SIM_PROFILE_SEQUENCE)
        if args.compare_three:
            return out_base / name_map[profile]
        return out_base

    base = Path(args.out_dir) / f"n{args.nodes}"
    name_map = dict(SIM_PROFILE_SEQUENCE)
    if args.compare_three:
        return base / name_map[profile]
    if profile == "full":
        return base
    return base / name_map[profile]


def _make_sim(args: argparse.Namespace, profile: str) -> DsdvEnvSim:
    return DsdvEnvSim(
        nodes=args.nodes,
        duration=args.duration,
        dt=args.dt,
        seed=args.seed,
        out_dir=args.out_dir,
        focus_node=args.focus_node,
        area=args.area,
        area_w=args.area_w,
        area_h=args.area_h,
        gateway_id=args.gateway_id,
        max_degree=args.max_degree,
        extra_edge_factor=args.extra_edge_factor,
        max_attempts=args.max_attempts,
        rssi_1m=args.rssi_1m,
        path_loss_n=args.path_loss_n,
        rssi_threshold=args.rssi_threshold,
        noise_rssi_amp=args.noise_rssi_amp,
        node_placement=args.placement,
        cluster_std_frac=args.cluster_std_frac,
        hello=args.hello,
        update=args.update,
        timeout=args.timeout,
        update_payload_limit=args.update_payload_limit,
        data_period=args.data_period,
        data_start_delay=args.data_start_delay,
        initial_energy=args.initial_energy,
        energy_per_tx=args.energy_per_tx,
        energy_per_rx=args.energy_per_rx,
        no_plots=args.no_plots,
        video=args.video,
        video_format=args.video_format,
        fps=args.fps,
        video_stride=args.video_stride,
        video_max_frames=args.video_max_frames,
        backbone_forward_only=args.backbone_forward_only,
        sim_profile=profile,
    )


def _warn_if_simulation_too_short(args: argparse.Namespace, sim: DsdvEnvSim) -> None:
    """HELLO first fires in [0.4×hello, hello]; DATA starts after ``data_start_delay``."""
    hello_p = float(args.hello)
    dur = float(args.duration)
    dsd = float(args.data_start_delay)
    if hello_p > 0 and dur < 0.4 * hello_p:
        print(
            f"[WARN] duration={dur}s < 0.4×hello={0.4 * hello_p:.1f}s: "
            "thường **chưa có HELLO** → update/route/data có thể toàn 0."
        )
    if args.data_period > 0 and dur < dsd + float(args.data_period):
        print(
            f"[WARN] duration={dur}s < data_start_delay({dsd}s)+data_period: "
            "**chưa có (hoặc rất ít) gói DATA** → PDR/độ trễ dữ liệu ≈ 0 không phản ánh chất lượng routing."
        )


def _export_one_run(args: argparse.Namespace, sim: DsdvEnvSim, run_dir: Path) -> Dict[str, Any]:
    run_dir.mkdir(parents=True, exist_ok=True)

    sim.run(record_video=args.video, video_stride=args.video_stride, video_max_frames=args.video_max_frames)
    _warn_if_simulation_too_short(args, sim)
    sim.export_metrics_csv(run_dir / "metrics_timeseries.csv")
    sim.export_routing_table_csv(run_dir / "routing_table_node.csv", focus_node=args.focus_node if args.focus_node >= 0 else None)
    (run_dir / "summary.txt").write_text(sim.summary_text(), encoding="utf-8")
    sim.export_per_node_stats_csv(run_dir / "per_node_stats.csv")

    chart_title = SIM_PROFILE_LABELS.get(sim.sim_profile, sim.sim_profile)

    if not args.no_plots:
        plot_topology_png(
            sim.positions,
            sim.adjacency,
            sim.gateway_id,
            [node.role for node in sim.nodes],
            [
                node.node_id
                for node in sim.nodes
                if node.role == ROLE_LEAF and node.gradient_next_hop is not None
            ],
            run_dir / "topology.png",
            title=sim._topology_frame_title(),
            sim_profile=sim.sim_profile,
        )
        plot_metrics_chart(sim.samples, run_dir / "metrics_chart.png", title=chart_title)
        plot_metrics_bar_chart(sim.samples, run_dir / "metrics_chart_bars.png")

    if args.video:
        suffix = ".gif" if args.video_format.lower() == "gif" else ".mp4"
        save_topology_animation(
            sim.video_frames,
            sim.positions,
            sim.adjacency,
            sim.gateway_id,
            [node.role for node in sim.nodes],
            run_dir / f"simulation{suffix}",
            fps=args.fps,
            title=sim._topology_frame_title(),
        )

    summary = sim.summary()
    print(f"DSDV WSN simulation complete | {chart_title}")
    print(f"Output directory: {run_dir}")
    print(f"PDR: {summary['pdr']:.2f}%")
    print(f"Avg latency: {summary['avg_latency_s']:.6f} s")
    print(f"Avg hops: {summary['avg_hops']:.3f}")
    print(f"Route changes: {summary['route_changes']}")

    scenario_id = getattr(args, "scenario_id", None)
    if scenario_id:
        spec: Dict[str, Any] = {
            "scenario_id": scenario_id,
            "description_vi": getattr(args, "scenario_description_vi", ""),
            "sim_profile": sim.sim_profile,
            "profile_label": SIM_PROFILE_LABELS.get(sim.sim_profile, sim.sim_profile),
            "duration_s": float(args.duration),
            "dt_s": float(args.dt),
            "seed": int(args.seed),
            "nodes": int(args.nodes),
            "placement": str(args.placement),
            "area_w_m": float(args.area_w),
            "area_h_m": float(args.area_h),
            "cluster_std_frac": float(args.cluster_std_frac),
            "data_period_s": float(args.data_period),
            "data_start_delay_s": float(args.data_start_delay),
            "hello_period_s": float(args.hello),
            "update_period_s": float(args.update),
            "route_timeout_s": float(args.timeout),
            "rssi_1m_dbm": float(args.rssi_1m),
            "path_loss_n": float(args.path_loss_n),
            "rssi_threshold_dbm": float(args.rssi_threshold),
            "noise_rssi_amp_db": float(args.noise_rssi_amp),
            "max_degree": int(args.max_degree),
            "extra_edge_factor": float(args.extra_edge_factor),
            "max_attempts": int(args.max_attempts),
            "backbone_forward_only": bool(args.backbone_forward_only),
            "compare_three": bool(args.compare_three),
            "summary": summary,
        }
        (run_dir / "scenario_spec.json").write_text(json.dumps(spec, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    return summary


def main(argv: Optional[Sequence[str]] = None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    except Exception:
        pass
    args = parse_args(argv)

    if args.list_scenarios:
        for sid in SCENARIO_ORDER:
            desc = SCENARIO_PRESETS[sid].get("description_vi", "")
            print(f"{sid}: {desc}")
        return 0

    if args.all_scenarios or args.scenario is not None:
        if args.scenarios_dir is None:
            args.scenarios_dir = Path(args.out_dir) / "scenarios"
        scenario_ids = list(SCENARIO_ORDER) if args.all_scenarios else [str(args.scenario)]
        profiles = [key for key, _ in SIM_PROFILE_SEQUENCE] if args.compare_three else [args.sim_profile]
        all_scenario_rows: List[Dict[str, Any]] = []

        for sid in scenario_ids:
            run_args = _apply_scenario_to_args(args, sid)
            run_args.scenarios_dir = args.scenarios_dir
            scenario_summaries: List[Dict[str, Any]] = []
            for profile in profiles:
                sim = _make_sim(run_args, profile)
                run_dir = _resolve_run_dir(run_args, profile)
                summary = _export_one_run(run_args, sim, run_dir)
                scenario_summaries.append(summary)
                all_scenario_rows.append({"scenario_id": sid, "sim_profile": profile, "summary": summary})
            if args.compare_three and not args.no_plots:
                plot_compare_control_overhead(scenario_summaries, _resolve_run_dir(run_args, profiles[0]).parent / "control_overhead_compare.png")

        root = Path(args.scenarios_dir)
        if args.compare_three:
            export_all_scenario_control_summary(all_scenario_rows, root / "control_overhead_12_scenarios.csv")
            if args.all_scenarios and not args.no_plots:
                plot_all_scenarios_control_metric(
                    all_scenario_rows,
                    root / "update_overhead_12_scenarios.png",
                    metric="update_packets",
                    title="UPDATE overhead across 12 scenarios",
                )
                plot_all_scenarios_control_metric(
                    all_scenario_rows,
                    root / "total_control_12_scenarios.png",
                    metric="total_control_packets",
                    title="Total control overhead across 12 scenarios",
                )
        if args.all_scenarios:
            print(f"--- all-scenarios: wrote {len(scenario_ids)} runs under {root} ---")
        else:
            print(f"--- scenario {scenario_ids[0]}: output under {root / scenario_ids[0]} ---")
        if args.compare_three:
            print("--- each scenario folder contains baseline_dsdv, dsdv_backbone_leaf, dsdv_backbone_leaf_gradient ---")
        return 0

    profiles = [key for key, _ in SIM_PROFILE_SEQUENCE] if args.compare_three else [args.sim_profile]

    summaries: List[Dict[str, Any]] = []
    for profile in profiles:
        sim = _make_sim(args, profile)
        run_dir = _resolve_run_dir(args, profile)
        summaries.append(_export_one_run(args, sim, run_dir))

    if args.compare_three:
        base = Path(args.out_dir) / f"n{args.nodes}"
        if not args.no_plots:
            plot_compare_control_overhead(summaries, base / "control_overhead_compare.png")
        print(f"--- compare-three: outputs under {base} (baseline_dsdv, dsdv_backbone_leaf, dsdv_backbone_leaf_gradient) ---")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
