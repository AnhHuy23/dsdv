"""
DSDV environment-aware simulator (pure Python, no `simpy`).

Mục tiêu:
- Mô phỏng DSDV (HELLO + UPDATE + forward DATA) như `wsn_dsdv_sim.py`.
- Thay vì đặt node trên grid, dùng topology ngẫu nhiên + radio model RSSI/threshold
  + ràng buộc max_degree (giống tinh thần `network.py` trong `dsdv_gradient_simulator`).

Chạy:
  python dsdv_env_sim.py --nodes 30 --duration 120
  python dsdv_env_sim.py --nodes 30 --duration 60 --video --video-format gif

Đầu ra hình minh họa (matplotlib):
  - topology.png: vị trí node + liên kết vật lý + đánh dấu gateway
  - metrics_chart.png: PDR, latency, hops, gói điều khiển theo thời gian
  - simulation.mp4 hoặc .gif: năng lượng node theo thời gian (khi --video)
"""

from __future__ import annotations

import argparse
import csv
import math
import random
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple


INF_HOPS = 255


# ============================================================
# Figures (topology + metrics + optional animation)
# ============================================================


def _try_import_matplotlib():
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        return plt, FuncAnimation
    except ImportError:
        return None, None


def plot_topology_png(
    *,
    adj: Dict[int, List[int]],
    pos: Dict[int, Tuple[float, float]],
    gateway_id: int,
    area_w: float,
    area_h: float,
    out_path: Path,
    title: str = "Topology (physical links)",
) -> bool:
    plt, _ = _try_import_matplotlib()
    if plt is None:
        return False
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(10, 8))
    drawn: set[Tuple[int, int]] = set()
    for u, nbrs in adj.items():
        x1, y1 = pos[u]
        for v in nbrs:
            key = (min(u, v), max(u, v))
            if key in drawn:
                continue
            drawn.add(key)
            x2, y2 = pos[v]
            ax.plot([x1, x2], [y1, y2], color="#94a3b8", linewidth=0.9, zorder=1)

    xs = [pos[i][0] for i in range(len(pos))]
    ys = [pos[i][1] for i in range(len(pos))]
    ax.scatter(xs, ys, s=55, c="#3b82f6", edgecolors="white", linewidths=0.6, zorder=2)
    gx, gy = pos[gateway_id]
    ax.scatter([gx], [gy], s=220, marker="s", c="#f59e0b", edgecolors="black", linewidths=0.8, zorder=3)

    for i in range(len(pos)):
        x, y = pos[i]
        ax.text(x + area_w * 0.012, y + area_h * 0.012, str(i), fontsize=8, color="#1e293b")

    ax.set_xlim(-area_w * 0.02, area_w * 1.02)
    ax.set_ylim(-area_h * 0.02, area_h * 1.02)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.grid(True, alpha=0.35)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return True


def plot_metrics_chart(samples: List[dict], out_path: Path, title: str = "KPI theo thời gian") -> bool:
    plt, _ = _try_import_matplotlib()
    if plt is None or not samples:
        return False
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    ts = [s["t"] for s in samples]
    pdr = [s.get("pdr", 0.0) * 100.0 for s in samples]
    lat = [s.get("avg_latency_s", 0.0) for s in samples]
    hops = [s.get("avg_hops", 0.0) for s in samples]
    hello = [s.get("hello_packets", 0) for s in samples]
    upd = [s.get("update_packets", 0) for s in samples]

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    ax0, ax1, ax2, ax3 = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

    ax0.plot(ts, pdr, color="#2563eb", linewidth=1.2)
    ax0.set_ylabel("PDR (%)")
    ax0.set_title("Packet delivery ratio (cumulative)")
    ax0.grid(True, alpha=0.35)

    ax1.plot(ts, lat, color="#059669", linewidth=1.2)
    ax1.set_ylabel("Latency (s)")
    ax1.set_title("Avg latency (delivered packets)")
    ax1.grid(True, alpha=0.35)

    ax2.plot(ts, hops, color="#7c3aed", linewidth=1.2)
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("Hops")
    ax2.set_title("Avg hop count (delivered)")
    ax2.grid(True, alpha=0.35)

    ax3.plot(ts, hello, label="HELLO", color="#ea580c", linewidth=1.0)
    ax3.plot(ts, upd, label="UPDATE", color="#0891b2", linewidth=1.0)
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("Packets (cumulative)")
    ax3.set_title("Control plane")
    ax3.legend(loc="upper left")
    ax3.grid(True, alpha=0.35)

    fig.suptitle(title, fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return True


def save_topology_animation(
    *,
    frames: List[Tuple[float, List[float]]],
    adj: Dict[int, List[int]],
    pos: Dict[int, Tuple[float, float]],
    gateway_id: int,
    area_w: float,
    area_h: float,
    initial_energy: float,
    out_path: Path,
    fps: int = 8,
) -> bool:
    plt, FuncAnimation = _try_import_matplotlib()
    if plt is None or FuncAnimation is None or not frames:
        return False
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    n = len(pos)
    fig, ax = plt.subplots(figsize=(10, 8))

    def draw_edges() -> None:
        drawn: set[Tuple[int, int]] = set()
        for u, nbrs in adj.items():
            x1, y1 = pos[u]
            for v in nbrs:
                key = (min(u, v), max(u, v))
                if key in drawn:
                    continue
                drawn.add(key)
                x2, y2 = pos[v]
                ax.plot([x1, x2], [y1, y2], color="#cbd5e1", linewidth=0.85, zorder=1)

    def update(frame_idx: int):
        ax.clear()
        t, energies = frames[frame_idx]
        draw_edges()
        xs = [pos[i][0] for i in range(n)]
        ys = [pos[i][1] for i in range(n)]
        sc = ax.scatter(
            xs,
            ys,
            c=energies,
            cmap="viridis",
            vmin=0.0,
            vmax=max(initial_energy, 1.0),
            s=70,
            edgecolors="white",
            linewidths=0.5,
            zorder=2,
        )
        gx, gy = pos[gateway_id]
        ax.scatter([gx], [gy], s=240, marker="s", facecolors="none", edgecolors="#f59e0b", linewidths=2.0, zorder=3)
        for i in range(n):
            ax.text(pos[i][0] + area_w * 0.01, pos[i][1] + area_h * 0.01, str(i), fontsize=7, color="#334155")
        ax.set_xlim(-area_w * 0.02, area_w * 1.02)
        ax.set_ylim(-area_h * 0.02, area_h * 1.02)
        ax.set_aspect("equal")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title(f"DSDV env sim — t = {t:.1f}s (màu = năng lượng node)")
        ax.grid(True, alpha=0.3)
        e_min = min(energies) if energies else 0.0
        e_max = max(energies) if energies else 0.0
        ax.text(
            0.02,
            0.98,
            f"E: min={e_min:.1f} max={e_max:.1f}",
            transform=ax.transAxes,
            va="top",
            fontsize=9,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
        )

    ani = FuncAnimation(fig, update, frames=len(frames), interval=1000 / max(1, fps), repeat=False)

    if out_path.suffix.lower() == ".mp4":
        try:
            ani.save(str(out_path), fps=fps, dpi=120)
        except Exception:
            gif_path = out_path.with_suffix(".gif")
            ani.save(str(gif_path), fps=fps, dpi=100, writer="pillow")
    else:
        ani.save(str(out_path), fps=fps, dpi=110, writer="pillow")

    plt.close(fig)
    return True


# ============================================================
# Topology generator (adapted from dsdv_gradient_simulator ideas)
# ============================================================


def _rssi_from_dist(
    d: float,
    *,
    rssi_1m: float,
    path_loss_n: float,
    noise_amp: float,
    rng: random.Random,
) -> float:
    """RSSI(d) = rssi_1m - 10*n*log10(d) + noise, với noise ~ U(-noise_amp, noise_amp)."""
    d = max(d, 0.5)
    base = rssi_1m - 10.0 * path_loss_n * math.log10(d)
    noise = rng.uniform(-noise_amp, noise_amp)
    return base + noise


def _max_distance_from_rssi_threshold(
    *,
    rssi_1m: float,
    path_loss_n: float,
    rssi_threshold: float,
) -> float:
    """
    Giả sử RSSI(d) = rssi_1m - 10*n*log10(d) (bỏ qua noise),
    giải dmax để RSSI(d) >= rssi_threshold.
    """
    if rssi_threshold >= rssi_1m:
        return 1.0
    return 10.0 ** ((rssi_1m - rssi_threshold) / (10.0 * path_loss_n))


def generate_positions(
    *,
    n_nodes: int,
    area_w: float,
    area_h: float,
    seed: int,
) -> Dict[int, Tuple[float, float]]:
    rng = random.Random(seed)
    pos: Dict[int, Tuple[float, float]] = {}
    for i in range(n_nodes):
        x = rng.uniform(0.0, area_w)
        y = rng.uniform(0.0, area_h)
        pos[i] = (x, y)
    return pos


def constrained_connectivity_graph(
    *,
    n_nodes: int,
    gateway_id: int,
    pos: Dict[int, Tuple[float, float]],
    max_degree: int,
    dmax: float,
    extra_edge_factor: float,
) -> Tuple[Dict[int, List[int]], Dict[Tuple[int, int], float]]:
    """
    Tạo graph đảm bảo liên thông bằng heuristic "grow tree" từ gateway,
    đồng thời ràng buộc degree <= max_degree.

    Trả về:
    - adjacency: node -> list(neighbors)
    - dist_weight: (u,v) -> distance (dùng để ưu tiên cạnh gần hơn khi thêm)
    """
    if n_nodes <= 0:
        raise ValueError("n_nodes must be > 0")
    if gateway_id < 0 or gateway_id >= n_nodes:
        raise ValueError("gateway_id out of range")

    # Candidate neighbors if within dmax
    dist_weight: Dict[Tuple[int, int], float] = {}
    candidate_neighbors: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(n_nodes)}
    for i in range(n_nodes):
        xi, yi = pos[i]
        for j in range(i + 1, n_nodes):
            xj, yj = pos[j]
            d = math.hypot(xi - xj, yi - yj)
            if d <= dmax:
                dist_weight[(i, j)] = d
                dist_weight[(j, i)] = d
                candidate_neighbors[i].append((j, d))
                candidate_neighbors[j].append((i, d))

    if n_nodes > 1 and not candidate_neighbors[gateway_id]:
        raise RuntimeError(
            "Gateway has 0 candidate neighbors; try bigger area or loosen RSSI threshold."
        )

    deg = {i: 0 for i in range(n_nodes)}
    adjacency: Dict[int, List[int]] = {i: [] for i in range(n_nodes)}
    connected = {gateway_id}
    remaining = set(range(n_nodes)) - connected

    # sort each candidate list by distance once (helps deterministic results)
    for u in candidate_neighbors:
        candidate_neighbors[u].sort(key=lambda t: t[1])

    # Step 1: grow tree to connect all nodes
    while remaining:
        best_edge: Optional[Tuple[float, int, int]] = None  # (distance, u, v)

        for u in list(remaining):
            for v, d in candidate_neighbors[u]:
                if v not in connected:
                    continue
                if deg[u] >= max_degree or deg[v] >= max_degree:
                    continue
                if best_edge is None or d < best_edge[0]:
                    best_edge = (d, u, v)

        if best_edge is None:
            raise RuntimeError(
                "Cannot connect all nodes under degree constraint. "
                "Try: increase max_attempts, loosen rssi_threshold, increase area density, "
                "or relax max_degree."
            )

        _, u, v = best_edge
        adjacency[u].append(v)
        adjacency[v].append(u)
        deg[u] += 1
        deg[v] += 1
        connected.add(u)
        remaining.remove(u)

    # Step 2: add redundancy edges if degree allows
    # target_edges ~ (n-1) * extra_edge_factor (like network.py)
    target_edges = int((n_nodes - 1) * extra_edge_factor)

    existing = set()
    for u in range(n_nodes):
        for v in adjacency[u]:
            existing.add((u, v))

    # gather all candidate edges (u,v) with u < v; sort by distance
    candidate_edges: List[Tuple[float, int, int]] = []
    for (u, v), d in dist_weight.items():
        if u < v:
            candidate_edges.append((d, u, v))
    candidate_edges.sort(key=lambda t: t[0])

    # current edges count (undirected)
    curr_edges = sum(len(adjacency[u]) for u in range(n_nodes)) // 2
    for d, u, v in candidate_edges:
        if curr_edges >= target_edges:
            break
        if (u in adjacency and v in adjacency[u]) or ((u, v) in existing) or ((v, u) in existing):
            continue
        if deg[u] >= max_degree or deg[v] >= max_degree:
            continue
        adjacency[u].append(v)
        adjacency[v].append(u)
        deg[u] += 1
        deg[v] += 1
        curr_edges += 1

    return adjacency, dist_weight


def generate_random_topology(
    *,
    n_nodes: int,
    gateway_id: int,
    area_w: float,
    area_h: float,
    max_degree: int,
    seed: int,
    rssi_1m: float,
    path_loss_n: float,
    rssi_threshold: float,
    extra_edge_factor: float,
    max_attempts: int,
) -> Tuple[Dict[int, List[int]], Dict[int, Tuple[float, float]]]:
    dmax = _max_distance_from_rssi_threshold(
        rssi_1m=rssi_1m, path_loss_n=path_loss_n, rssi_threshold=rssi_threshold
    )

    last_err: Optional[Exception] = None
    base_seed = seed if seed is not None else random.randint(1, 10**9)
    for k in range(max_attempts):
        pos = generate_positions(
            n_nodes=n_nodes,
            area_w=area_w,
            area_h=area_h,
            seed=base_seed + k,
        )
        try:
            adjacency, _ = constrained_connectivity_graph(
                n_nodes=n_nodes,
                gateway_id=gateway_id,
                pos=pos,
                max_degree=max_degree,
                dmax=dmax,
                extra_edge_factor=extra_edge_factor,
            )
            return adjacency, pos
        except Exception as e:
            last_err = e
            continue

    raise RuntimeError(f"Failed after {max_attempts} attempts. Last error: {last_err}")


# ============================================================
# DSDV simulator (HELLO + UPDATE + DATA)
# ============================================================


@dataclass
class RouteEntry:
    dest: int
    next_hop: int
    hop_count: int
    seq_num: int
    last_update: float
    changed: bool = False


@dataclass
class Message:
    kind: str  # hello | update | data
    src: int
    sender: int
    dst: Optional[int]
    payload: dict
    deliver_time: float


@dataclass
class Node:
    nid: int
    x: float
    y: float
    seq_num: int = 0
    routes: Dict[int, RouteEntry] = field(default_factory=dict)
    neighbor_rssi: Dict[int, float] = field(default_factory=dict)
    next_hello: float = 0.0
    next_update: float = 0.0

    sent_data: int = 0
    recv_data: int = 0

    energy: float = 0.0

    def route_to(self, dest: int, now: float, route_timeout: float) -> Optional[RouteEntry]:
        e = self.routes.get(dest)
        if e is None:
            return None
        if now - e.last_update > route_timeout:
            return None
        if e.hop_count >= INF_HOPS:
            return None
        return e


class DsdvEnvSim:
    def __init__(
        self,
        *,
        n_nodes: int = 30,
        duration: float = 120.0,
        dt: float = 1.0,
        seed: int = 42,
        # topology area
        area: float = 100.0,
        area_w: Optional[float] = None,
        area_h: Optional[float] = None,
        gateway_id: int = 0,
        # radio / RSSI model knobs
        rssi_1m: float = -55.0,
        path_loss_n: float = 2.5,
        rssi_threshold: float = -90.0,
        noise_rssi_amp: float = 2.0,
        # graph constraints
        max_degree: int = 6,
        extra_edge_factor: float = 1.4,
        max_attempts: int = 300,
        # DSDV protocol knobs
        hello_interval: float = 5.0,
        update_interval: float = 15.0,
        route_timeout: float = 45.0,
        update_payload_limit: int = 12,
        # traffic knobs
        data_period: float = 2.5,
        # energy knobs
        initial_energy: float = 1000.0,
        energy_per_tx: float = 0.01,
        energy_per_rx: float = 0.005,
    ) -> None:
        self.rng = random.Random(seed)
        self.n_nodes = n_nodes
        self.duration = duration
        self.dt = dt
        self.seed = seed

        self.area_w = area_w if area_w is not None else area
        self.area_h = area_h if area_h is not None else area
        self.gateway_id = gateway_id

        self.rssi_1m = rssi_1m
        self.path_loss_n = path_loss_n
        self.rssi_threshold = rssi_threshold
        self.noise_rssi_amp = noise_rssi_amp

        self.max_degree = max_degree
        self.extra_edge_factor = extra_edge_factor
        self.max_attempts = max_attempts

        self.hello_interval = hello_interval
        self.update_interval = update_interval
        self.route_timeout = route_timeout
        self.update_payload_limit = update_payload_limit

        self.data_period = data_period

        self.initial_energy = initial_energy
        self.energy_per_tx = energy_per_tx
        self.energy_per_rx = energy_per_rx

        self.time = 0.0
        self.in_flight: Deque[Message] = deque()
        self.next_data_at = 4.0

        # topology
        self.adj: Dict[int, List[int]] = {}
        self.pos: Dict[int, Tuple[float, float]] = {}
        self._build_topology()

        self.nodes: List[Node] = []
        self._init_nodes()

        # metrics
        self.route_changes = 0
        self.ctrl_hello = 0
        self.ctrl_update = 0
        self.data_sent = 0
        self.data_delivered = 0
        self.total_latency = 0.0
        self.total_hops = 0
        self.dropped_no_route = 0
        self.dropped_link = 0

        self.event_log: Deque[str] = deque(maxlen=20)
        self.samples: List[dict] = []

        self._video_recording = False
        self._video_stride = 1
        self._video_max_frames = 120
        self._video_step_counter = 0
        self._video_frames: List[Tuple[float, List[float]]] = []

    def _build_topology(self) -> None:
        self.adj, self.pos = generate_random_topology(
            n_nodes=self.n_nodes,
            gateway_id=self.gateway_id,
            area_w=self.area_w,
            area_h=self.area_h,
            max_degree=self.max_degree,
            seed=self.seed,
            rssi_1m=self.rssi_1m,
            path_loss_n=self.path_loss_n,
            rssi_threshold=self.rssi_threshold,
            extra_edge_factor=self.extra_edge_factor,
            max_attempts=self.max_attempts,
        )

    def _init_nodes(self) -> None:
        self.nodes.clear()
        for i in range(self.n_nodes):
            x, y = self.pos[i]
            node = Node(nid=i, x=x, y=y, energy=self.initial_energy)

            # route self
            node.routes[i] = RouteEntry(
                dest=i,
                next_hop=i,
                hop_count=0,
                seq_num=0,
                last_update=0.0,
                changed=True,
            )
            node.next_hello = self.rng.uniform(0.0, 2.0)
            node.next_update = self.rng.uniform(1.5, 4.0)
            node.seq_num = 0

            self.nodes.append(node)

    def _dist(self, a: Node, b: Node) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _rssi_from_dist_noisy(self, d: float) -> float:
        return _rssi_from_dist(
            d,
            rssi_1m=self.rssi_1m,
            path_loss_n=self.path_loss_n,
            noise_amp=self.noise_rssi_amp,
            rng=self.rng,
        )

    def _has_link(self, u: int, v: int) -> bool:
        return v in self.adj.get(u, [])

    def _broadcast(self, sender_id: int, kind: str, payload: dict) -> None:
        sender = self.nodes[sender_id]
        if sender.energy <= 0.0:
            return
        if sender.energy - self.energy_per_tx < 0.0:
            return
        sender.energy -= self.energy_per_tx

        for r in self.adj.get(sender_id, []):
            if r == sender_id:
                continue
            if self.nodes[r].energy <= 0.0:
                continue
            receiver = self.nodes[r]
            d = self._dist(sender, receiver)
            # propagation: giữ tinh thần wsn_dsdv_sim (delay tăng theo khoảng cách)
            propagation = 0.03 + d / 450.0
            self.in_flight.append(
                Message(
                    kind=kind,
                    src=sender_id,
                    sender=sender_id,
                    dst=r,
                    payload=payload,
                    deliver_time=self.time + propagation,
                )
            )

    def _unicast(self, sender_id: int, next_hop: int, kind: str, payload: dict) -> bool:
        s = self.nodes[sender_id]
        r = self.nodes[next_hop]
        if s.energy <= 0.0 or r.energy <= 0.0:
            return False
        if not self._has_link(sender_id, next_hop):
            return False
        if s.energy - self.energy_per_tx < 0.0:
            return False

        s.energy -= self.energy_per_tx
        d = self._dist(s, r)
        propagation = 0.04 + d / 450.0
        self.in_flight.append(
            Message(
                kind=kind,
                src=payload.get("src", sender_id),
                sender=sender_id,
                dst=next_hop,
                payload=payload,
                deliver_time=self.time + propagation,
            )
        )
        return True

    def _upsert_route(
        self,
        *,
        owner: Node,
        dest: int,
        next_hop: int,
        hop_count: int,
        seq_num: int,
    ) -> None:
        old = owner.routes.get(dest)
        if old is None:
            owner.routes[dest] = RouteEntry(
                dest=dest,
                next_hop=next_hop,
                hop_count=hop_count,
                seq_num=seq_num,
                last_update=self.time,
                changed=True,
            )
            self.route_changes += 1
            return

        # DSDV freshness first: seq_num lớn hơn -> ưu tiên
        if seq_num > old.seq_num:
            old.next_hop = next_hop
            old.hop_count = hop_count
            old.seq_num = seq_num
            old.last_update = self.time
            old.changed = True
            self.route_changes += 1
            return

        # Cùng seq: hop tốt hơn -> cập nhật
        if seq_num == old.seq_num and hop_count < old.hop_count:
            old.next_hop = next_hop
            old.hop_count = hop_count
            old.last_update = self.time
            old.changed = True
            self.route_changes += 1
            return

        # Không cải thiện: chỉ refresh last_update để tránh timeout sớm
        old.last_update = self.time

    def _send_hello(self, n: Node) -> None:
        if n.energy <= 0.0:
            return
        n.seq_num += 2
        payload = {"src": n.nid, "seq_num": n.seq_num}
        self._broadcast(n.nid, "hello", payload)
        self.ctrl_hello += 1
        jitter = self.rng.uniform(-0.6, 0.8)
        n.next_hello = self.time + max(1.5, self.hello_interval + jitter)

    def _send_update(self, n: Node) -> None:
        if n.energy <= 0.0:
            return

        entries: List[Tuple[int, int, int]] = []
        for e in n.routes.values():
            if e.dest == n.nid:
                continue
            if self.time - e.last_update > self.route_timeout:
                continue
            if e.hop_count >= INF_HOPS:
                continue
            entries.append((e.dest, e.hop_count, e.seq_num))

        # Limit payload để mimic embedded (giữ giống wsn_dsdv_sim)
        # ưu tiên route hop ngắn hơn
        entries.sort(key=lambda t: t[1])
        entries = entries[: self.update_payload_limit]

        payload = {"src": n.nid, "entries": entries}
        self._broadcast(n.nid, "update", payload)
        self.ctrl_update += 1

        jitter = self.rng.uniform(-1.2, 2.0)
        n.next_update = self.time + max(3.0, self.update_interval + jitter)

    def _handle_hello(self, receiver: Node, msg: Message) -> None:
        src = msg.payload["src"]
        if src == receiver.nid:
            return

        sender = self.nodes[msg.sender]
        d = self._dist(receiver, sender)
        rssi = self._rssi_from_dist_noisy(d)

        prev = receiver.neighbor_rssi.get(src, rssi)
        receiver.neighbor_rssi[src] = 0.8 * prev + 0.2 * rssi

        self._upsert_route(
            owner=receiver,
            dest=src,
            next_hop=src,
            hop_count=1,
            seq_num=msg.payload["seq_num"],
        )

    def _handle_update(self, receiver: Node, msg: Message) -> None:
        sender_id = msg.sender
        for dest, hop, seq in msg.payload["entries"]:
            if dest == receiver.nid:
                continue

            # candidate hop = hop(sender's view) + 1
            if hop >= INF_HOPS:
                actual_hop = INF_HOPS
            else:
                actual_hop = min(INF_HOPS, int(hop) + 1)

            self._upsert_route(
                owner=receiver,
                dest=dest,
                next_hop=sender_id,
                hop_count=actual_hop,
                seq_num=seq,
            )

    def _forward_data(self, at: Node, payload: dict) -> None:
        dest = payload["dest"]
        r = at.route_to(dest, self.time, self.route_timeout)
        if not r:
            self.dropped_no_route += 1
            return

        payload["hop_count"] += 1
        payload["path"].append(at.nid)

        ok = self._unicast(at.nid, r.next_hop, "data", payload)
        if not ok:
            self.dropped_link += 1

    def _handle_data(self, receiver: Node, msg: Message) -> None:
        p = msg.payload
        if receiver.nid == p["dest"]:
            receiver.recv_data += 1
            self.data_delivered += 1
            self.total_hops += p["hop_count"]
            self.total_latency += self.time - p["created_at"]
            return
        if receiver.energy <= 0.0:
            self.dropped_link += 1
            return
        self._forward_data(receiver, p)

    def _gen_data_packet(self) -> None:
        # chọn src/dst ngẫu nhiên
        src = self.rng.randrange(self.n_nodes)
        dst = self.rng.randrange(self.n_nodes - 1)
        if dst >= src:
            dst += 1

        node = self.nodes[src]
        if node.energy <= 0.0:
            return

        node.sent_data += 1
        self.data_sent += 1

        payload = {
            "src": src,
            "dest": dst,
            "created_at": self.time,
            "seq": int(self.time * 1000.0) + self.rng.randrange(1000),
            "hop_count": 0,
            "path": [src],
        }

        r = node.route_to(dst, self.time, self.route_timeout)
        if not r:
            self.dropped_no_route += 1
            return

        ok = self._unicast(src, r.next_hop, "data", payload)
        if not ok:
            self.dropped_link += 1

    def _process_in_flight(self) -> None:
        while self.in_flight and self.in_flight[0].deliver_time <= self.time:
            msg = self.in_flight.popleft()
            if msg.dst is None:
                continue
            receiver = self.nodes[msg.dst]
            if receiver.energy <= 0.0:
                continue

            # RX cost
            receiver.energy = max(0.0, receiver.energy - self.energy_per_rx)
            if receiver.energy <= 0.0:
                continue

            if msg.kind == "hello":
                self._handle_hello(receiver, msg)
            elif msg.kind == "update":
                self._handle_update(receiver, msg)
            elif msg.kind == "data":
                self._handle_data(receiver, msg)

    def _collect_sample(self) -> None:
        pdr = (self.data_delivered / self.data_sent) if self.data_sent else 0.0
        avg_lat = (self.total_latency / self.data_delivered) if self.data_delivered else 0.0
        avg_hops = (self.total_hops / self.data_delivered) if self.data_delivered else 0.0
        self.samples.append(
            {
                "t": self.time,
                "pdr": pdr,
                "avg_latency_s": avg_lat,
                "avg_hops": avg_hops,
                "hello_packets": self.ctrl_hello,
                "update_packets": self.ctrl_update,
                "route_changes": self.route_changes,
            }
        )

    def step(self) -> None:
        for n in self.nodes:
            if n.energy <= 0.0:
                continue
            if self.time >= n.next_hello:
                self._send_hello(n)
            if self.time >= n.next_update:
                self._send_update(n)

            # expire routes
            for d, e in list(n.routes.items()):
                if d == n.nid:
                    continue
                if self.time - e.last_update > self.route_timeout:
                    e.hop_count = INF_HOPS

        while self.time >= self.next_data_at:
            self._gen_data_packet()
            self.next_data_at += self.data_period

        self._process_in_flight()
        self._collect_sample()

        if self._video_recording and len(self._video_frames) < self._video_max_frames:
            self._video_step_counter += 1
            if self._video_step_counter % self._video_stride == 0:
                self._video_frames.append((self.time, [n.energy for n in self.nodes]))

        self.time += self.dt

    def run(
        self,
        *,
        record_video: bool = False,
        video_stride: int = 1,
        video_max_frames: int = 120,
    ) -> None:
        self._video_recording = record_video
        self._video_stride = max(1, video_stride)
        self._video_max_frames = max(1, video_max_frames)
        self._video_step_counter = 0
        self._video_frames = []
        n_steps = int(self.duration / self.dt)
        for _ in range(n_steps):
            self.step()
        self._video_recording = False

    def summary(self) -> dict:
        pdr = (self.data_delivered / self.data_sent) if self.data_sent else 0.0
        avg_lat = (self.total_latency / self.data_delivered) if self.data_delivered else 0.0
        avg_hops = (self.total_hops / self.data_delivered) if self.data_delivered else 0.0
        return {
            "pdr": pdr,
            "avg_latency_s": avg_lat,
            "avg_hops": avg_hops,
            "hello_packets": self.ctrl_hello,
            "update_packets": self.ctrl_update,
            "data_sent": self.data_sent,
            "data_delivered": self.data_delivered,
            "drop_no_route": self.dropped_no_route,
            "drop_link": self.dropped_link,
            "route_changes": self.route_changes,
        }

    def export_metrics_csv(self, out_path: Path) -> None:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.samples[0].keys()) if self.samples else [])
            if self.samples:
                writer.writeheader()
                writer.writerows(self.samples)

    def export_routing_table_csv(self, out_path: Path, node_id: int = 0) -> None:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        node = self.nodes[node_id]

        rows = []
        for dest, e in sorted(node.routes.items(), key=lambda kv: kv[0]):
            if dest == node_id:
                continue
            if e.hop_count >= INF_HOPS:
                continue
            rows.append(
                {
                    "dest": dest,
                    "next_hop": e.next_hop,
                    "hop_count": e.hop_count,
                    "seq_num": e.seq_num,
                    "last_update": e.last_update,
                }
            )

        with out_path.open("w", newline="", encoding="utf-8") as f:
            fieldnames = ["dest", "next_hop", "hop_count", "seq_num", "last_update"]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Environment-aware DSDV simulator (HELLO+UPDATE+DATA)")

    # run control
    p.add_argument("--nodes", type=int, default=30)
    p.add_argument("--duration", type=float, default=120.0)
    p.add_argument("--dt", type=float, default=1.0)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--out-dir", type=str, default="sim_outputs_env")
    p.add_argument("--focus-node", type=int, default=0)

    # topology / area
    p.add_argument("--area", type=float, default=100.0, help="Square area side length")
    p.add_argument("--area-w", type=float, default=None)
    p.add_argument("--area-h", type=float, default=None)
    p.add_argument("--gateway-id", type=int, default=0)
    p.add_argument("--max-degree", type=int, default=6)
    p.add_argument("--extra-edge-factor", type=float, default=1.4)
    p.add_argument("--max-attempts", type=int, default=300)

    # radio model (RSSI)
    p.add_argument("--rssi-1m", type=float, default=-55.0)
    p.add_argument("--path-loss-n", type=float, default=2.5)
    p.add_argument("--rssi-threshold", type=float, default=-90.0)
    p.add_argument("--noise-rssi-amp", type=float, default=2.0)

    # protocol
    p.add_argument("--hello", type=float, default=5.0, help="HELLO interval (s)")
    p.add_argument("--update", type=float, default=15.0, help="UPDATE interval (s)")
    p.add_argument("--timeout", type=float, default=45.0, help="Route timeout (s)")
    p.add_argument("--update-payload-limit", type=int, default=12)

    # traffic
    p.add_argument("--data-period", type=float, default=2.5)

    # energy
    p.add_argument("--initial-energy", type=float, default=1000.0)
    p.add_argument("--energy-per-tx", type=float, default=0.01)
    p.add_argument("--energy-per-rx", type=float, default=0.005)

    p.add_argument(
        "--no-plots",
        action="store_true",
        help="Không xuất topology.png và metrics_chart.png",
    )
    p.add_argument("--video", action="store_true", help="Xuất animation (simulation.mp4 hoặc .gif)")
    p.add_argument("--video-format", choices=["mp4", "gif"], default="mp4")
    p.add_argument("--fps", type=int, default=8)
    p.add_argument(
        "--video-stride",
        type=int,
        default=1,
        help="Ghi 1 frame animation mỗi N bước thời gian (--dt)",
    )
    p.add_argument("--video-max-frames", type=int, default=150)

    return p.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    sim = DsdvEnvSim(
        n_nodes=args.nodes,
        duration=args.duration,
        dt=args.dt,
        seed=args.seed,
        area=args.area,
        area_w=args.area_w,
        area_h=args.area_h,
        gateway_id=args.gateway_id,
        rssi_1m=args.rssi_1m,
        path_loss_n=args.path_loss_n,
        rssi_threshold=args.rssi_threshold,
        noise_rssi_amp=args.noise_rssi_amp,
        max_degree=args.max_degree,
        extra_edge_factor=args.extra_edge_factor,
        max_attempts=args.max_attempts,
        hello_interval=args.hello,
        update_interval=args.update,
        route_timeout=args.timeout,
        update_payload_limit=args.update_payload_limit,
        data_period=args.data_period,
        initial_energy=args.initial_energy,
        energy_per_tx=args.energy_per_tx,
        energy_per_rx=args.energy_per_rx,
    )

    sim.run(
        record_video=args.video,
        video_stride=args.video_stride,
        video_max_frames=args.video_max_frames,
    )
    summary = sim.summary()

    case_dir = out_dir / f"n{args.nodes}"
    case_dir.mkdir(parents=True, exist_ok=True)

    sim.export_metrics_csv(case_dir / "metrics_timeseries.csv")
    sim.export_routing_table_csv(case_dir / "routing_table_node.csv", node_id=args.focus_node)

    with (case_dir / "summary.txt").open("w", encoding="utf-8") as f:
        for k, v in summary.items():
            f.write(f"{k}: {v}\n")

    if not args.no_plots:
        ok_topo = plot_topology_png(
            adj=sim.adj,
            pos=sim.pos,
            gateway_id=sim.gateway_id,
            area_w=sim.area_w,
            area_h=sim.area_h,
            out_path=case_dir / "topology.png",
        )
        if not ok_topo:
            print("[WARN] Không ghi topology.png (cài: python -m pip install matplotlib)")
        ok_chart = plot_metrics_chart(sim.samples, case_dir / "metrics_chart.png")
        if not ok_chart:
            print("[WARN] Không ghi metrics_chart.png (thiếu matplotlib hoặc chưa có mẫu KPI)")

    if args.video:
        if sim._video_frames:
            ext = ".mp4" if args.video_format == "mp4" else ".gif"
            out_vid = case_dir / f"simulation{ext}"
            ok_vid = save_topology_animation(
                frames=sim._video_frames,
                adj=sim.adj,
                pos=sim.pos,
                gateway_id=sim.gateway_id,
                area_w=sim.area_w,
                area_h=sim.area_h,
                initial_energy=sim.initial_energy,
                out_path=out_vid,
                fps=args.fps,
            )
            if ok_vid and out_vid.exists():
                print(f"[OK] Animation: {out_vid}")
            elif ok_vid and not out_vid.exists():
                gif_fallback = out_vid.with_suffix(".gif")
                if gif_fallback.exists():
                    print(f"[OK] Animation (fallback GIF): {gif_fallback}")
            else:
                print("[WARN] Không ghi được simulation video (matplotlib/pillow/ffmpeg?)")
        else:
            print("[WARN] --video nhưng không có frame (tăng duration hoặc giảm --video-stride)")

    print(
        f"[DONE] N={args.nodes} | PDR={summary['pdr']*100:.1f}% | "
        f"AvgLatency={summary['avg_latency_s']:.3f}s | AvgHops={summary['avg_hops']:.3f}"
    )
    print(f"Saved to: {case_dir}")


if __name__ == "__main__":
    main()

