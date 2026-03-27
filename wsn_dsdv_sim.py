"""
WSN DSDV simulator (inspired by the project's embedded DSDV behavior).

Features:
- **Connected topology:** nodes placed on a grid with spacing ≤ radio range (always one connected component)
- Basic radio model (RSSI from distance)
- DSDV-like proactive routing (HELLO + UPDATE, sequence-based freshness)
- Periodic data traffic generation and packet forwarding by routing table
- Metrics collection: PDR, avg latency, avg hops, control/data packet counters
- Animated visualization export (MP4 if ffmpeg exists, otherwise GIF fallback)
- Routing table snapshots exported to CSV

Usage examples:
    python wsn_dsdv_sim.py --nodes 10 --duration 90 --video
    python wsn_dsdv_sim.py --batch 10,30,50,100 --duration 120 --video
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

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


INF_HOPS = 255


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

    def route_to(self, dest: int, now: float, route_timeout: float) -> Optional[RouteEntry]:
        e = self.routes.get(dest)
        if not e:
            return None
        if now - e.last_update > route_timeout:
            return None
        if e.hop_count >= INF_HOPS:
            return None
        return e


class WsnDsdvSim:
    def __init__(
        self,
        n_nodes: int = 30,
        area_size: float = 100.0,
        radio_range: float = 30.0,
        hello_interval: float = 5.0,
        update_interval: float = 15.0,
        route_timeout: float = 45.0,
        dt: float = 1.0,
        duration: float = 120.0,
        data_period: float = 2.5,
        seed: int = 42,
    ) -> None:
        self.rng = random.Random(seed)
        self.n_nodes = n_nodes
        self.area_size = area_size
        self.radio_range = radio_range
        self.hello_interval = hello_interval
        self.update_interval = update_interval
        self.route_timeout = route_timeout
        self.dt = dt
        self.duration = duration
        self.data_period = data_period
        self.seed = seed

        self.nodes: List[Node] = []
        self.time = 0.0
        self.in_flight: Deque[Message] = deque()
        self.next_data_at = 4.0
        self.route_changes = 0

        self.ctrl_hello = 0
        self.ctrl_update = 0
        self.data_sent = 0
        self.data_delivered = 0
        self.total_latency = 0.0
        self.total_hops = 0
        self.dropped_no_route = 0
        self.dropped_link = 0
        self.event_log: Deque[str] = deque(maxlen=12)
        self.samples: List[dict] = []
        self.active_data_edges: List[Tuple[int, int]] = []

        self._init_nodes()

    def _grid_positions(self) -> List[Tuple[float, float]]:
        """Place N nodes on a 4-connected grid so every node has a path of edges ≤ radio_range.

        Neighbors along the grid are at distance `s` with `s ≤ 0.95 * radio_range`, so the
        geometric graph contains a spanning grid → always connected for N ≥ 1.
        """
        n = self.n_nodes
        area = self.area_size
        r = self.radio_range * 0.95
        if n <= 0:
            return []
        if n == 1:
            return [(area / 2.0, area / 2.0)]

        cols = int(math.ceil(math.sqrt(n)))
        rows = int(math.ceil(n / cols))
        # Max spacing that fits in area along each axis
        sx = area / max(1, cols - 1)
        sy = area / max(1, rows - 1)
        s = min(r, sx, sy)
        total_w = (cols - 1) * s
        total_h = (rows - 1) * s
        ox = (area - total_w) / 2.0
        oy = (area - total_h) / 2.0

        positions: List[Tuple[float, float]] = []
        for i in range(n):
            row = i // cols
            col = i % cols
            x = ox + col * s
            y = oy + row * s
            positions.append((x, y))
        return positions

    def _init_nodes(self) -> None:
        self.nodes.clear()
        positions = self._grid_positions()
        for i in range(self.n_nodes):
            x, y = positions[i]
            node = Node(nid=i, x=x, y=y)
            node.next_hello = self.rng.uniform(0.0, 2.0)
            node.next_update = self.rng.uniform(1.5, 4.0)
            # Self route
            node.routes[i] = RouteEntry(
                dest=i,
                next_hop=i,
                hop_count=0,
                seq_num=0,
                last_update=0.0,
                changed=True,
            )
            self.nodes.append(node)

    def _dist(self, a: Node, b: Node) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _in_range(self, a: Node, b: Node) -> bool:
        return self._dist(a, b) <= self.radio_range

    def _rssi_from_dist(self, d: float) -> float:
        # Simple path loss style RSSI model with mild noise.
        if d < 0.5:
            d = 0.5
        base = -35.0 - 20.0 * math.log10(d)
        noise = self.rng.uniform(-2.0, 2.0)
        return max(-100.0, min(-30.0, base + noise))

    def _broadcast(self, sender_id: int, kind: str, payload: dict) -> None:
        sender = self.nodes[sender_id]
        for r in self.nodes:
            if r.nid == sender_id:
                continue
            if not self._in_range(sender, r):
                continue
            d = self._dist(sender, r)
            propagation = 0.03 + d / 450.0
            self.in_flight.append(
                Message(
                    kind=kind,
                    src=sender_id,
                    sender=sender_id,
                    dst=r.nid,
                    payload=payload,
                    deliver_time=self.time + propagation,
                )
            )

    def _unicast(self, sender_id: int, next_hop: int, kind: str, payload: dict) -> bool:
        s = self.nodes[sender_id]
        r = self.nodes[next_hop]
        if not self._in_range(s, r):
            return False
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

    def _send_hello(self, n: Node) -> None:
        n.seq_num += 2
        payload = {"src": n.nid, "seq_num": n.seq_num}
        self._broadcast(n.nid, "hello", payload)
        self.ctrl_hello += 1
        jitter = self.rng.uniform(-0.6, 0.8)
        n.next_hello = self.time + max(1.5, self.hello_interval + jitter)

    def _send_update(self, n: Node) -> None:
        entries = []
        for e in n.routes.values():
            if self.time - e.last_update > self.route_timeout:
                continue
            if e.dest == n.nid:
                continue
            entries.append((e.dest, e.hop_count, e.seq_num))
        # Limit payload to mimic embedded code
        entries = entries[:12]
        payload = {"src": n.nid, "entries": entries}
        self._broadcast(n.nid, "update", payload)
        self.ctrl_update += 1
        jitter = self.rng.uniform(-1.2, 2.0)
        n.next_update = self.time + max(3.0, self.update_interval + jitter)

    def _upsert_route(
        self,
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
        # DSDV freshness first
        if seq_num > old.seq_num:
            old.next_hop = next_hop
            old.hop_count = hop_count
            old.seq_num = seq_num
            old.last_update = self.time
            old.changed = True
            self.route_changes += 1
            return
        if seq_num == old.seq_num and hop_count < old.hop_count:
            old.next_hop = next_hop
            old.hop_count = hop_count
            old.last_update = self.time
            old.changed = True
            self.route_changes += 1
            return
        old.last_update = self.time

    def _handle_hello(self, receiver: Node, msg: Message) -> None:
        src = msg.payload["src"]
        if src == receiver.nid:
            return
        sender = self.nodes[msg.sender]
        d = self._dist(receiver, sender)
        rssi = self._rssi_from_dist(d)
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
        for dest, hop, seq in msg.payload["entries"]:
            if dest == receiver.nid:
                continue
            next_hop = msg.sender
            if hop >= INF_HOPS:
                actual_hop = INF_HOPS
            else:
                actual_hop = min(INF_HOPS, hop + 1)
            self._upsert_route(receiver, dest, next_hop, actual_hop, seq)

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
            return
        self.active_data_edges.append((at.nid, r.next_hop))

    def _handle_data(self, receiver: Node, msg: Message) -> None:
        p = msg.payload
        if receiver.nid == p["dest"]:
            receiver.recv_data += 1
            self.data_delivered += 1
            self.total_hops += p["hop_count"]
            self.total_latency += self.time - p["created_at"]
            self.event_log.append(
                f"t={self.time:5.1f}s: data {p['src']}->{p['dest']} delivered, hops={p['hop_count']}"
            )
            return
        self._forward_data(receiver, p)

    def _gen_data_packet(self) -> None:
        src = self.rng.randrange(self.n_nodes)
        dst = self.rng.randrange(self.n_nodes - 1)
        if dst >= src:
            dst += 1
        node = self.nodes[src]
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
            self.event_log.append(
                f"t={self.time:5.1f}s: drop {src}->{dst} (no route)"
            )
            return
        ok = self._unicast(src, r.next_hop, "data", payload)
        if not ok:
            self.dropped_link += 1
            self.event_log.append(
                f"t={self.time:5.1f}s: drop {src}->{dst} (link break)"
            )
            return
        self.active_data_edges.append((src, r.next_hop))

    def _process_in_flight(self) -> None:
        while self.in_flight and self.in_flight[0].deliver_time <= self.time:
            msg = self.in_flight.popleft()
            if msg.dst is None:
                continue
            receiver = self.nodes[msg.dst]
            if msg.kind == "hello":
                self._handle_hello(receiver, msg)
            elif msg.kind == "update":
                self._handle_update(receiver, msg)
            elif msg.kind == "data":
                self._handle_data(receiver, msg)

    def step(self) -> None:
        self.active_data_edges = []
        # generate control traffic
        for n in self.nodes:
            if self.time >= n.next_hello:
                self._send_hello(n)
            if self.time >= n.next_update:
                self._send_update(n)
            # expire old routes
            for d in list(n.routes.keys()):
                if d == n.nid:
                    continue
                if self.time - n.routes[d].last_update > self.route_timeout:
                    n.routes[d].hop_count = INF_HOPS
        # data traffic
        while self.time >= self.next_data_at:
            self._gen_data_packet()
            self.next_data_at += self.data_period

        self._process_in_flight()
        self._collect_sample()
        self.time += self.dt

    def run(self) -> None:
        n_steps = int(self.duration / self.dt)
        for _ in range(n_steps):
            self.step()

    def _collect_sample(self) -> None:
        pdr = (self.data_delivered / self.data_sent) if self.data_sent else 0.0
        avg_lat = (self.total_latency / self.data_delivered) if self.data_delivered else 0.0
        avg_hops = (self.total_hops / self.data_delivered) if self.data_delivered else 0.0
        self.samples.append(
            {
                "t": self.time,
                "pdr": pdr,
                "avg_latency": avg_lat,
                "avg_hops": avg_hops,
                "hello": self.ctrl_hello,
                "update": self.ctrl_update,
                "route_changes": self.route_changes,
            }
        )

    def summary(self) -> dict:
        pdr = (self.data_delivered / self.data_sent) if self.data_sent else 0.0
        avg_lat = (self.total_latency / self.data_delivered) if self.data_delivered else 0.0
        avg_hops = (self.total_hops / self.data_delivered) if self.data_delivered else 0.0
        return {
            "nodes": self.n_nodes,
            "duration": self.duration,
            "sent": self.data_sent,
            "delivered": self.data_delivered,
            "pdr": pdr,
            "avg_latency_s": avg_lat,
            "avg_hops": avg_hops,
            "hello_packets": self.ctrl_hello,
            "update_packets": self.ctrl_update,
            "drop_no_route": self.dropped_no_route,
            "drop_link": self.dropped_link,
            "route_changes": self.route_changes,
        }

    def export_routing_table_csv(self, out_file: Path, node_id: int = 0) -> None:
        node = self.nodes[node_id]
        out_file.parent.mkdir(parents=True, exist_ok=True)
        with out_file.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["owner_node", "dest", "next_hop", "hop_count", "seq_num", "age_s"])
            for dest in sorted(node.routes.keys()):
                e = node.routes[dest]
                age = max(0.0, self.time - e.last_update)
                w.writerow([node_id, dest, e.next_hop, e.hop_count, e.seq_num, round(age, 3)])

    def export_metrics_csv(self, out_file: Path) -> None:
        out_file.parent.mkdir(parents=True, exist_ok=True)
        with out_file.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(self.samples[0].keys()))
            w.writeheader()
            for s in self.samples:
                w.writerow(s)

    def animate(self, out_file: Path, focus_node: int = 0, fps: int = 8) -> None:
        out_file.parent.mkdir(parents=True, exist_ok=True)
        fig, (ax_net, ax_info) = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={"width_ratios": [2, 1]})

        xs = [n.x for n in self.nodes]
        ys = [n.y for n in self.nodes]

        # Precompute frames by replaying a fresh sim with same seed/settings
        replay = WsnDsdvSim(
            n_nodes=self.n_nodes,
            area_size=self.area_size,
            radio_range=self.radio_range,
            hello_interval=self.hello_interval,
            update_interval=self.update_interval,
            route_timeout=self.route_timeout,
            dt=self.dt,
            duration=self.duration,
            data_period=self.data_period,
            seed=self.seed,
        )
        frames = int(self.duration / self.dt)
        frame_states = []
        for _ in range(frames):
            replay.step()
            frame_states.append(
                {
                    "t": replay.time,
                    "summary": replay.summary(),
                    "routes": dict(replay.nodes[focus_node].routes),
                    "active_edges": list(replay.active_data_edges),
                    "pdr": replay.samples[-1]["pdr"],
                    "lat": replay.samples[-1]["avg_latency"],
                    "hops": replay.samples[-1]["avg_hops"],
                }
            )

        def draw_frame(i: int) -> None:
            s = frame_states[i]
            ax_net.clear()
            ax_info.clear()

            ax_net.set_title(f"WSN DSDV Simulation | t={s['t']:.1f}s | N={self.n_nodes}")
            ax_net.set_xlim(0, self.area_size)
            ax_net.set_ylim(0, self.area_size)
            ax_net.set_xlabel("X")
            ax_net.set_ylabel("Y")

            # Draw light connectivity lines
            for a in self.nodes:
                for b in self.nodes:
                    if a.nid < b.nid and self._in_range(a, b):
                        ax_net.plot([a.x, b.x], [a.y, b.y], color="#d0d0d0", linewidth=0.5, alpha=0.35)

            ax_net.scatter(xs, ys, c="#1f77b4", s=40)
            fx, fy = self.nodes[focus_node].x, self.nodes[focus_node].y
            ax_net.scatter([fx], [fy], c="orange", s=130, edgecolors="black", linewidth=1.2)
            for n in self.nodes:
                ax_net.text(n.x + 0.6, n.y + 0.6, str(n.nid), fontsize=7)

            # active forwarding edges
            for u, v in s["active_edges"]:
                nu, nv = self.nodes[u], self.nodes[v]
                ax_net.plot([nu.x, nv.x], [nu.y, nv.y], color="red", linewidth=2.0, alpha=0.8)

            ax_info.axis("off")
            sm = s["summary"]
            text = [
                f"Node focus: {focus_node}",
                "",
                f"Data sent: {sm['sent']}",
                f"Data delivered: {sm['delivered']}",
                f"PDR: {s['pdr']*100:.1f}%",
                f"Avg latency: {s['lat']:.3f}s",
                f"Avg hops: {s['hops']:.2f}",
                f"HELLO packets: {sm['hello_packets']}",
                f"UPDATE packets: {sm['update_packets']}",
                f"Route changes: {sm['route_changes']}",
                "",
                "Routing table sample (focus node):",
            ]
            rows = []
            for d in sorted(s["routes"].keys()):
                if d == focus_node:
                    continue
                e = s["routes"][d]
                if e.hop_count >= INF_HOPS:
                    continue
                rows.append(f"d={d:3d} via {e.next_hop:3d} hops={e.hop_count:2d} seq={e.seq_num:4d}")
                if len(rows) >= 12:
                    break
            if not rows:
                rows = ["(no valid routes yet)"]
            text.extend(rows)
            ax_info.text(0.02, 0.98, "\n".join(text), va="top", family="monospace", fontsize=9)

        ani = FuncAnimation(fig, draw_frame, frames=frames, interval=1000 / fps, repeat=False)
        if out_file.suffix.lower() == ".mp4":
            try:
                ani.save(str(out_file), fps=fps, dpi=130)
            except Exception:
                fallback = out_file.with_suffix(".gif")
                ani.save(str(fallback), fps=fps, dpi=120, writer="pillow")
        else:
            ani.save(str(out_file), fps=fps, dpi=120, writer="pillow")
        plt.close(fig)


def run_case(
    n_nodes: int,
    args: argparse.Namespace,
    out_dir: Path,
) -> dict:
    sim = WsnDsdvSim(
        n_nodes=n_nodes,
        area_size=args.area,
        radio_range=args.range,
        hello_interval=args.hello,
        update_interval=args.update,
        route_timeout=args.timeout,
        dt=args.dt,
        duration=args.duration,
        data_period=args.data_period,
        seed=args.seed,
    )
    sim.run()
    summary = sim.summary()

    case_dir = out_dir / f"n{n_nodes}"
    case_dir.mkdir(parents=True, exist_ok=True)
    sim.export_metrics_csv(case_dir / "metrics_timeseries.csv")
    sim.export_routing_table_csv(case_dir / "routing_table_node0.csv", node_id=0)

    if args.video:
        ext = ".mp4" if args.video_format == "mp4" else ".gif"
        sim.animate(case_dir / f"simulation{ext}", focus_node=args.focus_node, fps=args.fps)

    with (case_dir / "summary.txt").open("w", encoding="utf-8") as f:
        for k, v in summary.items():
            f.write(f"{k}: {v}\n")
    return summary


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="WSN DSDV simulation with routing/metrics animation")
    p.add_argument("--nodes", type=int, default=30, help="Node count for single run")
    p.add_argument("--batch", type=str, default="", help="Comma-separated node counts (e.g. 10,30,50,100)")
    p.add_argument("--duration", type=float, default=120.0, help="Simulation duration (s)")
    p.add_argument("--dt", type=float, default=1.0, help="Time step (s)")
    p.add_argument("--area", type=float, default=100.0, help="Square area side length")
    p.add_argument("--range", type=float, default=30.0, help="Radio range")
    p.add_argument("--hello", type=float, default=5.0, help="HELLO interval (s)")
    p.add_argument("--update", type=float, default=15.0, help="UPDATE interval (s)")
    p.add_argument("--timeout", type=float, default=45.0, help="Route timeout (s)")
    p.add_argument("--data-period", type=float, default=2.5, help="Data generation period (s)")
    p.add_argument("--seed", type=int, default=42, help="Random seed")
    p.add_argument("--video", action="store_true", help="Export animation video")
    p.add_argument("--video-format", choices=["mp4", "gif"], default="mp4")
    p.add_argument("--fps", type=int, default=8)
    p.add_argument("--focus-node", type=int, default=0, help="Node ID shown with routing table panel")
    p.add_argument("--out-dir", type=str, default="sim_outputs", help="Output directory")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cases = [args.nodes]
    if args.batch.strip():
        cases = [int(x.strip()) for x in args.batch.split(",") if x.strip()]

    all_summary = []
    for n in cases:
        summary = run_case(n, args, out_dir)
        all_summary.append(summary)
        print(f"[DONE] N={n} | PDR={summary['pdr']*100:.1f}% | AvgLatency={summary['avg_latency_s']:.3f}s")

    summary_csv = out_dir / "batch_summary.csv"
    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_summary[0].keys()))
        writer.writeheader()
        for row in all_summary:
            writer.writerow(row)
    print(f"\nSaved summary: {summary_csv}")
    print("Per-case files: sim_outputs/n*/{summary.txt,metrics_timeseries.csv,routing_table_node0.csv,simulation.*}")


if __name__ == "__main__":
    main()

