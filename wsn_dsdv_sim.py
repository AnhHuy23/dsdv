"""
WSN simulation: DSDV routing inspired by this repo's firmware (chat_cli.c).

What is modeled (basic environment):
- Random geometric graph topology (nodes in a square area)
- Radio range + RSSI as a function of distance (with small noise)
- Proactive DSDV: HELLO (TTL=1), UPDATE (flood with role-adaptive TTL cap)
- Multi-hop DATA forwarding based on routing table
- MCDS-like BACKBONE/LEAF role selection (degree + avg RSSI + HELLO-PDR proxy)

Outputs:
- Batch summary CSV (10..100 nodes by default)
- Per-case metrics timeseries CSV
- Routing table snapshot CSV (node0)
- Animation video (MP4 if possible, else GIF)

Run:
  python wsn_dsdv_sim.py                 # auto sweep 10..100
  python wsn_dsdv_sim.py --nodes 30      # single case
  python wsn_dsdv_sim.py --auto          # explicit auto sweep 10..100
  python wsn_dsdv_sim.py --auto --step 5 # 10,15,...,100
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


class Role:
    UNKNOWN = 0
    BACKBONE = 1
    LEAF = 2


@dataclass
class RouteEntry:
    dest: int
    next_hop: int
    hop_count: int
    seq_num: int
    last_update: float
    changed: bool = False


@dataclass
class NeighborInfo:
    addr: int
    degree: int
    role: int
    avg_rssi: float
    last_seen: float
    hello_rx_count: int = 0
    pdr_window_start: float = 0.0


@dataclass
class Message:
    kind: str  # hello | update | data
    origin: int  # original source of this message flood
    sender: int  # immediate sender (hop)
    dst: Optional[int]  # unicast destination (next hop) for link-layer delivery
    ttl: int
    seq: int
    payload: dict
    deliver_time: float


@dataclass
class Node:
    nid: int
    x: float
    y: float
    seq_num: int = 0
    role: int = Role.UNKNOWN
    degree: int = 0
    backbone_score: int = 0
    routes: Dict[int, RouteEntry] = field(default_factory=dict)
    neighbor_rssi: Dict[int, float] = field(default_factory=dict)
    neighbor_bb: Dict[int, NeighborInfo] = field(default_factory=dict)
    next_hello: float = 0.0
    next_update: float = 0.0
    next_backbone_eval: float = 0.0

    def route_to(self, dest: int, now: float, timeout: float) -> Optional[RouteEntry]:
        e = self.routes.get(dest)
        if not e:
            return None
        if e.hop_count >= INF_HOPS:
            return None
        if now - e.last_update > timeout:
            return None
        return e


class WsnDsdvSim:
    def __init__(
        self,
        n_nodes: int,
        *,
        seed: int,
        area: float,
        radio_range: float,
        dt: float,
        duration: float,
        hello_base_s: float,
        update_base_s: float,
        route_timeout_s: float,
        neighbor_window_s: float,
        backbone_eval_s: float,
        backbone_initial_delay_s: float,
        data_period_s: float,
    ) -> None:
        self.rng = random.Random(seed)
        self.n_nodes = n_nodes
        self.area = area
        self.radio_range = radio_range
        self.dt = dt
        self.duration = duration

        self.hello_base_s = hello_base_s
        self.update_base_s = update_base_s
        self.route_timeout_s = route_timeout_s
        self.neighbor_window_s = neighbor_window_s
        self.backbone_eval_s = backbone_eval_s
        self.backbone_initial_delay_s = backbone_initial_delay_s
        self.data_period_s = data_period_s

        self.nodes: List[Node] = []
        self.time = 0.0
        self.in_flight: Deque[Message] = deque()

        self.ctrl_hello = 0
        self.ctrl_update = 0
        self.data_sent = 0
        self.data_delivered = 0
        self.drop_no_route = 0
        self.drop_link = 0
        self.total_latency = 0.0
        self.total_hops = 0
        self.route_changes = 0

        self.active_edges: List[Tuple[int, int]] = []
        self.samples: List[dict] = []

        self._next_data_at = 6.0
        self._msg_seq = 1
        self._seen_flood: Dict[Tuple[str, int, int], float] = {}  # (kind, origin, seq)->last_time

        self._init_nodes()

    def _init_nodes(self) -> None:
        self.nodes = []
        for i in range(self.n_nodes):
            x = self.rng.uniform(0, self.area)
            y = self.rng.uniform(0, self.area)
            n = Node(nid=i, x=x, y=y)
            n.next_hello = self.rng.uniform(0.0, 2.0)
            n.next_update = self.rng.uniform(2.0, 5.0)
            n.next_backbone_eval = self.backbone_initial_delay_s + self.rng.uniform(0.0, 2.5)
            # self route
            n.routes[i] = RouteEntry(dest=i, next_hop=i, hop_count=0, seq_num=0, last_update=0.0, changed=True)
            self.nodes.append(n)

    def _dist(self, a: Node, b: Node) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _in_range(self, a: Node, b: Node) -> bool:
        return self._dist(a, b) <= self.radio_range

    def _rssi_from_dist(self, d: float) -> float:
        # Basic log-distance path loss with noise
        d = max(0.7, d)
        base = -35.0 - 20.0 * math.log10(d)
        return max(-100.0, min(-30.0, base + self.rng.uniform(-2.0, 2.0)))

    def _schedule_unicast(self, origin: int, sender: int, dst: int, kind: str, ttl: int, seq: int, payload: dict) -> bool:
        s = self.nodes[sender]
        r = self.nodes[dst]
        if not self._in_range(s, r):
            return False
        d = self._dist(s, r)
        delay = 0.03 + d / 450.0
        self.in_flight.append(
            Message(kind=kind, origin=origin, sender=sender, dst=dst, ttl=ttl, seq=seq, payload=payload, deliver_time=self.time + delay)
        )
        return True

    def _flood(self, origin: int, sender: int, kind: str, ttl: int, seq: int, payload: dict) -> None:
        if ttl <= 0:
            return
        s = self.nodes[sender]
        for r in self.nodes:
            if r.nid == sender:
                continue
            if not self._in_range(s, r):
                continue
            self._schedule_unicast(origin, sender, r.nid, kind, ttl, seq, payload)

    def _hello_ttl(self, node: Node) -> int:
        return 1

    def _update_ttl_cap(self, node: Node) -> int:
        # match firmware: backbone higher cap, leaf lower
        return 4 if node.role == Role.BACKBONE else 2

    def _should_relay(self, node: Node) -> bool:
        return node.role == Role.BACKBONE

    def _upsert_route(self, owner: Node, dest: int, next_hop: int, hop_count: int, seq_num: int) -> None:
        old = owner.routes.get(dest)
        if old is None:
            owner.routes[dest] = RouteEntry(dest=dest, next_hop=next_hop, hop_count=hop_count, seq_num=seq_num, last_update=self.time, changed=True)
            self.route_changes += 1
            return
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

    def _expire_routes(self, node: Node) -> None:
        for d, e in list(node.routes.items()):
            if d == node.nid:
                continue
            if self.time - e.last_update > self.route_timeout_s:
                e.hop_count = INF_HOPS

    def _recalc_degree(self, node: Node) -> int:
        active = 0
        for nb, last_rssi in list(node.neighbor_rssi.items()):
            _ = last_rssi
            info = node.neighbor_bb.get(nb)
            if info and (self.time - info.last_seen) <= self.neighbor_window_s:
                active += 1
        node.degree = active
        return active

    def _avg_neighbor_rssi(self, node: Node) -> float:
        vals = []
        for nb, rssi in node.neighbor_rssi.items():
            info = node.neighbor_bb.get(nb)
            if info and (self.time - info.last_seen) <= self.neighbor_window_s:
                vals.append(rssi)
        return sum(vals) / len(vals) if vals else -127.0

    def _avg_neighbor_pdr(self, node: Node) -> int:
        # proxy similar to firmware: based on HELLOs received in window vs expected
        total = 0
        count = 0
        expected_hello_s = 12.0
        for info in node.neighbor_bb.values():
            if (self.time - info.last_seen) > self.neighbor_window_s:
                continue
            window = max(1e-6, self.time - info.pdr_window_start)
            expected = max(1, int(window / expected_hello_s))
            pdr = int((info.hello_rx_count * 100) / expected)
            pdr = max(0, min(100, pdr))
            total += pdr
            count += 1
        return int(total / count) if count else 0

    def _score(self, degree: int, avg_rssi: float, pdr: int) -> int:
        rssi_score = int(avg_rssi + 100)
        rssi_score = max(0, min(70, rssi_score))
        pdr_bonus = int(pdr * 30 / 100)
        return degree * 100 + rssi_score + pdr_bonus

    def _backbone_evaluate(self, node: Node) -> None:
        # thresholds aligned to firmware constants (seconds conversion already)
        BACKBONE_RSSI_THRESHOLD = -70.0
        BACKBONE_RSSI_REJECT = -80.0
        BACKBONE_MIN_DEGREE = 3
        BACKBONE_PDR_REJECT = 50

        self._recalc_degree(node)
        avg_rssi = self._avg_neighbor_rssi(node)
        avg_pdr = self._avg_neighbor_pdr(node)

        old_role = node.role
        if node.degree < 2 or avg_rssi < BACKBONE_RSSI_REJECT or avg_pdr < BACKBONE_PDR_REJECT:
            node.role = Role.LEAF
            node.backbone_score = 0
        else:
            my_score = self._score(node.degree, avg_rssi, avg_pdr)
            node.backbone_score = my_score
            i_am_highest = True
            # compare with 1-hop neighbors using their advertised degree/role and our measured RSSI
            for nb_id, nb in node.neighbor_bb.items():
                if (self.time - nb.last_seen) > self.neighbor_window_s:
                    continue
                # estimate neighbor pdr similarly
                window = max(1e-6, self.time - nb.pdr_window_start)
                expected = max(1, int(window / 12.0))
                nb_pdr = int((nb.hello_rx_count * 100) / expected)
                nb_pdr = max(0, min(100, nb_pdr))
                nb_score = self._score(nb.degree, nb.avg_rssi, nb_pdr)
                if nb_score > my_score or (nb_score == my_score and nb_id < node.nid):
                    i_am_highest = False
                    break
            if i_am_highest and node.degree >= BACKBONE_MIN_DEGREE and avg_rssi >= BACKBONE_RSSI_THRESHOLD:
                node.role = Role.BACKBONE
            else:
                node.role = Role.LEAF

            # connectivity check: if leaf but no backbone neighbor, force highest to backbone
            if node.role == Role.LEAF:
                has_backbone_nb = any(
                    (nb.role == Role.BACKBONE and (self.time - nb.last_seen) <= self.neighbor_window_s)
                    for nb in node.neighbor_bb.values()
                )
                if not has_backbone_nb and i_am_highest and node.degree >= 1:
                    node.role = Role.BACKBONE

        if node.role != old_role:
            # emulate firmware effect: role changes influence TTL/relaying immediately
            pass

    def _send_hello(self, node: Node) -> None:
        node.seq_num += 2
        self._recalc_degree(node)
        payload = {"src": node.nid, "seq_num": node.seq_num, "my_degree": node.degree, "my_role": node.role}
        seq = self._msg_seq
        self._msg_seq += 1
        self._flood(origin=node.nid, sender=node.nid, kind="hello", ttl=self._hello_ttl(node), seq=seq, payload=payload)
        self.ctrl_hello += 1
        # adaptive hello like firmware (simplified): more routes -> slower
        active_routes = sum(1 for e in node.routes.values() if e.hop_count < INF_HOPS and e.dest != node.nid)
        base = min(15.0, 8.0 + active_routes * 0.3)
        node.next_hello = self.time + base + self.rng.uniform(0.0, 3.0)

    def _send_update(self, node: Node) -> None:
        entries = []
        for e in node.routes.values():
            if e.dest == node.nid:
                continue
            if self.time - e.last_update > self.route_timeout_s:
                continue
            entries.append((e.dest, e.hop_count, e.seq_num))
        entries = entries[:12]
        payload = {"src": node.nid, "entries": entries}
        seq = self._msg_seq
        self._msg_seq += 1
        self._flood(origin=node.nid, sender=node.nid, kind="update", ttl=self._update_ttl_cap(node), seq=seq, payload=payload)
        self.ctrl_update += 1
        node.next_update = self.time + max(3.0, self.update_base_s + self.rng.uniform(-1.5, 2.5))

    def _handle_hello(self, receiver: Node, msg: Message) -> None:
        p = msg.payload
        src = int(p["src"])
        if src == receiver.nid:
            return
        # update RSSI + neighbor_bb cache
        d = self._dist(receiver, self.nodes[msg.sender])
        rssi = self._rssi_from_dist(d)
        prev = receiver.neighbor_rssi.get(src, rssi)
        receiver.neighbor_rssi[src] = 0.8 * prev + 0.2 * rssi

        info = receiver.neighbor_bb.get(src)
        if info is None:
            info = NeighborInfo(addr=src, degree=int(p["my_degree"]), role=int(p["my_role"]), avg_rssi=receiver.neighbor_rssi[src], last_seen=self.time)
            info.pdr_window_start = self.time
            info.hello_rx_count = 1
            receiver.neighbor_bb[src] = info
        else:
            if (self.time - info.pdr_window_start) > 60.0:
                info.pdr_window_start = self.time
                info.hello_rx_count = 1
            else:
                info.hello_rx_count += 1
            info.degree = int(p["my_degree"])
            info.role = int(p["my_role"])
            info.avg_rssi = receiver.neighbor_rssi[src]
            info.last_seen = self.time

        # DSDV: upsert direct neighbor route
        self._upsert_route(receiver, src, src, 1, int(p["seq_num"]))

    def _handle_update(self, receiver: Node, msg: Message) -> None:
        p = msg.payload
        sender = msg.sender
        for dest, hop, seq in p["entries"]:
            if dest == receiver.nid:
                continue
            hop = int(hop)
            seq = int(seq)
            if hop >= INF_HOPS:
                actual = INF_HOPS
            else:
                actual = min(INF_HOPS, hop + 1)
            self._upsert_route(receiver, int(dest), sender, actual, seq)

    def _forward_flood_if_needed(self, receiver: Node, msg: Message) -> None:
        if msg.ttl <= 1:
            return
        # emulate leaf relay off: only backbone forwards UPDATE
        if msg.kind == "update" and not self._should_relay(receiver):
            return
        key = (msg.kind, msg.origin, msg.seq)
        if key in self._seen_flood and (self.time - self._seen_flood[key]) < 5.0:
            return
        self._seen_flood[key] = self.time
        self._flood(origin=msg.origin, sender=receiver.nid, kind=msg.kind, ttl=msg.ttl - 1, seq=msg.seq, payload=msg.payload)

    def _send_data(self) -> None:
        src = self.rng.randrange(self.n_nodes)
        dst = self.rng.randrange(self.n_nodes - 1)
        if dst >= src:
            dst += 1
        s = self.nodes[src]
        self.data_sent += 1
        r = s.route_to(dst, self.time, self.route_timeout_s)
        if not r:
            self.drop_no_route += 1
            return
        payload = {"src": src, "dest": dst, "created_at": self.time, "hop_count": 0, "path": [src]}
        ok = self._schedule_unicast(origin=src, sender=src, dst=r.next_hop, kind="data", ttl=10, seq=self._msg_seq, payload=payload)
        self._msg_seq += 1
        if not ok:
            self.drop_link += 1
            return
        self.active_edges.append((src, r.next_hop))

    def _forward_data(self, receiver: Node, msg: Message) -> None:
        p = msg.payload
        dest = int(p["dest"])
        if receiver.nid == dest:
            self.data_delivered += 1
            self.total_latency += self.time - float(p["created_at"])
            self.total_hops += int(p["hop_count"])
            return
        r = receiver.route_to(dest, self.time, self.route_timeout_s)
        if not r:
            self.drop_no_route += 1
            return
        p["hop_count"] += 1
        p["path"].append(receiver.nid)
        ok = self._schedule_unicast(origin=msg.origin, sender=receiver.nid, dst=r.next_hop, kind="data", ttl=msg.ttl - 1, seq=msg.seq, payload=p)
        if not ok:
            self.drop_link += 1
            return
        self.active_edges.append((receiver.nid, r.next_hop))

    def _process_in_flight(self) -> None:
        self.in_flight = deque(sorted(self.in_flight, key=lambda m: m.deliver_time))
        while self.in_flight and self.in_flight[0].deliver_time <= self.time:
            msg = self.in_flight.popleft()
            if msg.dst is None:
                continue
            receiver = self.nodes[msg.dst]
            if msg.kind == "hello":
                self._handle_hello(receiver, msg)
                # HELLO TTL=1 so no forwarding
            elif msg.kind == "update":
                self._handle_update(receiver, msg)
                self._forward_flood_if_needed(receiver, msg)
            elif msg.kind == "data":
                self._forward_data(receiver, msg)

    def _collect_sample(self) -> None:
        pdr = (self.data_delivered / self.data_sent) if self.data_sent else 0.0
        avg_lat = (self.total_latency / self.data_delivered) if self.data_delivered else 0.0
        avg_hops = (self.total_hops / self.data_delivered) if self.data_delivered else 0.0
        backbone_cnt = sum(1 for n in self.nodes if n.role == Role.BACKBONE)
        self.samples.append(
            {
                "t": round(self.time, 3),
                "pdr": pdr,
                "avg_latency_s": avg_lat,
                "avg_hops": avg_hops,
                "hello_packets": self.ctrl_hello,
                "update_packets": self.ctrl_update,
                "route_changes": self.route_changes,
                "backbone_nodes": backbone_cnt,
            }
        )

    def step(self) -> None:
        self.active_edges = []
        # timers
        for n in self.nodes:
            self._expire_routes(n)
            if self.time >= n.next_backbone_eval:
                self._backbone_evaluate(n)
                n.next_backbone_eval = self.time + self.backbone_eval_s + self.rng.uniform(0.0, 2.0)
            if self.time >= n.next_hello:
                self._send_hello(n)
            if self.time >= n.next_update:
                self._send_update(n)

        while self.time >= self._next_data_at:
            self._send_data()
            self._next_data_at += self.data_period_s

        self._process_in_flight()
        self._collect_sample()
        self.time += self.dt

    def run(self) -> None:
        steps = int(self.duration / self.dt)
        for _ in range(steps):
            self.step()

    def summary(self) -> dict:
        pdr = (self.data_delivered / self.data_sent) if self.data_sent else 0.0
        avg_lat = (self.total_latency / self.data_delivered) if self.data_delivered else 0.0
        avg_hops = (self.total_hops / self.data_delivered) if self.data_delivered else 0.0
        return {
            "nodes": self.n_nodes,
            "duration_s": self.duration,
            "sent": self.data_sent,
            "delivered": self.data_delivered,
            "pdr": pdr,
            "avg_latency_s": avg_lat,
            "avg_hops": avg_hops,
            "hello_packets": self.ctrl_hello,
            "update_packets": self.ctrl_update,
            "drop_no_route": self.drop_no_route,
            "drop_link": self.drop_link,
            "route_changes": self.route_changes,
            "backbone_nodes": sum(1 for n in self.nodes if n.role == Role.BACKBONE),
        }

    def export_metrics_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(self.samples[0].keys()))
            w.writeheader()
            w.writerows(self.samples)

    def export_routing_table_csv(self, path: Path, node_id: int = 0) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        n = self.nodes[node_id]
        with path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["owner", "dest", "next_hop", "hop_count", "seq_num", "age_s"])
            for dest in sorted(n.routes.keys()):
                e = n.routes[dest]
                age = max(0.0, self.time - e.last_update)
                w.writerow([node_id, dest, e.next_hop, e.hop_count, e.seq_num, round(age, 3)])

    def animate(self, out_file: Path, focus_node: int, fps: int) -> None:
        out_file.parent.mkdir(parents=True, exist_ok=True)

        # replay for deterministic frames
        replay = WsnDsdvSim(
            self.n_nodes,
            seed=123,
            area=self.area,
            radio_range=self.radio_range,
            dt=self.dt,
            duration=self.duration,
            hello_base_s=self.hello_base_s,
            update_base_s=self.update_base_s,
            route_timeout_s=self.route_timeout_s,
            neighbor_window_s=self.neighbor_window_s,
            backbone_eval_s=self.backbone_eval_s,
            backbone_initial_delay_s=self.backbone_initial_delay_s,
            data_period_s=self.data_period_s,
        )

        frames = int(self.duration / self.dt)
        states = []
        for _ in range(frames):
            replay.step()
            focus_routes = [
                replay.nodes[focus_node].routes[d]
                for d in sorted(replay.nodes[focus_node].routes.keys())
                if d != focus_node and replay.nodes[focus_node].routes[d].hop_count < INF_HOPS
            ]
            states.append(
                {
                    "t": replay.time,
                    "summary": replay.summary(),
                    "edges": list(replay.active_edges),
                    "roles": [n.role for n in replay.nodes],
                    "routes": focus_routes[:12],
                }
            )

        fig, (ax_net, ax_info) = plt.subplots(1, 2, figsize=(13, 6), gridspec_kw={"width_ratios": [2, 1]})
        xs = [n.x for n in replay.nodes]
        ys = [n.y for n in replay.nodes]

        def draw(i: int) -> None:
            s = states[i]
            ax_net.clear()
            ax_info.clear()

            ax_net.set_title(f"WSN DSDV + Backbone | t={s['t']:.1f}s | N={self.n_nodes}")
            ax_net.set_xlim(0, self.area)
            ax_net.set_ylim(0, self.area)
            ax_net.set_xlabel("X")
            ax_net.set_ylabel("Y")

            # connectivity lines (faint)
            for a in replay.nodes:
                for b in replay.nodes:
                    if a.nid < b.nid and replay._in_range(a, b):
                        ax_net.plot([a.x, b.x], [a.y, b.y], color="#d0d0d0", linewidth=0.5, alpha=0.25)

            # node colors by role
            colors = []
            for r in s["roles"]:
                if r == Role.BACKBONE:
                    colors.append("#2ca02c")
                elif r == Role.LEAF:
                    colors.append("#1f77b4")
                else:
                    colors.append("#7f7f7f")
            ax_net.scatter(xs, ys, c=colors, s=45)
            fx, fy = replay.nodes[focus_node].x, replay.nodes[focus_node].y
            ax_net.scatter([fx], [fy], c="orange", s=150, edgecolors="black", linewidth=1.2)
            for n in replay.nodes:
                ax_net.text(n.x + 0.5, n.y + 0.5, str(n.nid), fontsize=7)

            for u, v in s["edges"]:
                nu, nv = replay.nodes[u], replay.nodes[v]
                ax_net.plot([nu.x, nv.x], [nu.y, nv.y], color="red", linewidth=2.0, alpha=0.75)

            ax_info.axis("off")
            sm = s["summary"]
            lines = [
                f"Focus node: {focus_node}",
                f"Focus role: {'BACKBONE' if replay.nodes[focus_node].role==Role.BACKBONE else 'LEAF' if replay.nodes[focus_node].role==Role.LEAF else 'UNKNOWN'}",
                "",
                f"Data sent: {sm['sent']}",
                f"Delivered: {sm['delivered']}",
                f"PDR: {sm['pdr']*100:.1f}%",
                f"Avg latency: {sm['avg_latency_s']:.3f}s",
                f"Avg hops: {sm['avg_hops']:.2f}",
                "",
                f"HELLO: {sm['hello_packets']} | UPDATE: {sm['update_packets']}",
                f"Backbone nodes: {sm['backbone_nodes']}",
                "",
                "Routing table (sample):",
            ]
            for e in s["routes"]:
                lines.append(f"d={e.dest:3d} via {e.next_hop:3d} hops={e.hop_count:2d} seq={e.seq_num:4d}")
            if len(s["routes"]) == 0:
                lines.append("(no valid routes yet)")
            ax_info.text(0.02, 0.98, "\n".join(lines), va="top", family="monospace", fontsize=9)

        ani = FuncAnimation(fig, draw, frames=frames, interval=1000 / fps, repeat=False)
        if out_file.suffix.lower() == ".mp4":
            try:
                ani.save(str(out_file), fps=fps, dpi=130)
            except Exception:
                ani.save(str(out_file.with_suffix(".gif")), fps=fps, dpi=120, writer="pillow")
        else:
            ani.save(str(out_file), fps=fps, dpi=120, writer="pillow")
        plt.close(fig)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="WSN DSDV + Backbone simulation (auto 10..100)")
    p.add_argument("--nodes", type=int, default=30, help="Single case node count")
    p.add_argument("--auto", action="store_true", help="Auto sweep from --min to --max")
    p.add_argument("--min", dest="min_nodes", type=int, default=10)
    p.add_argument("--max", dest="max_nodes", type=int, default=100)
    p.add_argument("--step", type=int, default=10)
    p.add_argument("--duration", type=float, default=120.0)
    p.add_argument("--dt", type=float, default=1.0)
    p.add_argument("--area", type=float, default=100.0)
    p.add_argument("--range", dest="radio_range", type=float, default=30.0)
    p.add_argument("--hello", type=float, default=8.0)
    p.add_argument("--update", type=float, default=15.0)
    p.add_argument("--timeout", type=float, default=45.0)
    p.add_argument("--neighbor-window", type=float, default=90.0)
    p.add_argument("--backbone-eval", type=float, default=30.0)
    p.add_argument("--backbone-delay", type=float, default=15.0)
    p.add_argument("--data-period", type=float, default=2.5)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--video", action="store_true", default=True, help="Export video (default on)")
    p.add_argument("--video-format", choices=["mp4", "gif"], default="mp4")
    p.add_argument("--fps", type=int, default=8)
    p.add_argument("--focus-node", type=int, default=0)
    p.add_argument("--out-dir", type=str, default="sim_outputs")
    return p.parse_args()


def run_case(n: int, args: argparse.Namespace, out_dir: Path) -> dict:
    sim = WsnDsdvSim(
        n,
        seed=args.seed,
        area=args.area,
        radio_range=args.radio_range,
        dt=args.dt,
        duration=args.duration,
        hello_base_s=args.hello,
        update_base_s=args.update,
        route_timeout_s=args.timeout,
        neighbor_window_s=args.neighbor_window,
        backbone_eval_s=args.backbone_eval,
        backbone_initial_delay_s=args.backbone_delay,
        data_period_s=args.data_period,
    )
    sim.run()
    case_dir = out_dir / f"n{n}"
    case_dir.mkdir(parents=True, exist_ok=True)

    sim.export_metrics_csv(case_dir / "metrics_timeseries.csv")
    sim.export_routing_table_csv(case_dir / "routing_table_node0.csv", node_id=0)

    if args.video:
        ext = ".mp4" if args.video_format == "mp4" else ".gif"
        sim.animate(case_dir / f"simulation{ext}", focus_node=args.focus_node, fps=args.fps)

    summary = sim.summary()
    with (case_dir / "summary.txt").open("w", encoding="utf-8") as f:
        for k, v in summary.items():
            f.write(f"{k}: {v}\n")
    return summary


def main() -> None:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # auto sweep by default if user didn't specify --auto and didn't set --nodes explicitly
    # (in Cursor runs, args.nodes always has a default; so we decide: default behavior is auto sweep)
    do_auto = args.auto or True

    cases = [args.nodes]
    if do_auto:
        cases = list(range(args.min_nodes, args.max_nodes + 1, args.step))
        if cases[0] != args.min_nodes or cases[-1] != args.max_nodes:
            # ensure inclusive max if not aligned with step
            if cases[-1] != args.max_nodes:
                cases.append(args.max_nodes)

    summaries: List[dict] = []
    for n in cases:
        s = run_case(n, args, out_dir)
        summaries.append(s)
        print(f"[DONE] N={n} | PDR={s['pdr']*100:.1f}% | AvgLat={s['avg_latency_s']:.3f}s | Backbone={s['backbone_nodes']}")

    with (out_dir / "batch_summary.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(summaries[0].keys()))
        w.writeheader()
        w.writerows(summaries)

    print(f"\nSaved: {out_dir / 'batch_summary.csv'}")
    print("Per-case: n*/{summary.txt, metrics_timeseries.csv, routing_table_node0.csv, simulation.*}")


if __name__ == "__main__":
    main()

