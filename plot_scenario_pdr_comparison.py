#!/usr/bin/env python3
"""
Aggregate metrics across preset scenarios S1..S12 (after ``--all-scenarios --compare-three``).

Generates figures + CSV + Markdown aligned with thesis sections:
  - 4.5 PDR, 4.6 latency, 4.7 control overhead (HELLO+UPDATE + dropped control as note),
  - 4.8.1 route changes,
  - 4.8.2 forwarding load (top nodes + TX variance) for one chosen scenario.

Usage:
    python dsdv_env_sim.py --all-scenarios --compare-three --duration 300 --seed 7
    python plot_scenario_pdr_comparison.py --scenarios-dir sim_outputs/scenarios

Outputs (under scenarios-dir by default):
    scenario_runs_wide.csv
    chapter4_scenario_metrics.md
    pdr_by_scenario_*.png, latency_ms_by_scenario_*.png, control_total_by_scenario_*.png,
    route_changes_by_scenario_*.png
    forwarding_top_nodes_<SCENARIO>.png   (if per_node_stats.csv exists for all three profiles)
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from dsdv_env_sim import SCENARIO_ORDER, SCENARIO_PRESETS, SIM_PROFILE_LABELS, SIM_PROFILE_SEQUENCE


def _try_utf8_stdout() -> None:
    try:
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    except Exception:
        pass


def parse_summary_txt(path: Path) -> Dict[str, Any]:
    """Parse key:value lines from ``summary.txt`` produced by ``DsdvEnvSim.summary_text``."""
    out: Dict[str, Any] = {}
    text = path.read_text(encoding="utf-8", errors="replace")
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, _, rest = line.partition(":")
        key = key.strip()
        val = rest.strip()
        if key in ("pdr_pct", "avg_latency_s", "avg_hops"):
            try:
                out[key] = float(val)
            except ValueError:
                out[key] = float("nan")
        elif key in (
            "hello_packets",
            "update_packets",
            "total_control_packets",
            "control_dropped",
            "data_packets",
            "data_delivered",
            "data_dropped",
            "route_changes",
            "acks_delivered",
            "backbone_nodes",
            "leaf_nodes",
            "nodes",
        ):
            try:
                out[key] = int(float(val))
            except ValueError:
                out[key] = 0
        elif key in ("sim_profile",):
            out[key] = val

    h = int(out.get("hello_packets", 0))
    u = int(out.get("update_packets", 0))
    if "total_control_packets" not in out or out.get("total_control_packets") == 0:
        out["total_control_packets"] = h + u
    return out


def _summary_path_compare_three(scenarios_dir: Path, scenario_id: str, profile_subdir: str) -> Path:
    return scenarios_dir / scenario_id / profile_subdir / "summary.txt"


def _per_node_path(scenarios_dir: Path, scenario_id: str, profile_subdir: str) -> Path:
    return scenarios_dir / scenario_id / profile_subdir / "per_node_stats.csv"


def _get_total_control(s: Dict[str, Any]) -> int:
    v = int(s.get("total_control_packets", 0))
    if v > 0:
        return v
    return int(s.get("hello_packets", 0)) + int(s.get("update_packets", 0))


def collect_rows(scenarios_dir: Path) -> Tuple[List[Dict[str, Any]], bool]:
    rows: List[Dict[str, Any]] = []
    used_three = False
    if SCENARIO_ORDER:
        probe = _summary_path_compare_three(scenarios_dir, SCENARIO_ORDER[0], SIM_PROFILE_SEQUENCE[0][1])
        used_three = probe.is_file()

    metrics = [
        ("pdr", "pdr_pct"),
        ("latency_s", "avg_latency_s"),
        ("hops", "avg_hops"),
        ("total_control", None),  # special
        ("control_dropped", "control_dropped"),
        ("hello", "hello_packets"),
        ("update", "update_packets"),
        ("route_changes", "route_changes"),
    ]

    for sid in SCENARIO_ORDER:
        preset = SCENARIO_PRESETS.get(sid, {})
        desc = str(preset.get("description_vi", ""))
        row: Dict[str, Any] = {"scenario_id": sid, "description_vi": desc}
        for prof in ("baseline", "backbone_leaf", "full"):
            for short, skey in metrics:
                if short == "total_control":
                    row[f"{prof}_total_control"] = 0
                else:
                    row[f"{prof}_{short}"] = float("nan") if short in ("pdr", "latency_s", "hops") else 0

        if used_three:
            for profile_key, subdir in SIM_PROFILE_SEQUENCE:
                p = _summary_path_compare_three(scenarios_dir, sid, subdir)
                if not p.is_file():
                    continue
                s = parse_summary_txt(p)
                prefix = profile_key
                row[f"{prefix}_pdr"] = float(s.get("pdr_pct", float("nan")))
                row[f"{prefix}_latency_s"] = float(s.get("avg_latency_s", float("nan")))
                row[f"{prefix}_hops"] = float(s.get("avg_hops", float("nan")))
                row[f"{prefix}_total_control"] = _get_total_control(s)
                row[f"{prefix}_control_dropped"] = int(s.get("control_dropped", 0))
                row[f"{prefix}_hello"] = int(s.get("hello_packets", 0))
                row[f"{prefix}_update"] = int(s.get("update_packets", 0))
                row[f"{prefix}_route_changes"] = int(s.get("route_changes", 0))
                if profile_key == "full":
                    row["full_backbone_nodes"] = int(s.get("backbone_nodes", 0))
                    row["full_leaf_nodes"] = int(s.get("leaf_nodes", 0))
        else:
            p = scenarios_dir / sid / "summary.txt"
            if p.is_file():
                s = parse_summary_txt(p)
                prof = str(s.get("sim_profile", "full"))
                prefix = prof if prof in ("baseline", "backbone_leaf", "full") else "full"
                row[f"{prefix}_pdr"] = float(s.get("pdr_pct", float("nan")))
                row[f"{prefix}_latency_s"] = float(s.get("avg_latency_s", float("nan")))
                row[f"{prefix}_hops"] = float(s.get("avg_hops", float("nan")))
                row[f"{prefix}_total_control"] = _get_total_control(s)
                row[f"{prefix}_control_dropped"] = int(s.get("control_dropped", 0))
                row[f"{prefix}_hello"] = int(s.get("hello_packets", 0))
                row[f"{prefix}_update"] = int(s.get("update_packets", 0))
                row[f"{prefix}_route_changes"] = int(s.get("route_changes", 0))

        for prof in ("baseline", "backbone_leaf", "full"):
            ls = row.get(f"{prof}_latency_s")
            row[f"{prof}_latency_ms"] = (float(ls) * 1000.0) if ls == ls else float("nan")

        rows.append(row)
    return rows, used_three


def _finite_max(vals: Sequence[float]) -> float:
    ok = [float(v) for v in vals if v == v]
    return max(ok) if ok else 0.0


def _lazy_plt():
    try:
        import matplotlib.pyplot as plt  # type: ignore
        import numpy as np  # type: ignore
    except Exception as exc:
        return None, None, exc
    return plt, np, None


def _plot_grouped_triple_pillow(
    rows: List[Dict[str, Any]],
    field_b: str,
    field_bl: str,
    field_f: str,
    ylabel: str,
    title: str,
    out_path: Path,
    ymax: Optional[float] = None,
) -> Optional[Path]:
    try:
        from PIL import Image, ImageDraw, ImageFont  # type: ignore
    except Exception as exc:
        print(f"[WARN] pillow: {exc}; skip grouped plot {out_path.name}")
        return None

    labels = [r["scenario_id"] for r in rows]
    b = [float(r.get(field_b, 0.0)) for r in rows]
    bl = [float(r.get(field_bl, 0.0)) for r in rows]
    f = [float(r.get(field_f, 0.0)) for r in rows]

    width, height = 2048, 900
    margin_l, margin_r, margin_t, margin_b = 100, 55, 85, 90
    plot_w = width - margin_l - margin_r
    plot_h = height - margin_t - margin_b
    top = ymax if ymax is not None else max(_finite_max(b + bl + f) * 1.08, 1.0)
    step = max(1.0, math.ceil(top / 8.0 / 100.0) * 100.0)
    ytop = math.ceil(top / step) * step

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 22)
        title_font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 28)
        legend_font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 20)
    except Exception:
        font = ImageFont.load_default()
        title_font = font
        legend_font = font

    colors = ("#1f77b4", "#ff7f0e", "#2ca02c")
    legends = (
        SIM_PROFILE_LABELS["baseline"],
        SIM_PROFILE_LABELS["backbone_leaf"],
        SIM_PROFILE_LABELS["full"],
    )

    def sy(value: float) -> float:
        return margin_t + plot_h - (value / ytop) * plot_h

    title_w = draw.textlength(title, font=title_font)
    draw.text(((width - title_w) / 2, 18), title, fill="#111111", font=title_font)
    draw.line((margin_l, margin_t, margin_l, margin_t + plot_h), fill="#222222", width=2)
    draw.line((margin_l, margin_t + plot_h, margin_l + plot_w, margin_t + plot_h), fill="#222222", width=2)
    draw.text((margin_l + plot_w / 2 - 35, height - 38), "Kịch bản", fill="#111111", font=font)
    draw.text((18, margin_t + plot_h / 2 - 20), ylabel, fill="#111111", font=font)

    tick = 0.0
    while tick <= ytop + 1e-9:
        y = sy(tick)
        draw.line((margin_l, y, margin_l + plot_w, y), fill="#e0e0e0", width=1)
        draw.text((34, y - 12), f"{int(tick)}", fill="#222222", font=font)
        tick += step

    group_w = plot_w / max(1, len(labels))
    bar_w = min(36.0, group_w * 0.22)
    offsets = (-bar_w, 0.0, bar_w)
    series = (b, bl, f)
    for idx, sid in enumerate(labels):
        center = margin_l + group_w * idx + group_w * 0.5
        for vals, color, offset in zip(series, colors, offsets):
            value = float(vals[idx])
            x = center + offset
            y = sy(value)
            draw.rectangle((x - bar_w / 2, y, x + bar_w / 2, margin_t + plot_h), fill=color)
        draw.text((center - 11, margin_t + plot_h + 18), sid, fill="#111111", font=font)

    legend_x, legend_y = margin_l + 10, margin_t + 12
    legend_w, legend_h = 435, 105
    draw.rectangle((legend_x - 4, legend_y - 4, legend_x + legend_w, legend_y + legend_h), outline="#d0d0d0", fill="#ffffff")
    for i, (label, color) in enumerate(zip(legends, colors)):
        y = legend_y + i * 32
        draw.rectangle((legend_x + 10, y + 5, legend_x + 42, y + 23), fill=color)
        draw.text((legend_x + 55, y + 2), label, fill="#111111", font=legend_font)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(out_path)
    return out_path


def plot_grouped_triple(
    rows: List[Dict[str, Any]],
    field_b: str,
    field_bl: str,
    field_f: str,
    ylabel: str,
    title: str,
    out_path: Path,
    ymax: Optional[float] = None,
) -> Optional[Path]:
    plt, np, err = _lazy_plt()
    if err is not None:
        print(f"[WARN] matplotlib/numpy: {err}; using Pillow fallback for {out_path.name}")
        return _plot_grouped_triple_pillow(rows, field_b, field_bl, field_f, ylabel, title, out_path, ymax=ymax)

    labels = [r["scenario_id"] for r in rows]
    b = [float(r.get(field_b, float("nan"))) for r in rows]
    bl = [float(r.get(field_bl, float("nan"))) for r in rows]
    f = [float(r.get(field_f, float("nan"))) for r in rows]

    x = np.arange(len(labels))
    width = 0.24

    fig, ax = plt.subplots(figsize=(16, 7.2), dpi=150)
    ax.bar(x - width, b, width, label=SIM_PROFILE_LABELS["baseline"], color="#1f77b4", edgecolor="#0d3d66", linewidth=0.35)
    ax.bar(x, bl, width, label=SIM_PROFILE_LABELS["backbone_leaf"], color="#ff7f0e", edgecolor="#a35200", linewidth=0.35)
    ax.bar(x + width, f, width, label=SIM_PROFILE_LABELS["full"], color="#2ca02c", edgecolor="#1d6b1d", linewidth=0.35)

    ax.set_ylabel(ylabel)
    ax.set_xlabel("Kịch bản")
    ax.set_title(title)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=0)
    top = ymax if ymax is not None else max(_finite_max(b + bl + f) * 1.08, 1.0)
    ax.set_ylim(0.0, top)
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def plot_lines_triple(
    rows: List[Dict[str, Any]],
    field_b: str,
    field_bl: str,
    field_f: str,
    ylabel: str,
    title: str,
    out_path: Path,
    ymax: Optional[float] = None,
) -> Optional[Path]:
    plt, np, err = _lazy_plt()
    if err is not None:
        print(f"[WARN] matplotlib/numpy: {err}; skip line plot {out_path.name}")
        return None

    labels = [r["scenario_id"] for r in rows]
    x = np.arange(len(labels))
    b = np.array([float(r.get(field_b, float("nan"))) for r in rows], dtype=float)
    bl = np.array([float(r.get(field_bl, float("nan"))) for r in rows], dtype=float)
    f = np.array([float(r.get(field_f, float("nan"))) for r in rows], dtype=float)

    fig, ax = plt.subplots(figsize=(16, 7.0), dpi=150)
    ax.plot(x, b, "o-", color="#1f77b4", linewidth=2.0, markersize=7, label=SIM_PROFILE_LABELS["baseline"])
    ax.plot(x, bl, "s-", color="#ff7f0e", linewidth=2.0, markersize=7, label=SIM_PROFILE_LABELS["backbone_leaf"])
    ax.plot(x, f, "^-", color="#2ca02c", linewidth=2.0, markersize=7, label=SIM_PROFILE_LABELS["full"])

    ax.set_ylabel(ylabel)
    ax.set_xlabel("Kịch bản")
    ax.set_title(title)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    top = ymax if ymax is not None else max(_finite_max(list(b) + list(bl) + list(f)) * 1.08, 1.0)
    ax.set_ylim(0.0, top)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def write_wide_csv(rows: List[Dict[str, Any]], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys: List[str] = []
    for r in rows:
        keys.extend(k for k in r if k not in keys)
    keys.sort(key=lambda k: (0 if k in ("scenario_id", "description_vi") else 1, k))
    if "scenario_id" in keys:
        keys.remove("scenario_id")
        keys.insert(0, "scenario_id")
    if "description_vi" in keys:
        keys.remove("description_vi")
        keys.insert(1, "description_vi")

    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})
    return path


def _fmt(x: Any) -> str:
    if isinstance(x, float) and x != x:
        return "—"
    if isinstance(x, float):
        return f"{x:.2f}"
    return str(x)


def load_per_node_tx(path: Path) -> Dict[int, int]:
    if not path.is_file():
        return {}
    out: Dict[int, int] = {}
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            out[int(row["node_id"])] = int(row["tx_count"])
    return out


def tx_variance(txs: Sequence[int]) -> float:
    if len(txs) < 2:
        return 0.0
    return float(statistics.pvariance(txs))


def plot_forwarding_top_nodes(
    scenarios_dir: Path,
    scenario_id: str,
    out_path: Path,
    top_k: int = 10,
) -> Optional[Path]:
    plt, np, err = _lazy_plt()
    if err is not None:
        return None

    series: List[Tuple[str, str, Dict[int, int]]] = []
    for profile_key, subdir in SIM_PROFILE_SEQUENCE:
        p = _per_node_path(scenarios_dir, scenario_id, subdir)
        m = load_per_node_tx(p)
        if not m:
            print(f"[WARN] missing per_node_stats for {scenario_id}/{subdir}; re-run sim to export.")
            return None
        series.append((profile_key, subdir, m))

    all_ids = set()
    for _, _, m in series:
        all_ids.update(m.keys())
    scores = {nid: max(m.get(nid, 0) for _, _, m in series) for nid in all_ids}
    top_ids = sorted(all_ids, key=lambda i: (-scores[i], i))[:top_k]

    x = np.arange(len(top_ids))
    width = 0.25
    fig, ax = plt.subplots(figsize=(14, 6.5), dpi=150)
    for i, (pk, _, m) in enumerate(series):
        vals = [m.get(nid, 0) for nid in top_ids]
        offset = (i - 1) * width
        ax.bar(x + offset, vals, width, label=SIM_PROFILE_LABELS.get(pk, pk))

    ax.set_xticks(x)
    ax.set_xticklabels([str(n) for n in top_ids])
    ax.set_xlabel("node_id (top theo max TX giữa 3 profile)")
    ax.set_ylabel("tx_count (HELLO+UPDATE+DATA+ACK, proxy tải relay)")
    ax.set_title(f"Forwarding / TX load — {scenario_id} (top {top_k} nodes)")
    ax.legend(loc="best", fontsize=9)
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def build_pdr_section(rows: List[Dict[str, Any]], used_three: bool) -> str:
    lines: List[str] = []
    lines.append("## 4.5 Kết quả PDR\n")
    lines.append("### Mục tiêu\n")
    lines.append(
        "- Kiểm tra **cải tiến** có làm **mất gói nhiều hơn** Baseline hay không; nếu PDR **giảm nhẹ** thì **ghi rõ** (hop, đường đi, leaf hạn chế relay).\n"
    )
    if not used_three:
        lines.append("\n> Chưa có ``S*/baseline_dsdv/summary.txt``. Chạy ``--all-scenarios --compare-three``.\n")
        return "\n".join(lines)

    eps = 0.75
    regressions: List[str] = []
    for r in rows:
        sid = r["scenario_id"]
        b, bl, g = r["baseline_pdr"], r["backbone_leaf_pdr"], r["full_pdr"]
        hb, hf = r["baseline_hops"], r["full_hops"]
        if b == b and g == g and (g + 1e-9) < (b - eps):
            hints: List[str] = []
            if hf == hf and hb == hb and hf > hb + 0.05:
                hints.append(f"avg hop cao hơn Baseline ({_fmt(hf)} vs {_fmt(hb)})")
            hints.append("đường chuyển tiếp dài hơn / backbone-forward-only")
            hints.append("leaf hạn chế relay đa chặng")
            regressions.append(
                f"- **{sid}**: Full {_fmt(g)}% < Baseline {_fmt(b)}% — " + "; ".join(hints) + "."
            )
        elif b == b and bl == bl and (bl + 1e-9) < (b - eps):
            regressions.append(f"- **{sid}**: Backbone–Leaf {_fmt(bl)}% < Baseline {_fmt(b)}% — phân vùng đổi đồ thị forward, tạm thời churn.\n")

    if regressions:
        lines.append("### Ghi nhận tự động (PDR giảm nhẹ)\n")
        lines.extend(regressions)
    else:
        lines.append(f"\nKhông có kịch bản nào Full/BL thấp hơn Baseline > **{eps}** điểm % (theo dữ liệu hiện tại).\n")

    lines.append("\n| Kịch bản | Baseline | B–Leaf | B–Leaf+Grad |\n|---|---:|---:|---:|")
    for r in rows:
        lines.append(
            f"| {r['scenario_id']} | {_fmt(r['baseline_pdr'])} | {_fmt(r['backbone_leaf_pdr'])} | {_fmt(r['full_pdr'])} |"
        )
    return "\n".join(lines)


def build_latency_section(rows: List[Dict[str, Any]], used_three: bool) -> str:
    lines: List[str] = []
    lines.append("\n## 4.6 Kết quả độ trễ (Average delay)\n")
    lines.append("### Mục tiêu\n")
    lines.append(
        "- Xem cải tiến có làm **route / delivery chậm hơn** không (độ trễ ACK end-to-end trong mô phỏng).\n"
    )
    lines.append(
        "### Khung phân tích (định tính)\n"
        "- **Baseline**: độ trễ thường **tăng mạnh** khi flooding / relay đồng loạt → collision & queueing.\n"
        "- **Backbone–Leaf**: giảm **relay dư**, giảm va chạm → độ trễ có thể **giảm**.\n"
        "- **Gradient**: **đôi khi độ trễ tăng nhẹ** — chọn đường tránh quá tải, **thêm hop** leaf–leaf; đây là trade-off “research” hợp lý.\n"
    )
    if not used_three:
        return "\n".join(lines)

    lines.append("\n### Ghi nhận tự động (Full chậm hơn Baseline ≥ 5 ms)\n")
    thr_ms = 5.0
    notes: List[str] = []
    for r in rows:
        sid = r["scenario_id"]
        b, g = r["baseline_latency_ms"], r["full_latency_ms"]
        hb, hf = r["baseline_hops"], r["full_hops"]
        if b == b and g == g and (g > b + thr_ms):
            extra = ""
            if hf == hf and hb == hb and hf > hb + 0.05:
                extra = f" (avg hop Full {_fmt(hf)} vs Baseline {_fmt(hb)})"
            notes.append(f"- **{sid}**: Full **{_fmt(g)} ms** > Baseline **{_fmt(b)} ms**{extra} — có thể do thêm hop / đường gradient dài hơn.")
    lines.extend(notes if notes else ["- Không phát hiện chênh lệch độ trễ Full > Baseline quá ngưỡng trên (có thể chỉnh ngưỡng trong script).\n"])

    lines.append("\n| Kịch bản | Baseline (ms) | B–Leaf (ms) | Full (ms) |\n|---|---:|---:|---:|")
    for r in rows:
        lines.append(
            f"| {r['scenario_id']} | {_fmt(r['baseline_latency_ms'])} | {_fmt(r['backbone_leaf_latency_ms'])} | {_fmt(r['full_latency_ms'])} |"
        )
    return "\n".join(lines)


def build_control_section(rows: List[Dict[str, Any]], used_three: bool) -> str:
    lines: List[str] = []
    lines.append("\n## 4.7 Kết quả control overhead (quan trọng)\n")
    lines.append("### Định nghĩa trong mô phỏng\n")
    lines.append(
        "- **Total control** = **HELLO + UPDATE** (lũy kế, toàn mạng). Đây là chỉ số chính cho **flooding / DSDV control-plane**.\n"
        "- Trường **control_dropped** (nếu có trong ``summary.txt``): gói điều khiển không xử lý được (ví dụ năng lượng, filter backbone); dùng **bổ trợ** khi phân tích nghẽn relay điều khiển.\n"
    )
    lines.append(
        "### Khung H1 (giảm overhead)\n"
        "- **Baseline**: overhead **tăng mạnh** khi số nút / tải tăng (mọi nút tham gia relay điều khiển đầy đủ).\n"
        "- **Backbone–Leaf**: **giảm rõ** — backbone gánh phần lớn relay đa chặng điều khiển; leaf **không participate full** như Baseline.\n"
        "- **Full + Gradient**: có thể **tăng nhẹ** so với chỉ Backbone–Leaf do **coordination** (gradient, vai trò), nhưng thường **vẫn thấp hơn Baseline nhiều**.\n"
    )
    if not used_three:
        return "\n".join(lines)

    lines.append("\n| Kịch bản | Baseline TC | B–Leaf TC | Full TC | BL vs Base | Full vs Base |\n")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for r in rows:
        b, bl, f = r["baseline_total_control"], r["backbone_leaf_total_control"], r["full_total_control"]
        dbl = (bl - b) if all(isinstance(x, int) for x in (b, bl)) else 0
        df = (f - b) if all(isinstance(x, int) for x in (b, f)) else 0
        lines.append(
            f"| {r['scenario_id']} | {b} | {bl} | {f} | {dbl:+d} | {df:+d} |"
        )
    return "\n".join(lines)


def build_route_section(rows: List[Dict[str, Any]], used_three: bool) -> str:
    lines: List[str] = []
    lines.append("\n## 4.8.1 Route stability — Route changes\n")
    lines.append(
        "- **Baseline**: **route churn** cao hơn khi relay đông / link biến động → nhiều invalidation.\n"
        "- **Backbone–Leaf**: bảng tuyến **ổn định hơn** trên leaf (single-parent), ích đổi trên backbone.\n"
        "- **Gradient**: tuning tốt → churn **giảm**; tuning aggressive (đổi cha / gradient) → có thể **tăng nhẹ** — cần đối chiếu số liệu.\n"
    )
    if not used_three:
        return "\n".join(lines)
    lines.append("\n| Kịch bản | Baseline | B–Leaf | Full |\n|---|---:|---:|---:|")
    for r in rows:
        lines.append(
            f"| {r['scenario_id']} | {r['baseline_route_changes']} | {r['backbone_leaf_route_changes']} | {r['full_route_changes']} |"
        )
    return "\n".join(lines)


def build_forwarding_section(
    scenarios_dir: Path,
    scenario_id: str,
    used_three: bool,
) -> str:
    lines: List[str] = []
    lines.append("\n## 4.8.2 Forwarding load — phân bố tải relay (proxy: tx_count)\n")
    lines.append(
        "- **tx_count** trên mỗi nút = tổng lần **phát** (HELLO + UPDATE + DATA + ACK); dùng làm **proxy** cho tải relay / airtime.\n"
        "- **Phương tuyến nóng**: một vài nút có TX **cao hơn hẳn** đám còn lại → **variance** TX lớn.\n"
        "- **Cân bằng tốt**: variance **thấp** hơn khi tải được dàn đều (Backbone/Gradient).\n"
    )
    if not used_three:
        return "\n".join(lines)

    tbl: List[str] = []
    for profile_key, subdir in SIM_PROFILE_SEQUENCE:
        p = _per_node_path(scenarios_dir, scenario_id, subdir)
        m = load_per_node_tx(p)
        if not m:
            tbl.append(f"- **{profile_key}**: (không có ``per_node_stats.csv`` — chạy lại mô phỏng sau khi cập nhật ``dsdv_env_sim``).\n")
            continue
        v = tx_variance(list(m.values()))
        mx = max(m.values()) if m else 0
        tbl.append(f"- **{SIM_PROFILE_LABELS.get(profile_key, profile_key)}**: variance(tx_count)={v:.1f}, max_tx={mx}\n")

    lines.extend(tbl)
    lines.append(
        f"\nBiểu đồ cột (top nodes): ``forwarding_top_nodes_{scenario_id}.png`` (nếu đã tạo).\n"
    )
    return "\n".join(lines)


def build_chapter4_md(
    rows: List[Dict[str, Any]],
    used_three: bool,
    scenarios_dir: Path,
    fwd_scenario: str,
) -> str:
    parts = [
        "# Chương 4 — Hình và ghi chú theo kịch bản S1–S12\n",
        "_File tự động từ ``plot_scenario_pdr_comparison.py``; bổ sung diễn giải tay khi chèn vào luận văn._\n",
        build_pdr_section(rows, used_three),
        build_latency_section(rows, used_three),
        build_control_section(rows, used_three),
        build_route_section(rows, used_three),
        build_forwarding_section(scenarios_dir, fwd_scenario, used_three),
    ]
    return "\n".join(parts)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Scenario S1–S12 figures: PDR, latency, control, routes, forwarding.")
    p.add_argument("--scenarios-dir", type=Path, default=Path("sim_outputs/scenarios"))
    p.add_argument(
        "--forwarding-scenario",
        type=str,
        default="S8",
        metavar="S8",
        help="Scenario id for per-node TX comparison (needs per_node_stats.csv in all three profile dirs). Default S8.",
    )
    p.add_argument("--forwarding-top-k", type=int, default=10)
    p.add_argument("--style", choices=("grouped", "lines", "both"), default="both")
    return p.parse_args()


def main() -> int:
    _try_utf8_stdout()
    args = parse_args()
    root = Path(args.scenarios_dir)
    if not root.is_dir():
        print(f"[ERR] scenarios dir not found: {root}")
        return 1

    rows, used_three = collect_rows(root)
    if not rows:
        print("[ERR] no scenario rows")
        return 1

    write_wide_csv(rows, root / "scenario_runs_wide.csv")

    # PDR (giữ tên file cũ)
    if args.style in ("grouped", "both"):
        plot_grouped_triple(
            rows,
            "baseline_pdr",
            "backbone_leaf_pdr",
            "full_pdr",
            "PDR (%)",
            "PDR theo kịch bản (S1–S12)",
            root / "pdr_by_scenario_grouped.png",
            ymax=105.0,
        )
    if args.style in ("lines", "both"):
        plot_lines_triple(
            rows,
            "baseline_pdr",
            "backbone_leaf_pdr",
            "full_pdr",
            "PDR (%)",
            "PDR theo kịch bản — đường xu hướng",
            root / "pdr_by_scenario_lines.png",
            ymax=105.0,
        )

    # 4.6 Latency (ms)
    lat_max = max(1.0, _finite_max([r["baseline_latency_ms"] for r in rows] + [r["backbone_leaf_latency_ms"] for r in rows] + [r["full_latency_ms"] for r in rows]) * 1.15)
    if args.style in ("grouped", "both"):
        plot_grouped_triple(
            rows,
            "baseline_latency_ms",
            "backbone_leaf_latency_ms",
            "full_latency_ms",
            "Average delay (ms)",
            "Độ trễ trung bình theo kịch bản (ACK end-to-end, ms)",
            root / "latency_ms_by_scenario_grouped.png",
            ymax=lat_max,
        )
    if args.style in ("lines", "both"):
        plot_lines_triple(
            rows,
            "baseline_latency_ms",
            "backbone_leaf_latency_ms",
            "full_latency_ms",
            "Average delay (ms)",
            "Độ trễ trung bình — đường xu hướng (ms)",
            root / "latency_ms_by_scenario_lines.png",
            ymax=lat_max,
        )

    # 4.7 Control total
    cmax = max(1.0, _finite_max([float(r["baseline_total_control"]) for r in rows] + [float(r["backbone_leaf_total_control"]) for r in rows] + [float(r["full_total_control"]) for r in rows]) * 1.08)
    if args.style in ("grouped", "both"):
        plot_grouped_triple(
            rows,
            "baseline_total_control",
            "backbone_leaf_total_control",
            "full_total_control",
            "Total control packets (HELLO + UPDATE)",
            "Overhead điều khiển lũy kế theo kịch bản",
            root / "control_total_by_scenario_grouped.png",
            ymax=cmax,
        )
    if args.style in ("lines", "both"):
        plot_lines_triple(
            rows,
            "baseline_total_control",
            "backbone_leaf_total_control",
            "full_total_control",
            "Total control packets (HELLO + UPDATE)",
            "Overhead điều khiển — đường xu hướng",
            root / "control_total_by_scenario_lines.png",
            ymax=cmax,
        )

    # 4.8.1 Route changes
    rmax = max(1.0, _finite_max([float(r["baseline_route_changes"]) for r in rows] + [float(r["backbone_leaf_route_changes"]) for r in rows] + [float(r["full_route_changes"]) for r in rows]) * 1.08)
    if args.style in ("grouped", "both"):
        plot_grouped_triple(
            rows,
            "baseline_route_changes",
            "backbone_leaf_route_changes",
            "full_route_changes",
            "Route changes (count)",
            "Độ biến động bảng định tuyến theo kịch bản",
            root / "route_changes_by_scenario_grouped.png",
            ymax=rmax,
        )
    if args.style in ("lines", "both"):
        plot_lines_triple(
            rows,
            "baseline_route_changes",
            "backbone_leaf_route_changes",
            "full_route_changes",
            "Route changes (count)",
            "Route changes — đường xu hướng",
            root / "route_changes_by_scenario_lines.png",
            ymax=rmax,
        )

    fwd_sid = str(args.forwarding_scenario).strip().upper()
    if fwd_sid and used_three:
        fp = root / f"forwarding_top_nodes_{fwd_sid}.png"
        if plot_forwarding_top_nodes(root, fwd_sid, fp, top_k=int(args.forwarding_top_k)):
            print(f"Wrote {fp}")

    md_path = root / "chapter4_scenario_metrics.md"
    md_path.write_text(build_chapter4_md(rows, used_three, root, fwd_sid), encoding="utf-8")

    # Giữ file ghi chú PDR riêng (tương thích tên cũ)
    (root / "pdr_analysis_notes.md").write_text(build_pdr_section(rows, used_three), encoding="utf-8")

    # Giữ CSV hẹp cho PDR (Excel nhanh)
    narrow = root / "pdr_by_scenario.csv"
    with narrow.open("w", newline="", encoding="utf-8") as f:
        fn = ["scenario_id", "description_vi", "baseline_pdr", "backbone_leaf_pdr", "full_pdr"]
        w = csv.DictWriter(f, fieldnames=fn, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in fn})

    print(f"Wrote {root / 'scenario_runs_wide.csv'}")
    print(f"Wrote {md_path}")
    print(f"Wrote {narrow}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
