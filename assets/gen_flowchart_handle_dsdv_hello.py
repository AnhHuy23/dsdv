#!/usr/bin/env python3
"""Generate flowchart PNG for chat_cli_handle_dsdv_hello (matches src/chat_cli_mesh.c)."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Polygon

OUT = Path(__file__).resolve().parent / "flowchart_handle_dsdv_hello.png"

# (x, y), text, shape: start|end|proc|dec
NODES = [
    ((0.5, 0.97), "START\nchat_cli_handle_dsdv_hello", "start"),
    ((0.5, 0.90), "buf->len < sizeof(hello)?", "dec"),
    ((0.18, 0.83), "Return -EINVAL", "proc"),
    ((0.5, 0.83), "Decode HELLO (memcpy)\nneighbor = ctx->addr\ndest = hello.src\nmy_addr = rt->addr", "proc"),
    ((0.5, 0.74), "neighbor != dest?", "dec"),
    ((0.18, 0.67), "Return 0", "proc"),
    ((0.5, 0.67), "dest == my_addr?", "dec"),
    ((0.18, 0.60), "Return 0", "proc"),
    ((0.5, 0.60), "existing = dsdv_find_route(dest)\nthreshold = -80 if existing else -75", "proc"),
    ((0.5, 0.52), "rssi != 0 AND rssi < threshold?", "dec"),
    ((0.18, 0.45), "Return 0", "proc"),
    ((0.5, 0.45), "update_neighbor_rssi\n(if rssi != 0)", "proc"),
    (
        (0.5, 0.36),
        "Update neighbor_backbone_info\n(degree, role, leaf_type,\ngradient, PDR, last_seen)",
        "proc",
    ),
    ((0.5, 0.27), "g_my_role == LEAF?", "dec"),
    ((0.82, 0.20), "chat_cli_refresh_leaf\n_routing_mode()", "proc"),
    ((0.5, 0.20), "dsdv_upsert(dest, neighbor,\nhop=1, hello.seq_num)", "proc"),
    ((0.5, 0.12), "route changed?", "dec"),
    ((0.82, 0.05), "reschedule\ndsdv_update_work\n(1.5–3 s)", "proc"),
    ((0.5, 0.05), "Return 0", "proc"),
    ((0.5, -0.02), "END", "end"),
]

EDGES = [
    (0, 1),
    (1, 2, "YES"),
    (1, 3, "NO"),
    (2, 19),
    (3, 4),
    (4, 5, "YES"),
    (4, 6, "NO"),
    (5, 19),
    (6, 7, "YES"),
    (6, 8, "NO"),
    (7, 19),
    (8, 9),
    (9, 10, "YES"),
    (9, 11, "NO"),
    (10, 19),
    (11, 12),
    (12, 13),
    (13, 14, "YES"),
    (13, 15, "NO"),
    (14, 15),
    (15, 16),
    (16, 17, "YES"),
    (16, 18, "NO"),
    (17, 18),
    (18, 19),
]


def draw_node(ax, xy, text, kind):
    x, y = xy
    fs = 7.5 if kind == "proc" else 8
    if kind == "start":
        c = Circle((x, y), 0.035, facecolor="#E8F5E9", edgecolor="#2E7D32", linewidth=1.5)
        ax.add_patch(c)
    elif kind == "end":
        c = Circle((x, y), 0.03, facecolor="#FFEBEE", edgecolor="#C62828", linewidth=1.5)
        ax.add_patch(c)
    elif kind == "dec":
        w, h = 0.22, 0.055
        pts = [(x, y + h), (x + w / 2, y), (x, y - h), (x - w / 2, y)]
        p = Polygon(pts, closed=True, facecolor="#FFF8E1", edgecolor="#F57F17", linewidth=1.5)
        ax.add_patch(p)
    else:
        w, h = 0.26, 0.065
        if len(text) > 40:
            h = 0.08
        box = FancyBboxPatch(
            (x - w / 2, y - h / 2),
            w,
            h,
            boxstyle="round,pad=0.01",
            facecolor="#E3F2FD",
            edgecolor="#1565C0",
            linewidth=1.5,
        )
        ax.add_patch(box)
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, family="sans-serif")


def arrow(ax, a, b, label=None):
    x0, y0 = NODES[a][0]
    x1, y1 = NODES[b][0]
    dy0 = 0.04 if NODES[a][2] in ("start", "proc") else 0.055
    dy1 = 0.04 if NODES[b][2] in ("start", "proc", "end") else 0.055
    if y0 > y1:
        y0 -= dy0
        y1 += dy1
    else:
        y0 += dy0
        y1 -= dy1
    ax.annotate(
        "",
        xy=(x1, y1),
        xytext=(x0, y0),
        arrowprops=dict(arrowstyle="->", color="#37474F", lw=1.2),
    )
    if label:
        ax.text((x0 + x1) / 2 + 0.04, (y0 + y1) / 2, label, fontsize=7, color="#BF360C")


def main():
    fig, ax = plt.subplots(figsize=(11, 15), dpi=200)
    ax.set_xlim(0, 1)
    ax.set_ylim(-0.08, 1.02)
    ax.axis("off")
    ax.set_title(
        "Flowchart: chat_cli_handle_dsdv_hello()\n(src/chat_cli_mesh.c)",
        fontsize=12,
        fontweight="bold",
        pad=12,
    )

    for i, (xy, text, kind) in enumerate(NODES):
        draw_node(ax, xy, text, kind)

    for e in EDGES:
        if len(e) == 3:
            arrow(ax, e[0], e[1], e[2])
        else:
            arrow(ax, e[0], e[1])

    # side merge lines to Return 0 / END
    fig.tight_layout()
    fig.savefig(OUT, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"Saved: {OUT}")


if __name__ == "__main__":
    main()
