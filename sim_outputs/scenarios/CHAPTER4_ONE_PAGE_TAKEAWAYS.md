# Chương 4 — Tóm tắt một trang (S1–S12, 3 profile)

_Nguồn: `scenario_runs_wide.csv` / `plot_scenario_pdr_comparison.py` — dùng khi viết kết luận hoặc slide bảo vệ._

## Câu chốt (một câu)

**Backbone–Leaf** (và thường cả **Full + Gradient**) **vượt trội rõ** so với Baseline DSDV khi mạng **≥ 30 nút** hoặc **tải/link xấu**; trên mạng **10 nút, link tốt**, cải tiến **không cần thiết** và profile Full có thể **làm PDR tệ hơn** Baseline do đường relay dài hơn / leaf không relay đa chặng.

## Số nổi bật

| Nhóm kịch bản | Baseline PDR (min–max) | Backbone–Leaf (min–max) | Ghi chú |
|---|---:|---:|---|
| S1–S3 (10 node) | 82–93% | 81–91% | Full: **71–85%** — 3/3 kịch bản **thua** Baseline |
| S4–S12 (30–100 node) | 41–83% | 92–**100%** | Baseline **sụp** (S4,S5,S7,S9 ~41–47%); B–Leaf **≥92%** mọi kịch bản |
| Latency (ms) | 2094–6695 | 2000–2505 | Baseline chậm nhất ở S7,S9,S11 (≥5.3s); cải tiến ~2–2.5s |
| Control (TC) | tương đương nhau | ± vài % | Không phải “thắng lợi” chính; PDR + delay mới tách biệt |
| Route changes (S11) | 17 684 | 11 963 | Giảm ~32% churn trên 100 node |
| TX variance (S8 proxy) | 1293 | 907 (B–Leaf) | Relay **dàn đều hơn**; Full 1028 |

## Ba luận điểm cho H1 / kết luận

1. **Giảm flooding data-plane**: PDR Baseline 40–65% trên 30–50 node (S4–S10) → B–Leaf **94–99%** — phù hợp giả thuyết “giảm relay dư”.
2. **Trade-off hop trên mạng nhỏ**: S1–S3 Full **−8 đến −13 điểm %** PDR vs Baseline — **ghi rõ hạn chế** khi triển khai trên WSN nhỏ, link ổn.
3. **Gradient**: trên mạng lớn thường **gần B–Leaf** (S7,S11 100%); S4–S6 Full **nhẹ hơn** B–Leaf vài điểm % — chấp nhận được đổi lấy coordination; S10 vẫn **89%** vs B–Leaf 92%.

## Câu mẫu (copy vào luận văn)

> Trên tập kịch bản S4–S12 (30–100 nút), DSDV Backbone–Leaf đạt PDR từ 92,05% đến 100%, trong khi Baseline DSDV chỉ đạt 40,51%–83,48% tùy kịch bản; độ trễ trung bình giảm từ 2,7–6,7 s xuống khoảng 2,0–2,5 s. Ngược lại, trên S1–S3 (10 nút, liên kết tốt), profile Full + Gradient làm PDR giảm nhẹ (71–85% so với 82–93% Baseline), phù hợp với cơ chế hạn chế relay trên leaf và đường đi dài hơn.

## File đính kèm nhanh

- Bảng đầy đủ + hình: `chapter4_scenario_metrics.md`
- CSV plot: `pdr_by_scenario.csv`, `scenario_runs_wide.csv`
- Hình: `pdr_by_scenario_*.png`, `latency_ms_by_scenario_*.png`, `forwarding_top_nodes_S8.png`
