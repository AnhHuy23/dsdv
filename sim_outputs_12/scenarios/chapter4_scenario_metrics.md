# Chương 4 — Hình và ghi chú theo kịch bản S1–S12

_File tự động từ ``plot_scenario_pdr_comparison.py``; bổ sung diễn giải tay khi chèn vào luận văn._

## 4.5 Kết quả PDR

### Mục tiêu

- Kiểm tra **cải tiến** có làm **mất gói nhiều hơn** Baseline hay không; nếu PDR **giảm nhẹ** thì **ghi rõ** (hop, đường đi, leaf hạn chế relay).

### Ghi nhận tự động (PDR giảm nhẹ)

- **S1**: Full 90.67% < Baseline 93.37% — đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.
- **S2**: Full 77.14% < Baseline 88.11% — đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.
- **S3**: Full 77.76% < Baseline 82.36% — avg hop cao hơn Baseline (1.16 vs 1.05); đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.

| Kịch bản | Baseline | B–Leaf | B–Leaf+Grad |
|---|---:|---:|---:|
| S1 | 93.37 | 95.95 | 90.67 |
| S2 | 88.11 | 83.05 | 77.14 |
| S3 | 82.36 | 87.69 | 77.76 |
| S4 | 46.53 | 98.54 | 89.71 |
| S5 | 44.93 | 96.00 | 95.86 |
| S6 | 64.92 | 97.79 | 97.05 |
| S7 | 47.55 | 99.54 | 98.97 |
| S8 | 68.49 | 97.45 | 99.16 |
| S9 | 40.51 | 98.17 | 99.51 |
| S10 | 63.63 | 94.29 | 92.95 |
| S11 | 50.62 | 97.96 | 97.96 |
| S12 | 83.48 | 98.44 | 94.82 |

## 4.6 Kết quả độ trễ (Average delay)

### Mục tiêu

- Xem cải tiến có làm **route / delivery chậm hơn** không (độ trễ ACK end-to-end trong mô phỏng).

### Khung phân tích (định tính)
- **Baseline**: độ trễ thường **tăng mạnh** khi flooding / relay đồng loạt → collision & queueing.
- **Backbone–Leaf**: giảm **relay dư**, giảm va chạm → độ trễ có thể **giảm**.
- **Gradient**: **đôi khi độ trễ tăng nhẹ** — chọn đường tránh quá tải, **thêm hop** leaf–leaf; đây là trade-off “research” hợp lý.


### Ghi nhận tự động (Full chậm hơn Baseline ≥ 5 ms)

- Không phát hiện chênh lệch độ trễ Full > Baseline quá ngưỡng trên (có thể chỉnh ngưỡng trong script).


| Kịch bản | Baseline (ms) | B–Leaf (ms) | Full (ms) |
|---|---:|---:|---:|
| S1 | 2148.65 | 2000.00 | 2000.00 |
| S2 | 2123.75 | 2000.00 | 2000.00 |
| S3 | 2093.54 | 2009.93 | 2011.20 |
| S4 | 4580.00 | 2041.20 | 2162.26 |
| S5 | 4455.81 | 2699.67 | 2329.11 |
| S6 | 2687.50 | 2084.30 | 2072.82 |
| S7 | 5353.53 | 2195.12 | 2490.07 |
| S8 | 2453.38 | 2236.08 | 2236.08 |
| S9 | 5120.48 | 2504.20 | 2254.78 |
| S10 | 2374.79 | 2189.72 | 2214.42 |
| S11 | 6694.74 | 2505.26 | 2505.26 |
| S12 | 2705.88 | 2281.51 | 2135.08 |

## 4.7 Kết quả control overhead (quan trọng)

### Định nghĩa trong mô phỏng

- **Total control** = **HELLO + UPDATE** (lũy kế, toàn mạng). Đây là chỉ số chính cho **flooding / DSDV control-plane**.
- Trường **control_dropped** (nếu có trong ``summary.txt``): gói điều khiển không xử lý được (ví dụ năng lượng, filter backbone); dùng **bổ trợ** khi phân tích nghẽn relay điều khiển.

### Khung H1 (giảm overhead)
- **Baseline**: overhead **tăng mạnh** khi số nút / tải tăng (mọi nút tham gia relay điều khiển đầy đủ).
- **Backbone–Leaf**: **giảm rõ** — backbone gánh phần lớn relay đa chặng điều khiển; leaf **không participate full** như Baseline.
- **Full + Gradient**: có thể **tăng nhẹ** so với chỉ Backbone–Leaf do **coordination** (gradient, vai trò), nhưng thường **vẫn thấp hơn Baseline nhiều**.


| Kịch bản | Baseline TC | B–Leaf TC | Full TC | BL vs Base | Full vs Base |

|---|---:|---:|---:|---:|---:|
| S1 | 337 | 243 | 241 | -94 | -96 |
| S2 | 266 | 228 | 239 | -38 | -27 |
| S3 | 275 | 213 | 208 | -62 | -67 |
| S4 | 1090 | 816 | 791 | -274 | -299 |
| S5 | 1048 | 802 | 790 | -246 | -258 |
| S6 | 1004 | 782 | 784 | -222 | -220 |
| S7 | 1765 | 1395 | 1406 | -370 | -359 |
| S8 | 1821 | 1376 | 1369 | -445 | -452 |
| S9 | 1747 | 1389 | 1399 | -358 | -348 |
| S10 | 1795 | 1346 | 1342 | -449 | -453 |
| S11 | 3831 | 2844 | 2844 | -987 | -987 |
| S12 | 3812 | 2800 | 2805 | -1012 | -1007 |

## 4.8.1 Route stability — Route changes

- **Baseline**: **route churn** cao hơn khi relay đông / link biến động → nhiều invalidation.
- **Backbone–Leaf**: bảng tuyến **ổn định hơn** trên leaf (single-parent), ích đổi trên backbone.
- **Gradient**: tuning tốt → churn **giảm**; tuning aggressive (đổi cha / gradient) → có thể **tăng nhẹ** — cần đối chiếu số liệu.


| Kịch bản | Baseline | B–Leaf | Full |
|---|---:|---:|---:|
| S1 | 101 | 206 | 200 |
| S2 | 129 | 172 | 186 |
| S3 | 122 | 153 | 148 |
| S4 | 1141 | 1623 | 1765 |
| S5 | 1155 | 1762 | 1632 |
| S6 | 1104 | 1592 | 1673 |
| S7 | 4110 | 3419 | 3968 |
| S8 | 3750 | 3686 | 3762 |
| S9 | 4279 | 4030 | 4171 |
| S10 | 3657 | 3033 | 3107 |
| S11 | 17684 | 8922 | 8922 |
| S12 | 18827 | 9549 | 9683 |

## 4.8.2 Forwarding load — phân bố tải relay (proxy: tx_count)

- **tx_count** trên mỗi nút = tổng lần **phát** (HELLO + UPDATE + DATA + ACK); dùng làm **proxy** cho tải relay / airtime.
- **Phương tuyến nóng**: một vài nút có TX **cao hơn hẳn** đám còn lại → **variance** TX lớn.
- **Cân bằng tốt**: variance **thấp** hơn khi tải được dàn đều (Backbone/Gradient).

- **Baseline DSDV**: variance(tx_count)=1293.1, max_tx=149

- **DSDV + Backbone-Leaf**: variance(tx_count)=1310.6, max_tx=143

- **DSDV + Backbone-Leaf + Gradient**: variance(tx_count)=1361.5, max_tx=143


Biểu đồ cột (top nodes): ``forwarding_top_nodes_S8.png`` (nếu đã tạo).
