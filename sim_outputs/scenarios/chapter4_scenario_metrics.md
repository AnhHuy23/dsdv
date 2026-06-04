# Chương 4 — Hình và ghi chú theo kịch bản S1–S12

_File tự động từ ``plot_scenario_pdr_comparison.py``; bổ sung diễn giải tay khi chèn vào luận văn._

## 4.5 Kết quả PDR

### Mục tiêu

- Kiểm tra **cải tiến** có làm **mất gói nhiều hơn** Baseline hay không; nếu PDR **giảm nhẹ** thì **ghi rõ** (hop, đường đi, leaf hạn chế relay).

### Ghi nhận tự động (PDR giảm nhẹ)

- **S1**: Full 85.12% < Baseline 93.37% — đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.
- **S2**: Full 75.53% < Baseline 88.11% — đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.
- **S3**: Full 71.25% < Baseline 82.36% — avg hop cao hơn Baseline (1.17 vs 1.05); đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.

| Kịch bản | Baseline | B–Leaf | B–Leaf+Grad |
|---|---:|---:|---:|
| S1 | 93.37 | 91.67 | 85.12 |
| S2 | 88.11 | 91.27 | 75.53 |
| S3 | 82.36 | 81.05 | 71.25 |
| S4 | 46.53 | 98.23 | 86.48 |
| S5 | 44.93 | 94.43 | 86.94 |
| S6 | 64.92 | 94.59 | 92.98 |
| S7 | 47.55 | 95.81 | 95.81 |
| S8 | 68.49 | 99.54 | 96.89 |
| S9 | 40.51 | 97.85 | 97.75 |
| S10 | 63.63 | 92.05 | 89.11 |
| S11 | 50.62 | 100.00 | 100.00 |
| S12 | 83.48 | 97.22 | 94.49 |

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
| S3 | 2093.54 | 2010.78 | 2012.12 |
| S4 | 4580.00 | 2356.32 | 2088.79 |
| S5 | 4455.81 | 2419.23 | 2320.00 |
| S6 | 2687.50 | 2085.47 | 2087.41 |
| S7 | 5353.53 | 2183.21 | 2183.21 |
| S8 | 2453.38 | 2033.82 | 2032.18 |
| S9 | 5120.48 | 2315.07 | 2248.28 |
| S10 | 2374.79 | 2311.21 | 2304.93 |
| S11 | 6694.74 | 2504.85 | 2504.85 |
| S12 | 2705.88 | 2162.44 | 2167.87 |

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
| S1 | 337 | 337 | 340 | +0 | +3 |
| S2 | 266 | 326 | 317 | +60 | +51 |
| S3 | 275 | 294 | 274 | +19 | -1 |
| S4 | 1090 | 1066 | 1090 | -24 | +0 |
| S5 | 1048 | 1079 | 1060 | +31 | +12 |
| S6 | 1004 | 1039 | 1035 | +35 | +31 |
| S7 | 1765 | 1885 | 1885 | +120 | +120 |
| S8 | 1821 | 1809 | 1796 | -12 | -25 |
| S9 | 1747 | 1914 | 1907 | +167 | +160 |
| S10 | 1795 | 1785 | 1780 | -10 | -15 |
| S11 | 3831 | 3876 | 3876 | +45 | +45 |
| S12 | 3812 | 3680 | 3675 | -132 | -137 |

## 4.8.1 Route stability — Route changes

- **Baseline**: **route churn** cao hơn khi relay đông / link biến động → nhiều invalidation.
- **Backbone–Leaf**: bảng tuyến **ổn định hơn** trên leaf (single-parent), ích đổi trên backbone.
- **Gradient**: tuning tốt → churn **giảm**; tuning aggressive (đổi cha / gradient) → có thể **tăng nhẹ** — cần đối chiếu số liệu.


| Kịch bản | Baseline | B–Leaf | Full |
|---|---:|---:|---:|
| S1 | 101 | 199 | 227 |
| S2 | 129 | 155 | 181 |
| S3 | 122 | 139 | 138 |
| S4 | 1141 | 2006 | 2041 |
| S5 | 1155 | 2126 | 2150 |
| S6 | 1104 | 1655 | 1693 |
| S7 | 4110 | 3663 | 3663 |
| S8 | 3750 | 4379 | 4367 |
| S9 | 4279 | 4522 | 4530 |
| S10 | 3657 | 3424 | 3392 |
| S11 | 17684 | 11963 | 11963 |
| S12 | 18827 | 9927 | 9555 |

## 4.8.2 Forwarding load — phân bố tải relay (proxy: tx_count)

- **tx_count** trên mỗi nút = tổng lần **phát** (HELLO + UPDATE + DATA + ACK); dùng làm **proxy** cho tải relay / airtime.
- **Phương tuyến nóng**: một vài nút có TX **cao hơn hẳn** đám còn lại → **variance** TX lớn.
- **Cân bằng tốt**: variance **thấp** hơn khi tải được dàn đều (Backbone/Gradient).

- **Baseline DSDV**: variance(tx_count)=1293.1, max_tx=149

- **DSDV + Backbone-Leaf**: variance(tx_count)=906.8, max_tx=141

- **DSDV + Backbone-Leaf + Gradient**: variance(tx_count)=1028.2, max_tx=146



