# WSN DSDV Python Simulation Guide

## Mục tiêu

Mô phỏng mạng WSN với thuật toán DSDV dựa trên logic code firmware hiện tại:

- HELLO/UPDATE proactive
- Routing table theo `dest -> next_hop -> hop -> seq`
- Gửi dữ liệu unicast multi-hop
- Thu thập các thông số chính
- Xuất video mô phỏng + bảng định tuyến mẫu

## Topology (luôn liên thông)

Các node được đặt trên **lưới (grid)** trong vùng `--area`, với khoảng cách giữa láng giềng lưới **≤ 0,95 × `--range`**. Như vậy đồ thị láng giềng theo tầm phủ sóng **luôn liên thông** (một thành phần), kể cả khi N nhỏ hay lớn — không còn trường hợp node “rơi rụng” ngẫu nhiên ngoài tầm.

## Cài môi trường

```bash
python -m pip install matplotlib
```

Nếu muốn xuất MP4, máy cần ffmpeg; nếu không có ffmpeg, script sẽ tự fallback sang GIF.

## Chạy 1 kịch bản

Ví dụ 30 node:

```bash
python wsn_dsdv_sim.py --nodes 30 --duration 120 --video --video-format mp4
```

## Chạy batch 10-100 node

```bash
python wsn_dsdv_sim.py --batch 10,30,50,100 --duration 120 --video --video-format mp4
```

## Output

Sau khi chạy, thư mục `sim_outputs` sẽ có:

- `batch_summary.csv`: tổng hợp KPI theo từng N node
- `n10/`, `n30/`, ... mỗi case có:
  - `summary.txt`
  - `metrics_timeseries.csv`
  - `routing_table_node0.csv`
  - `simulation.mp4` (hoặc `simulation.gif`)

## Các thông số môi trường cơ bản có thể chỉnh

- `--area` (mặc định 100): kích thước vùng mô phỏng
- `--range` (mặc định 30): bán kính truyền
- `--hello` (mặc định 5s): chu kỳ HELLO
- `--update` (mặc định 15s): chu kỳ UPDATE
- `--timeout` (mặc định 45s): timeout route
- `--data-period` (mặc định 2.5s): chu kỳ sinh traffic DATA
- `--seed`: random seed để tái lập kết quả

## KPI chính

- `PDR`: tỉ lệ gói DATA đến đích
- `avg_latency_s`: trễ trung bình (s)
- `avg_hops`: số hop trung bình
- `hello_packets`, `update_packets`: overhead control plane
- `drop_no_route`, `drop_link`: nguyên nhân mất gói
- `route_changes`: mức độ biến động routing

