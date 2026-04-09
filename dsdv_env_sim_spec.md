# Đặc tả file mô phỏng WSN + DSDV

## 1. Mục tiêu và ràng buộc

File mô phỏng phải là một script Python 3 chạy độc lập bằng lệnh:

```bash
python dsdv_env_sim.py [options]
```

Yêu cầu cốt lõi:

- Không dùng `simpy`.
- Dùng vòng lặp thời gian rời rạc với bước `dt`.
- Mô phỏng DSDV đơn giản hóa với 3 luồng chính:
  - `HELLO` broadcast.
  - `UPDATE` broadcast với payload giới hạn số entry.
  - `DATA` unicast và forward theo bảng route.
- Mạng là tập node ngẫu nhiên trong vùng hình chữ nhật.
- Liên kết vật lý suy ra từ mô hình RSSI + ngưỡng, quy đổi ra khoảng cách tối đa `dmax`.
- Đồ thị phải liên thông dưới ràng buộc `max_degree`.
- Có thể thêm cạnh dư theo `extra_edge_factor`.
- Năng lượng là tùy chọn nhưng phải có sẵn trong file mẫu.
- Đầu ra phải ghi vào thư mục `out_dir/n{N}/`.

## 2. Thứ tự khối trong file

File phải được sắp xếp theo đúng thứ tự logic sau.

### 2.1 Docstring đầu file

Docstring phải mô tả:

- Mục tiêu mô phỏng.
- Cách chạy ví dụ.
- Phụ thuộc ngoài:
  - `matplotlib`
  - `pillow` cho GIF
  - `ffmpeg` cho MP4

### 2.2 Import

Phải có:

- `from __future__ import annotations`
- Import chuẩn:
  - `argparse`
  - `csv`
  - `math`
  - `random`
  - `collections.deque`
  - `dataclasses`
  - `pathlib.Path`
  - `typing`

### 2.3 Hằng số

Ví dụ:

- `INF_HOPS = 255`

### 2.4 Khối visualization, import matplotlib lazy

Phải có các hàm:

- `plot_topology_png(...)`
- `plot_metrics_chart(samples, ...)`
- `save_topology_animation(...)`

Yêu cầu:

- `plot_topology_png(...)`
  - Vẽ cạnh từ adjacency.
  - Scatter node.
  - Đánh dấu gateway.
  - Nhãn ID node.
- `plot_metrics_chart(samples, ...)`
  - 2x2 subplot.
  - Biểu diễn:
    - PDR %
    - latency
    - hops
    - hello/update cumulative
- `save_topology_animation(...)`
  - Dùng `FuncAnimation`.
  - Mỗi frame tô màu node theo vector năng lượng.
  - Nếu xuất MP4 thất bại thì fallback sang GIF.

### 2.5 Khối topology/radio

Phải có các hàm:

- `_rssi_from_dist`
- `_max_distance_from_rssi_threshold`
- `generate_positions`
- `constrained_connectivity_graph`
- `generate_random_topology`

Yêu cầu:

- `generate_positions`
  - Tạo vị trí ngẫu nhiên trong vùng chữ nhật.
- `_rssi_from_dist` và `_max_distance_from_rssi_threshold`
  - Quy đổi giữa khoảng cách và RSSI.
- `constrained_connectivity_graph`
  - Xây spanning tree từ `gateway_id`.
  - Sau đó thêm cạnh dư.
- `generate_random_topology`
  - Lặp `max_attempts` lần.
  - Mỗi lần dùng seed lệch đi một chút.
  - Nếu không tạo được topology hợp lệ thì báo lỗi rõ ràng.

### 2.6 Khối dữ liệu giao thức

Phải có các dataclass:

- `RouteEntry`
- `Message`
- `Node`

Schema mong muốn:

- `RouteEntry`
  - `dest`
  - `next_hop`
  - `hop_count`
  - `seq_num`
  - `last_update`
  - `changed`
- `Message`
  - `kind`
  - `src`
  - `sender`
  - `dst`
  - `payload`
  - `deliver_time`
- `Node`
  - tọa độ
  - routes
  - neighbor_rssi
  - timer hello/update
  - energy
  - counters

### 2.7 Lớp mô phỏng chính `DsdvEnvSim`

Phải có lớp chính `DsdvEnvSim` với các phương thức sau.

#### `__init__`

Phải nhận đủ knob cho:

- area
- RSSI
- graph
- DSDV
- traffic
- energy

Nhiệm vụ:

- Build topology.
- Init nodes.
- Init counters.
- Init samples.
- Init state để ghi video.

#### `_broadcast` và `_unicast`

Yêu cầu:

- Duy trì hàng đợi `in_flight`.
- Delay propagation phụ thuộc khoảng cách.
- Kiểm tra cạnh trong adjacency.
- Trừ năng lượng TX.

#### `_send_hello`, `_send_update`

Yêu cầu:

- Có lịch jitter.
- `UPDATE` chỉ mang số entry giới hạn bởi `update_payload_limit`.

#### `_upsert_route`

Quy tắc DSDV:

- Ưu tiên seq lớn hơn.
- Nếu cùng seq thì hop count nhỏ hơn thắng.

#### `_handle_hello`, `_handle_update`, `_handle_data`, `_forward_data`, `_gen_data_packet`

Yêu cầu xử lý:

- HELLO để học neighbor 1-hop.
- UPDATE để đồng bộ route.
- DATA để forward theo bảng route.
- Nếu node hết năng lượng thì không gửi/nhận.

#### `step()`

Mỗi bước phải thực hiện:

- Lặp node để xử lý hello/update.
- Expire route theo `route_timeout`.
- Sinh data theo `data_period`.
- Deliver `in_flight`.
- Trừ năng lượng RX.
- Ghi sample.
- Nếu có video thì append frame.
- Tăng `time += dt`.

#### `run(...)`

Phải hỗ trợ:

- `record_video`
- `video_stride`
- `video_max_frames`

#### `summary()`, `export_metrics_csv`, `export_routing_table_csv`

Phải xuất:

- Tổng kết chạy mô phỏng.
- CSV time series.
- CSV routing table mẫu.

## 3. Hợp đồng dữ liệu

### 3.1 `samples`

Mỗi bước phải là một dict có key cố định:

- `t`
- `pdr`
- `avg_latency_s`
- `avg_hops`
- `hello_packets`
- `update_packets`
- `route_changes`

Có thể thêm trường tương đương, nhưng các key trên phải luôn có.

### 3.2 `summary`

Phải trả về dict hoặc chuỗi dòng có ít nhất:

- pdr
- avg_latency_s
- avg_hops
- counters control/data
- drops
- route_changes

### 3.3 Routing table CSV

Snapshot 1 node, chỉ ghi route hợp lệ:

- `dest`
- `next_hop`
- `hop_count`
- `seq_num`
- `last_update`

Chỉ xuất các route có `hop_count < INF_HOPS`.

## 4. Hợp đồng thư mục đầu ra

Phải tạo cấu trúc:

```text
out_dir/
  n{N}/
    metrics_timeseries.csv
    routing_table_node.csv   # hoặc routing_table_node{focus}.csv
    summary.txt
    topology.png
    metrics_chart.png
    simulation.gif | simulation.mp4
```

## 5. CLI tối thiểu

CLI phải map 1-1 với tham số `__init__`.

### 5.1 Chạy mô phỏng

- `--nodes`
- `--duration`
- `--dt`
- `--seed`
- `--out-dir`
- `--focus-node`

### 5.2 Vùng

- `--area`
- `--area-w`
- `--area-h`
- `--gateway-id`
- `--max-degree`
- `--extra-edge-factor`
- `--max-attempts`

### 5.3 Radio

- `--rssi-1m`
- `--path-loss-n`
- `--rssi-threshold`
- `--noise-rssi-amp`

### 5.4 DSDV

- `--hello`
- `--update`
- `--timeout`
- `--update-payload-limit`

### 5.5 Traffic

- `--data-period`

### 5.6 Năng lượng

- `--initial-energy`
- `--energy-per-tx`
- `--energy-per-rx`

### 5.7 Hình ảnh và video

- `--no-plots`
- `--video`
- `--video-format`
- `--fps`
- `--video-stride`
- `--video-max-frames`

## 6. Hành vi lỗi và gợi ý

### 6.1 Không tạo được topology

Phải báo lỗi kèm gợi ý:

- nới `--rssi-threshold`
- tăng `--area`
- tăng `--max-degree`
- tăng `--max-attempts`

### 6.2 Thiếu matplotlib

Phải in `[WARN]` và bỏ qua PNG.

### 6.3 Thiếu hỗ trợ video

Tương tự, nếu thiếu `pillow` hoặc `ffmpeg` thì bỏ qua video và báo cảnh báo.

## 7. Phụ thuộc bên ngoài

- Chuẩn: `matplotlib` cho PNG và animation.
- GIF: `pillow`.
- MP4: `ffmpeg` là khuyến nghị.

## 8. Ghi chú triển khai

- Không dùng thư viện mô phỏng sự kiện rời rạc bên ngoài.
- Giữ mô hình đơn giản, dễ chạy lại theo seed.
- Mọi chỉ số phải xuất được để hậu xử lý.
- File nên là một script độc lập, không phụ thuộc vào các file firmware Zephyr.
