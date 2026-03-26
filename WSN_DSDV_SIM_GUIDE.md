# Mô phỏng WSN DSDV (10–100 nodes, auto)

File chạy: `wsn_dsdv_sim.py`

## Cài dependency

```bash
python -m pip install matplotlib pillow
```

Nếu muốn xuất MP4, cần `ffmpeg` trong PATH. Nếu không có, script sẽ tự lưu GIF fallback.

## Chạy auto 10→100 node (mặc định)

```bash
python wsn_dsdv_sim.py
```

## Chạy auto và chỉnh bước

```bash
python wsn_dsdv_sim.py --min 10 --max 100 --step 10
python wsn_dsdv_sim.py --min 10 --max 100 --step 5
```

## Chạy 1 case

Hiện script mặc định auto sweep; nếu bạn muốn chỉ chạy 1 case, dùng `--auto` là không cần (vẫn auto).
Nếu bạn muốn mình thêm chế độ `--single` để tắt auto, nói mình sẽ bổ sung.

## Output

Thư mục `sim_outputs/`:

- `batch_summary.csv`: KPI theo từng N
- `n10/`, `n20/`, … `n100/`:
  - `summary.txt`
  - `metrics_timeseries.csv`
  - `routing_table_node0.csv`
  - `simulation.mp4` (hoặc `simulation.gif`)

## KPI được xuất

- `pdr`, `avg_latency_s`, `avg_hops`
- `hello_packets`, `update_packets`
- `drop_no_route`, `drop_link`
- `route_changes`
- `backbone_nodes` (số node đang ở vai trò BACKBONE)

## Môi trường cơ bản (có thể chỉnh)

- `--area`: kích thước vùng mô phỏng
- `--range`: bán kính radio
- `--hello`, `--update`: chu kỳ HELLO/UPDATE
- `--timeout`: timeout route
- `--data-period`: chu kỳ sinh traffic DATA
- `--seed`: tái lập topology/traffic
