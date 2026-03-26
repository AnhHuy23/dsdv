# Báo cáo hệ thống: Bluetooth Mesh DSDV Routing (project `dsdv_1`)

## 1) Hệ thống này là gì?

Đây là một **firmware chạy trên Zephyr / nRF Connect SDK** triển khai một **Vendor Model Bluetooth Mesh** (Chat Client) được mở rộng để:

- **Chạy định tuyến chủ động DSDV** (Destination-Sequenced Distance Vector) trên nền Bluetooth Mesh.
- **Duy trì routing table** (đích → next hop → hop count → sequence number).
- **Gửi/nhận gói DATA unicast qua nhiều hop** theo route DSDV (forward theo `next_hop`).
- **Thu thập và hiển thị “network metrics” cơ bản** (hop count, RSSI tới “target”, TTL, timestamp) và **đo latency ước lượng** qua cơ chế ACK (RTT/2).
- **Điều khiển LED từ xa** (gửi lệnh toggle, node đích blink LED).
- Cung cấp **shell commands** qua UART để quan sát trạng thái, routes, neighbors RSSI, gửi metrics, bật/tắt relay, gửi LED toggle.

Project có cấu trúc/entrypoint theo Zephyr:

- `src/main.c`: bật Bluetooth và init Mesh.
- `src/model_handler.c`: composition (elements/models), shell commands, format “hiển thị”.
- `src/chat_cli.c` + `include/chat_cli.h`: DSDV core (HELLO/UPDATE), routing table, TTL policy, duplicate cache, gửi/forward DATA, ACK latency, LED message.
- `prj.conf`: cấu hình Zephyr + Bluetooth Mesh (buffer, segmentation, log, shell…).

## 2) Các “thứ hệ thống hiển thị” là gì?

Hệ thống **không có UI đồ họa**; toàn bộ hiển thị/quan sát thông qua:

- **UART Shell** (các lệnh bắt đầu bằng `chat ...`).
- **Log (LOG_INF/LOG_WRN)** của Zephyr (ví dụ routing table định kỳ mỗi ~30s).
- **LED trên DK** (blink theo event).

Các nội dung được in ra chủ yếu:

- **Trạng thái provisioning** (đã provision chưa, địa chỉ unicast của node).
- **Bảng định tuyến DSDV**: Dest / Next / Hops / Seq / Age(s).
- **Neighbor RSSI**: danh sách neighbor + RSSI (dBm) (trong “window” thời gian).
- **Network metrics** khi node đích nhận được DATA: hop_count, rssi_dbm, initial_ttl, request_ack, timestamp.
- **Latency**: RTT và latency ước lượng từ metrics ACK.
- **Thông tin forward/LED**: log khi gửi/forward LED toggle.

## 3) Flow tổng thể (luồng chạy của firmware)

### 3.1. Boot và khởi tạo Mesh

1. `main()` in “Initializing…”
2. `bt_enable(bt_ready)`; trong `bt_ready()`:
  - init LED + buttons (DK)
  - `bt_mesh_init(..., model_handler_init())`
  - `settings_load()` (nếu bật `CONFIG_SETTINGS`)
  - enable provisioning: `bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT)`

Khi Mesh stack “start model”, callback start của model Chat CLI được gọi, và DSDV bắt đầu schedule timer/work.

### 3.2. DSDV: HELLO / UPDATE chạy định kỳ (proactive)

Trong `src/chat_cli.c`, model init/start sẽ schedule 3 “k_work_delayable”:

- **HELLO**: bắt đầu sau ~2–7s, sau đó reschedule theo cơ chế adaptive + jitter.
- **UPDATE**: bắt đầu sau ~3–5s, sau đó reschedule adaptive.
- **Print routes**: in routing table định kỳ (mặc định 30s/lần sau lần đầu).

#### HELLO flow

- Node publish opcode `BT_MESH_CHAT_CLI_OP_DSDV_HELLO` (vendor opcode 0x12).
- TTL cho HELLO cố định: `TTL_HELLO = 1` (chỉ neighbor 1-hop).
- HELLO chứa:
  - `src` (địa chỉ node gửi)
  - `seq_num` (sequence number của node gửi)

Khi nhận HELLO:

- Chỉ accept nếu `ctx->addr == hello.src` (chống giả mạo/chuyển tiếp HELLO).
- Lọc RSSI theo ngưỡng (hysteresis): node mới ~-75 dBm, node đã có route ~-80 dBm.
- Cập nhật RSSI neighbor (EWMA smoothing).
- Upsert route 1-hop: dest = neighbor, next_hop = neighbor, hop=1.

#### UPDATE flow

- Node publish opcode `BT_MESH_CHAT_CLI_OP_DSDV_UPDATE` (vendor opcode 0x13).
- TTL cho UPDATE bị “cap”: `TTL_UPDATE_CAP = 3` (giới hạn lan truyền, giảm flood).
- UPDATE có header:
  - `src` (node gửi)
  - `num_entries`
  - `flags` (reserved)
- Payload: danh sách entry `{dest, hop_count, seq_num}`.

Khi nhận UPDATE:

- Với mỗi entry: hop_count được +1 (tính thêm hop từ sender sang receiver), và `next_hop` sẽ là **sender** (`ctx->addr`).
- Nếu entry là invalid (`hop_count == 0xFF`), propagate invalid.
- Upsert routing table dựa trên `seq_num` (freshness) và tie-break theo hop/RSSI/settle-time.

### 3.3. Data plane: gửi/forward DATA + ACK latency

#### Gửi metrics (DATA)

Shell gọi `bt_mesh_chat_cli_metrics_send(&chat, dest)`:

- Tìm route DSDV tới `dest` (`find_route(dest)`), nếu không có thì báo `-ENOENT`.
- Tạo `dsdv_data_packet`:
  - `src`, `dest`, `seq_num` (dùng uptime ms)
  - `hop_count` khởi tạo 1 (sẽ tăng khi forward)
  - `path_len` + `path_nodes[]` (path vector, tối đa 8)
  - `metrics` (`bt_mesh_network_metrics`): src_addr, about_addr, timestamp, rssi_dbm, initial_ttl, hop_count, request_ack
  - `collect_relay_metrics = 1` (cờ này hiện **chỉ có trong struct**, chưa thấy relay gửi metrics riêng trong code hiện tại)
- Gửi unicast tới `route->next_hop`, với TTL được tính:
  - `ttl = hop_count(route) + 1`, clamp max `TTL_DEFAULT_MAX = 10`

#### Nhận/forward DATA

Khi node nhận DATA:

- Nếu `pkt.dest == my_addr`: đây là node đích
  - in log “RECV DATA …”
  - nếu `request_ack` thì gửi `METRICS_ACK` về (unicast trực tiếp theo ctx hiện tại)
  - gọi handler `network_metrics` để in ra metrics lên shell
- Nếu không phải đích:
  - Chống lặp bằng duplicate cache `seen_duplicate(src, seq_num)`
  - `pkt.hop_count++` và append `path_nodes` nếu còn chỗ
  - Tìm route tới `pkt.dest`, forward tới `route->next_hop` với TTL tính theo route

#### ACK latency (RTT/2)

- Node đích gửi opcode `BT_MESH_CHAT_CLI_OP_METRICS_ACK` (vendor opcode 0x0F), payload chứa:
  - `src_addr` (địa chỉ của node gửi metrics)
  - `original_timestamp` (echo timestamp của metrics)
- Node nguồn nhận ACK:
  - tính `rtt = now - original_timestamp`
  - in lên shell: RTT và latency ước lượng `rtt/2`

### 3.4. LED toggle qua DSDV

Shell gọi `bt_mesh_chat_cli_led_toggle_send(&chat, dest)`:

- Tìm route tới dest, gửi `led_toggle_message {src, dest, seq_num}` tới `next_hop` với TTL theo route.

Khi nhận LED toggle:

- Chống lặp bằng duplicate cache.
- Nếu là đích: `mesh_led_blink(3)` (blink 3 lần).
- Nếu không: forward theo next hop.

## 4) Flow “hiển thị/quan sát” (shell + log + LED)

### 4.1. Shell commands hiện có trong code

Các lệnh được đăng ký trong `src/model_handler.c` (UART shell):

- `chat status`
  - Hiển thị provisioning status + địa chỉ node (nếu đã provision).
- `chat metrics_to <addr>`
  - Gửi metrics (DATA) tới node đích theo DSDV.
  - Kết quả hiển thị chủ yếu nằm ở node đích (NETWORK METRICS) và node nguồn (LATENCY khi có ACK).
- `chat routes`
  - In routing table DSDV (dest/next/hops/seq/age).
- `chat verify_route <addr>`
  - In route hiện tại tới addr, sau đó gửi metrics để bạn đối chiếu “thực tế” (xem log/path vector ở node đích).
- `chat neighbors`
  - In RSSI theo neighbor trong window gần đây.
- `chat relay <on/off>`
  - Bật/tắt relay qua config server (giúp test topology backbone/leaf).
- `chat led_toggle <addr>`
  - Gửi lệnh để node đích blink LED.

### 4.2. Log định kỳ

Trong `src/chat_cli.c`, routing table được log định kỳ qua `LOG_INF` mỗi ~30s (sau lần đầu ~10s), gồm:

- Node address
- Dest / Next / Hops / Seq / Age(s)
- Nhãn trạng thái (INVALID/OLD) theo điều kiện tuổi route.

### 4.3. LED indicators

- Khi node nhận `LED_TOGGLE` và nó là đích: blink 3 lần.
- (Trong code hiện tại) chưa thấy “blink khi provision” hay “blink mỗi packet receive” như README mô tả; phần này nên coi là **chưa/không còn implement** trong revision hiện tại.

## 5) Các thông số/metrics cụ thể (đúng theo code hiện tại)

### 5.1. Network metrics được gửi trong DATA (`struct bt_mesh_network_metrics`)

Các trường thực sự được populate khi gửi `metrics_to`:

- `src_addr`: địa chỉ node nguồn (unicast)
- `about_addr`: bằng `dest` (node target)
- `timestamp`: `k_uptime_get_32()` tại thời điểm tạo packet
- `rssi_dbm`: RSSI “neighbor” tới target (nếu không có thì set -90)
  - Lưu ý: đây là RSSI lấy từ cache neighbor, **không phải** RSSI end-to-end
- `initial_ttl`: TTL dự kiến cho DATA (route->hop_count + 1)
- `hop_count`: hop_count từ routing table (hoặc fallback 1 nếu không có)
- `request_ack`: luôn set 1 trong flow này (để đo RTT)

Hiển thị trên shell (ở node đích):

- Hop count, RSSI (dBm), TTL, Req-ACK, Timestamp.

### 5.2. Latency (từ ACK)

Hiển thị trên shell (ở node nguồn):

- RTT (ms) và latency ước lượng = RTT/2.

### 5.3. Neighbor RSSI

- RSSI được update từ `ctx->recv_rssi` khi nhận HELLO/UPDATE.
- EWMA smoothing rất “mượt”: new = (31·prev + 1·now) / 32.
- Window “valid”: `NEIGHBOR_RSSI_VALID_WINDOW_MS = 90000` (90s).

### 5.4. DSDV routing table fields

Mỗi route entry (`struct dsdv_route_entry`):

- `dest`, `next_hop`, `hop_count`
  - `hop_count == 0xFF` được dùng như **invalidation** (route broken)
- `seq_num`: sequence number DSDV (odd/even dùng cho invalidation theo convention trong code)
- `last_update_time`: age dùng để expire
- `changed`: phục vụ incremental UPDATE

## 6) TTL policy (chính sách TTL)

Trong `dsdv_calc_ttl()`:

- **HELLO**: TTL = 1
- **UPDATE**: TTL = 3 (cap)
- **UNICAST DATA**: TTL = hop_count(route)+1, clamp tối đa 10
- (Có enum MSG_BROADCAST_APP nhưng hiện tại không thấy sử dụng thực tế)

Mục tiêu: giới hạn phạm vi flood UPDATE/HELLO, còn DATA thì đủ TTL để đi đúng số hop.

## 7) Expiration/timeout và ổn định route (stability)

Các “knob” quan trọng (đang hardcode trong `src/chat_cli.c`):

- `DSDV_NEIGHBOR_TIMEOUT_MS = 45000` (1-hop)
- `DSDV_ROUTE_TIMEOUT_MS = 120000` (multi-hop)
- `ROUTE_SETTLE_TIME_MS = 10000` (chống “flapping”)
- `UPDATE_MIN_INTERVAL_MS = 3000` (rate limit UPDATE)

Hành vi nổi bật:

- Nếu route quá tuổi: bị invalid (hop=0xFF) rồi mới xóa hẳn sau đó.
- Khi route thay đổi: schedule UPDATE với delay + jitter để “gom” thay đổi và tránh bùng nổ traffic.
- UPDATE có cơ chế “rotation” và giới hạn `MAX_UPDATE_ENTRIES = 12` để giảm payload.

## 8) Những điểm README nói nhưng code hiện tại chưa có (chênh lệch)

`README.md` mô tả nhiều tính năng như:

- battery ADC, congestion detection, relay metrics (mỗi relay gửi metrics riêng),
- convergence monitoring (stable vs converging), structure_to / convergence_to / history…

Nhưng trong code mình đọc ở revision hiện tại:

- **Không có file `battery_adc.*`** trong repo.
- **Không có shell command** `battery`, `structure_to`, `convergence_to`, `show_history`, `clear_history`.
- `bt_mesh_chat_cli_structure_request()` được **declare trong header** nhưng **không có implementation** trong `src/chat_cli.c`.
- `collect_relay_metrics` tồn tại trong packet struct nhưng **không thấy** code gửi relay-metrics riêng hay cache/print history.

Vì vậy, báo cáo này ưu tiên mô tả **những gì đang thực sự chạy theo code hiện tại**; các mục trên nên coi là “đã từng có/định hướng” nhưng hiện chưa có trong repo này.

## 9) File/điểm đọc code quan trọng (để bạn tự lần tiếp)

- `src/main.c`: init Bluetooth/Mesh.
- `src/model_handler.c`: composition + shell commands + format output trên shell.
- `src/chat_cli.c`: DSDV HELLO/UPDATE, routing table + forward DATA/ACK/LED.
- `include/chat_cli.h`: định nghĩa opcodes + packet formats.
- `prj.conf`: buffer/log/shell/mesh settings.

## 10) Gợi ý cách “chạy và quan sát” (thực tế theo code hiện tại)

Sau khi flash và provision:

- Trên mỗi node:
  - `chat status`
  - `chat routes` (đợi vài chục giây để DSDV hội tụ ban đầu)
  - `chat neighbors`
- Test unicast multi-hop:
  - Trên node nguồn: `chat metrics_to 0x0003`
  - Trên node đích: xem “NETWORK METRICS”
  - Trên node nguồn: xem dòng “[LATENCY] …”
- Test relay/backbone:
  - `chat relay on` (node backbone)
  - `chat relay off` (node leaf)
- Test LED:
  - `chat led_toggle 0x0003`

