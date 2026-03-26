# Báo cáo hệ thống (project hiện tại): Bluetooth Mesh DSDV + MCDS Backbone

## 1) Hệ thống này là gì?

Đây là firmware chạy trên **Zephyr / nRF Connect SDK** triển khai một **Vendor Model Bluetooth Mesh** (Chat Client) và mở rộng để:

- **Định tuyến chủ động DSDV** (Destination-Sequenced Distance Vector) bằng các gói `HELLO` + `UPDATE`.
- Duy trì **routing table**: `dest -> next_hop -> hop_count -> seq_num -> age`.
- **Chuyển tiếp gói DATA unicast multi-hop** theo route DSDV, kèm **path vector**.
- Thu thập và in **network metrics** (hop count, RSSI “đến target”, TTL, timestamp) và đo **latency** theo ACK (RTT/2).
- **Remote LED toggle** (đích blink LED 3 lần).
- Cung cấp **UART shell** để quan sát routing/neighbors/metrics và điều khiển relay/LED.

Điểm mới trong project này: có thêm **MCDS backbone selection** (vai trò node `BACKBONE`/`LEAF`), tự động:

- **Đánh giá vai trò định kỳ** dựa trên degree, RSSI trung bình, PDR “HELLO nhận được”.
- **Bật/tắt relay** tùy vai trò (BACKBONE bật relay; LEAF tắt relay).
- **Điều chỉnh TTL** cho UPDATE/broadcast dựa trên vai trò (BACKBONE TTL cao hơn).

## 2) “Hiển thị” những gì?

Không có UI đồ họa. Quan sát bằng:

- **UART Shell**: nhóm lệnh `chat ...`.
- **Zephyr logs**: in routing table định kỳ + log backbone role.
- **LED** trên board: blink khi nhận `LED_TOGGLE` (đích blink 3 lần).

## 3) Kiến trúc code / entrypoints

- `src/main.c`
  - `bt_enable(bt_ready)` → init LED/buttons → `bt_mesh_init(..., model_handler_init())` → load settings → enable provisioning.
- `src/model_handler.c`
  - Khai báo composition (CFG_SRV + HEALTH_SRV + CHAT_CLI).
  - Đăng ký UART shell commands và format output (routes, neighbors, backbone…).
  - Handler khi nhận metrics/ack để in ra shell.
- `src/chat_cli.c` + `include/chat_cli.h`
  - DSDV core: HELLO/UPDATE, routing table, expire/invalidate, duplicate cache.
  - DATA forwarding + METRICS_ACK.
  - MCDS backbone selection: degree/RSSI/PDR score → role → relay config → TTL policy.

## 4) Flow tổng thể (runtime flow)

### 4.1 Boot / init

1. Boot → `main()` in “Initializing…”
2. `bt_enable(bt_ready)`
3. `bt_mesh_init(..., model_handler_init())`
4. `settings_load()` (nếu bật `CONFIG_SETTINGS`)
5. `bt_mesh_prov_enable(...)`

Khi model start, `chat_cli.c` schedule các work định kỳ:

- HELLO work
- UPDATE work
- Print routing table work
- Backbone selection work (delay ban đầu)

### 4.2 DSDV control-plane: HELLO

**Gửi HELLO**

- Publish opcode `BT_MESH_CHAT_CLI_OP_DSDV_HELLO` (0x12).
- TTL: `TTL_HELLO = 1` (1-hop neighbor).
- Payload `struct dsdv_hello` gồm:
  - `src`, `seq_num`
  - `my_degree` (số neighbor active)
  - `my_role` (UNKNOWN/BACKBONE/LEAF)
- HELLO interval là **adaptive + jitter** (càng nhiều route càng gửi thưa hơn).

**Nhận HELLO**

- Chỉ accept khi `ctx->addr == hello.src` (HELLO không được forward).
- Lọc RSSI theo hysteresis (route đã tồn tại cho phép RSSI thấp hơn).
- Update RSSI neighbor (EWMA).
- Upsert route 1-hop: `dest=neighbor`, `next_hop=neighbor`, `hop=1`, `seq=...`
- Đồng thời cache `neighbor_backbone_info[]` để backbone election (degree/role + hello_rx_count).

### 4.3 DSDV control-plane: UPDATE

**Gửi UPDATE**

- Publish opcode `BT_MESH_CHAT_CLI_OP_DSDV_UPDATE` (0x13).
- UPDATE là incremental nếu có `changed`; payload giới hạn `MAX_UPDATE_ENTRIES=12`.
- TTL UPDATE **phụ thuộc role**:
  - BACKBONE: cap cao hơn
  - LEAF: cap thấp hơn

**Nhận UPDATE**

- Với mỗi entry: `actual_hops = hop + 1` (trừ trường hợp invalid `0xFF`).
- `next_hop = ctx->addr`.
- Upsert dựa trên `seq_num` (freshness), tie-break bằng hop + RSSI + settle-time chống flapping.
- Route timeout khác nhau giữa 1-hop neighbor và multi-hop.

### 4.4 Data-plane: DATA + ACK latency

**Gửi metrics_to**

Shell gọi `bt_mesh_chat_cli_metrics_send(dest)`:

- Nếu có route: tạo `dsdv_data_packet` có `metrics`:
  - `src_addr`, `about_addr`, `timestamp`
  - `rssi_dbm` (lấy từ neighbor RSSI cache, fallback -90)
  - `initial_ttl` (hop+1)
  - `hop_count` (từ route)
  - `request_ack=1`
- Gửi unicast tới `next_hop` với TTL = `hop+1` (clamp max 10).

**Forward DATA**

- Duplicate cache theo `(src, seq_num)`.
- Tăng `pkt.hop_count`, append `path_nodes[]` (tối đa 8).
- Forward tới `route->next_hop`.

**ACK latency**

- Node đích gửi `BT_MESH_CHAT_CLI_OP_METRICS_ACK` (0x0F) echo `original_timestamp`.
- Node nguồn tính RTT và in latency ước lượng `RTT/2` trên shell.

### 4.5 MCDS Backbone selection (điểm mới)

Định kỳ (sau delay ban đầu và mỗi ~30s):

- Tính `degree` = số neighbor active (RSSI window).
- Tính `avg RSSI` trên neighbor active.
- Tính `avg PDR` từ số HELLO nhận được / số HELLO kỳ vọng trong window.
- Nếu node “edge/weak/unreliable” thì ép LEAF.
- Tính score:
  - `score = degree*100 + rssi_normalized + pdr_bonus`
- Greedy so sánh score với neighbor 1-hop:
  - score cao nhất → BACKBONE (tie-break theo địa chỉ nhỏ hơn)
- Connectivity check: nếu không có backbone neighbor và node có score cao nhất cluster → force BACKBONE
- Apply relay config:
  - BACKBONE → relay enabled
  - LEAF → relay disabled

## 5) Shell commands (đúng theo code hiện tại)

Các lệnh `chat` hiện có:

- `chat status`: trạng thái provisioned + địa chỉ unicast.
- `chat routes`: in bảng DSDV (dest/next/hops/seq/age).
- `chat neighbors`: in RSSI theo neighbor (trong window).
- `chat metrics_to <addr>`: gửi DATA(metrics) tới node đích theo DSDV.
- `chat verify_route <addr>`: in route hiện tại rồi gửi metrics để verify.
- `chat relay <on/off>`: cấu hình relay thủ công.
- `chat led_toggle <addr>`: gửi lệnh remote LED blink.
- `chat backbone`: in vai trò backbone hiện tại + score + danh sách backbone neighbors.

## 6) Thông số/knobs quan trọng trong code

### 6.1 DSDV timing/limits

- `DSDV_NEIGHBOR_TIMEOUT_MS = 45000`
- `DSDV_ROUTE_TIMEOUT_MS = 120000`
- `NEIGHBOR_RSSI_VALID_WINDOW_MS = 90000`
- `MAX_UPDATE_ENTRIES = 12`
- `UPDATE_MIN_INTERVAL_MS = 3000`
- `ROUTE_SETTLE_TIME_MS = 10000`

### 6.2 TTL policy (role-adaptive)

- HELLO: TTL=1
- UPDATE: TTL cap phụ thuộc role
  - BACKBONE: cao hơn
  - LEAF: thấp hơn
- Unicast DATA: TTL = `hop+1` (max 10)

### 6.3 Backbone constants

- `BACKBONE_RSSI_THRESHOLD = -70`, `BACKBONE_RSSI_REJECT = -80`
- `BACKBONE_MIN_DEGREE = 3`
- `BACKBONE_EVAL_INTERVAL_MS = 30000`, `BACKBONE_INITIAL_DELAY_MS = 15000`
- `BACKBONE_PDR_REJECT = 50`, `BACKBONE_PDR_WINDOW_MS = 60000`
- `BACKBONE_EXPECTED_HELLO_MS = 12000`

## 7) Chênh lệch so với README.md (những gì README nói nhưng code chưa thấy)

`README.md` có đề cập battery / convergence / structure_to / history / relay-metrics… nhưng trong code hiện tại:

- **Không thấy** module battery (`battery_adc.*` không có trong repo).
- **Không có** shell commands `battery`, `structure_to`, `convergence_to`, `show_history`, `clear_history`.
- `bt_mesh_chat_cli_structure_request()` vẫn là declaration trong header nhưng **không thấy implementation** trong `src/chat_cli.c`.
- `collect_relay_metrics` có trong struct nhưng **chưa thấy** code relay gửi metrics riêng.
