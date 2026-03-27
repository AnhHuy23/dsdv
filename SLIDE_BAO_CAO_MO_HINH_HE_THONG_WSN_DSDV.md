# Slide 1 - Tieu de
## Mo hinh hoa he thong WSN su dung DSDV tren Bluetooth Mesh

- De tai: Phan tich he thong tu code firmware hien tai
- Nen tang: Zephyr / nRF Connect SDK / Bluetooth Mesh
- Muc tieu: Hieu kien truc, luong du lieu, diem nghen va cach danh gia

---

# Slide 2 - Bai toan
## Van de can giai quyet

- Xay dung mang WSN da hop tren Bluetooth Mesh
- Dinh tuyen chu dong bang DSDV de tim duong toi node dich
- Giam mat goi, dam bao do tre chap nhan duoc
- Theo doi chat luong lien ket qua RSSI, hop count, RTT

---

# Slide 3 - Tong quan kien truc
## Cac module chinh trong code

- `src/main.c`: khoi tao Bluetooth, Mesh, provisioning
- `src/model_handler.c`: shell command, in thong tin he thong
- `src/chat_cli.c`: DSDV core (HELLO, UPDATE, DATA, ACK, LED)
- `include/chat_cli.h`: dinh nghia packet, opcode, route struct

So do:

`UART Shell -> model_handler -> chat_cli (DSDV) -> Bluetooth Mesh stack -> RF`

---

# Slide 4 - Mo hinh node
## Moi node WSN co gi?

- Bang dinh tuyen DSDV: `dest, next_hop, hop_count, seq_num, age`
- Bo nho RSSI hang xom (EWMA)
- Bo loc duplicate packet (src + seq)
- Vai tro MCDS: `BACKBONE` hoac `LEAF`
- Cac timer: HELLO, UPDATE, backbone evaluation, print routes

---

# Slide 5 - Giao thuc DSDV trong he thong
## Nguyen ly hoat dong

- `HELLO` (TTL=1): phat hien neighbor + cap nhat RSSI + degree/role
- `UPDATE`: quang ba route table theo chu ky, co incremental update
- `DATA`: unicast theo `next_hop` den dich
- `METRICS_ACK`: phan hoi de tinh RTT/latency

Tieu chi cap nhat route:
- Uu tien `seq_num` moi hon
- Neu cung `seq_num`: uu tien hop count nho hon
- Co hysteresis theo RSSI de tranh flapping

---

# Slide 6 - Luong xu ly tu luc khoi dong
## Runtime flow

1. `bt_enable()` -> `bt_mesh_init()`
2. Load settings + enable provisioning
3. Scheduler chay dinh ky:
   - gui HELLO
   - gui UPDATE
   - danh gia backbone role
4. Nhan packet -> parse opcode -> cap nhat route/forward
5. Shell command giup quan sat va test

---

# Slide 7 - Cac goi tin quan trong
## Packet model theo code

- `HELLO`: `src, seq_num, my_degree, my_role`
- `UPDATE`: `src, num_entries, [dest, hop_count, seq_num]`
- `DATA`: `src, dest, seq, hop_count, path_nodes[], metrics`
- `ACK`: `src_addr, original_timestamp`
- `LED_TOGGLE`: dieu khien LED tu xa qua da hop

Y nghia:
- Control-plane: HELLO/UPDATE
- Data-plane: DATA/ACK

---

# Slide 8 - Chinh sach TTL va relay
## Role-adaptive forwarding

- HELLO: TTL co dinh = 1
- UPDATE: TTL cap phu thuoc role
  - BACKBONE: cao hon
  - LEAF: thap hon
- DATA: TTL theo do dai route (`hop + 1`, co gioi han max)

Tac dung:
- Giam flood khong can thiet
- Van du bao phu route cho node trung tam

---

# Slide 9 - MCDS Backbone Selection
## Co che chon node Backbone

He thong danh gia dinh ky dua tren:
- Degree (so neighbor active)
- RSSI trung binh
- PDR uoc luong tu HELLO nhan duoc

Score:
- `score = degree*100 + rssi_norm + pdr_bonus`
- Node cao diem (co tie-break theo dia chi) -> BACKBONE
- Neu khong co backbone lang gieng: force backbone de dam bao ket noi cum

---

# Slide 10 - Shell command de van hanh
## Lenh test truc tiep

- `chat status`
- `chat routes`
- `chat neighbors`
- `chat metrics_to <addr>`
- `chat verify_route <addr>`
- `chat relay <on/off>`
- `chat led_toggle <addr>`
- `chat backbone`

Gia tri:
- Kiem tra route table, do tre, RSSI, role backbone ngay tren board

---

# Slide 11 - Van de he thong khi scale N lon
## Broadcast Storm trong Bluetooth Mesh + DSDV

- Flooding + relay tren nhieu node -> mat do packet RF tang manh
- DSDV them HELLO/UPDATE dinh ky -> control overhead tang
- Hau qua:
  - Collision tang
  - PDR giam
  - RTT tang va dao dong
  - Route cap nhat cham/nhay

---

# Slide 12 - Mo hinh thuc nghiem de chung minh
## Cach test khoa hoc

Giữ co dinh:
- Firmware, TX power, traffic profile

Thay doi:
- So node `N` (10 -> 100)
- Topology (deu, tuyen tinh, cluster)

Do:
- PDR, RTT, avg hop, route changes, control packet count

---

# Slide 13 - Mo phong Python (da tao trong repo)
## Cong cu ho tro

- File: `wsn_dsdv_sim.py`
- Xuat:
  - `batch_summary.csv`
  - `metrics_timeseries.csv`
  - `routing_table_node0.csv`
  - `simulation.gif/mp4`

Diem manh:
- Cho phep danh gia xu huong N lon ngay ca khi khong du board that

---

# Slide 14 - Ket qua can ky vong
## Xu huong tong quat

- N tang -> control overhead tang
- PDR giam dan neu all-relay
- RTT va max RTT tang
- Topology khac nhau cho ket qua khac nhau du cung N

Thong diep:
- Bai toan khong chi do thuat toan route, ma con do flooding + hinh hoc mang

---

# Slide 15 - Ket luan
## Hieu he thong va dinh huong bao cao

- He thong hien tai da co:
  - DSDV routing core
  - Metrics/ACK latency
  - Backbone role selection
- Mo hinh hoa cho thay:
  - Luong control + relay la yeu to quyet dinh khi scale
- Nen bao cao theo 2 phan:
  1) Chung minh hien tuong broadcast storm
  2) Danh gia hieu qua giai phap cai tien

---

# Slide 16 - Phu luc (neu can)
## So do de dua vao PowerPoint

- So do kien truc module
- Sequence diagram: HELLO -> UPDATE -> DATA -> ACK
- Bieu do PDR/RTT theo N
- Heatmap/topology minh hoa collision

