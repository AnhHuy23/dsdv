# Báo cáo thí nghiệm

## Ảnh hưởng của cơ chế Flooding và Broadcast Storm đến hiệu năng mạng Bluetooth Mesh khi tích hợp giao thức định tuyến DSDV

**Mục đích:** Minh họa (trên cơ sở thí nghiệm có kiểm soát và/hoặc mô phỏng) xu hướng suy giảm hiệu năng khi số node tăng trong điều kiện **tất cả node bật Relay** và **lưu lượng điều khiển DSDV (HELLO/UPDATE) định kỳ**, phù hợp với giả thuyết **nghẽn kênh RF** và **va chạm gói tin**.

**Lưu ý sử dụng:** Các bảng số liệu dưới đây là **dữ liệu minh họa (synthetic)** nhằm mô tả **xu hướng** điển hình trong báo cáo; khi nộp bài chính thức, cần **thay bằng số liệu đo thực tế** từ phòng lab hoặc mô phỏng đã chạy với cùng kịch bản.

---

## 1. Tóm tắt (Abstract)

Bluetooth Mesh mặc định sử dụng cơ chế **Flooding**: gói được relay bởi các node bật Relay, dẫn đến số lần phát lại lớn khi mật độ node cao. Khi tích hợp **DSDV**, các gói **HELLO** và **UPDATE** được gửi định kỳ, làm tăng thêm tải điều khiển trên kênh vật lý. Báo cáo này trình bày phương pháp đo và **kết quả minh họa** cho thấy: khi **N** (số node) tăng trong chế độ **toàn mạng relay**, **tỷ lệ phân phối gói (PDR)** có xu hướng giảm, **độ trễ (RTT)** tăng và dao động; **đối chứng** được thiết kế bằng **cùng N** nhưng **topology triển khai khác nhau** (không phải giải pháp nâng cấp phần mềm), để chứng minh hiệu năng phụ thuộc mạnh vào **hình học mạng** và **mức độ va chạm trên kênh**.

---

## 2. Đặt vấn đề

### 2.1. Flooding và Relay trong Bluetooth Mesh

- Trong mô hình mesh chuẩn, các gói mạng được **phát và relay** theo quy tắc của stack; node có **Relay bật** sẽ tham gia chuyển tiếp, làm tăng số lần truyền trên không gian RF.

### 2.2. Gói điều khiển DSDV

- **HELLO:** phát định kỳ để phát hiện láng giềng và đo RSSI.
- **UPDATE:** lan truyền bảng định tuyến (giới hạn số entry mỗi lần nhưng vẫn lặp theo chu kỳ).

Khi **N** lớn và **mọi node đều relay**, số gói **ADV** trên kênh tăng nhanh; dễ xảy ra **collision**, **retry**, và **độ trễ biến thiên** — hiện tượng thường được gọi là **Broadcast Storm** trong ngữ cảnh flood (không nhất thiết đo được hệ số O(N²) trực tiếp trên sóng, nhưng **xu hướng** quá tải là có thể quan sát).

### 2.3. Giả thuyết thí nghiệm

- **H1:** Với **N** tăng, **PDR** của gói ứng dụng (DATA/metrics) **giảm**, **RTT** **tăng** (khi tất cả relay bật).
- **H2:** Với **cùng N** và **cùng cấu hình relay** (ví dụ tất cả relay ON), **topology triển khai khác nhau** (phân bố node trong không gian / số hop trung bình / mật độ láng giềng) dẫn đến **PDR và RTT khác nhau** — phản ánh mức độ **xung đột trên kênh** và **đặc đường đi** khác nhau, không cần đổi thuật toán.

*Ghi chú:* Các biện pháp **nâng cấp** (ví dụ chọn backbone, tắt relay lá, tinh chỉnh TTL…) là **hướng giải quyết riêng**; **không** dùng làm kịch bản B trong báo cáo thí nghiệm này.

---

## 3. Phương pháp thí nghiệm

### 3.1. Thiết bị và phần mềm

- Nền tảng: **nRF Connect SDK / Zephyr**, ứng dụng **Bluetooth Mesh** với vendor model **DSDV** (HELLO/UPDATE/DATA).
- Giao tiếp: **UART Shell** (`chat metrics_to`, `chat routes`, …).
- (Tuỳ chọn) Mô phỏng Python (`wsn_dsdv_sim.py`) để bổ sung đường cong **N = 10 … 100** khi không đủ số board.

### 3.2. Biến độc lập và cố định

| Yếu tố | Cách xử lý |
|--------|------------|
| **N** | Thay đổi theo bậc: 10, 20, … hoặc 10, 20, …, 100 |
| Công suất phát | Giữ cố định (cùng `prj.conf`) |
| Bố trí | Cùng phòng lab, cùng vị trí tương đối (hoặc cùng seed mô phỏng) |
| Traffic ứng dụng | Cố định: ví dụ **1 gói metrics / giây** từ node A → node B trong **T = 5 phút** |
| Thời gian ổn định trước đo | 2–5 phút sau provision |
| **Topology** | Khi so sánh A vs B: giữ **N**, **TX power**, **firmware**; chỉ đổi **cách bố trí node** (xem mục 3.4) |

### 3.3. Chỉ số đo

| Chỉ số | Định nghĩa ngắn |
|--------|-----------------|
| **PDR** | (Số gói nhận thành công) / (Số gói gửi) × 100% |
| **RTT** | Thời gian vòng (ms) từ shell `[LATENCY]` hoặc timestamp ACK |
| **Độ lệch RTT** | Độ lệch chuẩn hoặc khoảng [min, max] trong cửa sổ đo |
| **Ổn định route** | Số lần thay đổi next-hop hoặc tuổi route (tuỳ khả năng log) |

### 3.4. Hai kịch bản đối chứng (cùng N, khác topology)

Tất cả các run: **Relay ON** trên mọi node (baseline flood), cùng **N**, cùng **traffic đo**.

| Kịch bản | Mô tả topology (ví dụ triển khai lab) |
|----------|--------------------------------------|
| **A – Mật độ đều / vùng rộng** | Node phân bố **ngẫu nhiên đồng đều** trong một **hình vuông** (hoặc lưới) — nhiều láng giềng trong vùng radio, **collision** giữa các cặp hop có xu hướng cao khi N lớn. |
| **B – Tuyến tính (chuỗi)** | Cùng **N** nhưng bố trí **gần thẳng hàng** (corridor / dải dài), khoảng cách giữa các cặp liền kề trong tầm radio — **đường đi** dài hơn, **số hop** end-to-end lớn hơn, mật độ “trùng sóng” cục bộ có thể **khác** so với A (ít neighbor đồng thời trên một đoạn, nhưng tổng hop tăng). |

*Có thể thay B bằng topology khác tương đương mục đích đối chứng:* **cụm (cluster)** — nửa số node tụ một góc, nửa còn lại xa; hoặc **hai cụm nối bằng cầu** — miễn là **mô tả rõ hình học** và **lặp đo 3 lần**.

---

## 4. Kết quả (dữ liệu minh họa — cần thay bằng số đo thực)

*Chú thích: Bảng dưới mô phỏng xu hướng điển hình; không thay thế sổ ghi chép lab.*

### 4.1. Kịch bản A — Tất cả node Relay ON

| N (node) | PDR (%) | RTT trung bình (ms) | RTT max (ms) | Ghi chú |
|----------|---------|---------------------|--------------|---------|
| 10 | 96 | 45 | 120 | Ổn định |
| 30 | 82 | 95 | 380 | Bắt đầu tăng độ lệch |
| 50 | 68 | 160 | 620 | Delay spike |
| 80 | 54 | 240 | 980 | PDR giảm rõ |
| 100 | 41 | 310 | 1250 | Nghẽn rõ trên kênh |

### 4.2. Đối chứng topology — cùng N = 50, cùng all-relay

| Topology | PDR (%) | RTT TB (ms) | RTT max (ms) | Ghi chú minh họa |
|----------|---------|-------------|--------------|------------------|
| **A:** phân bố đều trong vùng vuông | 68 | 160 | 620 | Nhiều node “thấy” nhau → va chạm cục bộ cao |
| **B:** bố trí tuyến (chuỗi) cùng N | 59 | 210 | 890 | Hop trung bình lớn hơn, trễ tích lũy; đặc trưng lỗi khác A |

→ **Chênh lệch** A vs B tại cùng **N** cho thấy: **broadcast storm / nghẽn kênh** không chỉ là hàm của N mà còn của **hình học triển khai** (topology). Đây là cơ sở để **tách** “vấn đề hiện tượng mạng” khỏi **kế hoạch nâng cấp** (xử lý bằng thuật toán/cấu hình khác — trình bày ở phần luận văn riêng).

### 4.3. (Tuỳ chọn) Mô phỏng N lớn

| N | PDR mô phỏng (%) | Gói HELLO+UPDATE (tương đối) |
|---|------------------|-------------------------------|
| 10 | 88 | 1.0× |
| 50 | 62 | ~3.2× |
| 100 | 48 | ~6.1× |

*(Chỉ số cột phải là **tương đối** nếu không có bộ đếm phần cứng; có thể lấy từ log hoặc sniffer.)*

---

## 5. Phân tích và thảo luận

1. **Tải điều khiển:** HELLO/UPDATE định kỳ làm tăng số khung quảng bá trên kênh **2.4 GHz** dùng chung; khi **N** lớn, xác suất **collision** tăng → **PDR** giảm.
2. **Flooding + Relay:** Mỗi node relay tham gia tái phát; với **N** cao, số lần “thấy” cùng một sự kiện trên không trung tăng → **RTT** và **biến thiên** tăng.
3. **DSDV và trễ thông tin:** Bảng định tuyến cập nhật theo chu kỳ; dưới nghẽn, **thứ tự nhận** và **độ tươi** của route có thể dao động — dễ quan sát **route “nhảy”** hoặc **tuổi route** không ổn định (nếu có log).
4. **Topology:** Cùng **N** nhưng **khác cách bố trí** → khác **phân bố va chạm** và **độ dài đường đi**; vì vậy **không** nên chỉ dùng một layout để kết luận cho mọi triển khai thực tế.
5. **Giới hạn:** Kết quả phụ thuộc **bố trí vật lý**, **nhiễu Wi‑Fi**, **phiên bản stack**; nên **lặp 3 lần** và lấy trung bình ± độ lệch chuẩn.

---

## 6. Kết luận

- Trong điều kiện **Bluetooth Mesh + DSDV**, khi **số node tăng** và **tất cả node bật Relay**, hiện tượng **quá tải kênh** và **suy giảm PDR / tăng RTT** là **phù hợp** với mô hình **flooding** và **tải điều khiển định kỳ**.
- **Đối chứng theo topology** (cùng **N**, cùng relay) cho thấy hiệu năng **không cố định** khi chỉ đổi **hình học mạng** — phù hợp với việc phân tích **broadcast storm** trong bối cảnh **triển khai thực tế** (mật độ, hop, vùng phủ sóng).
- **Giải pháp nâng cấp** (relay có chọn lọc, backbone, TTL, v.v.) là **mục riêng**; báo cáo thí nghiệm này **chỉ** nhằm **chứng minh hiện tượng** và **ảnh hưởng của topology**, làm nền cho phần cải tiến trong luận văn.

---

## 7. Hướng dẫn chèn số liệu thật khi hoàn thiện

1. Điền bảng mục 4 bằng **Excel** từ file log serial hoặc `batch_summary.csv` (mô phỏng).
2. Thêm **ảnh chụp màn hình** terminal (PDR, RTT) hoặc **đồ thị** (N trục X, PDR/RTT trục Y).
3. Ghi rõ **ngày**, **phòng lab**, **số board**, **phiên bản firmware**.

---

## 8. Tài liệu tham khảo (gợi ý)

- Bluetooth SIG — *Mesh Profile* (flooding, relay, network PDU).
- Nordic Semiconductor — *nRF Connect SDK* / *Bluetooth Mesh* documentation.
- Tài liệu nội bộ / luận văn — mô tả chi tiết DSDV, topology thí nghiệm, và phần giải pháp nâng cấp (nếu có).

---

*Tệp được tạo để chỉnh sửa và copy sang Microsoft Word. Khuyến nghị đổi tiêu đề, thêm tên tác giả, mã đề tài, và thay toàn bộ bảng số bằng **kết quả đo thực** trước khi nộp.*
