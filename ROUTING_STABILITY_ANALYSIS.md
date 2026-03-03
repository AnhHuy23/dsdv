# Phân tích và Đề xuất Cải thiện Ổn định Bảng Định tuyến DSDV

## 1. Tổng quan Hệ thống Hiện tại

### Cấu hình TX Power
- **Hiện tại**: `-20 dBm` (rất thấp, chỉ phù hợp cho test cự ly ngắn)
- **Vấn đề**: Công suất phát quá thấp gây:
  - Phạm vi phủ sóng hạn chế
  - Dễ bị nhiễu
  - Mất gói tin HELLO/UPDATE thường xuyên
  - Routes timeout không đúng lúc

### RSSI Filtering
- **New route threshold**: -70 dBm
- **Existing route threshold**: -75 dBm (hysteresis)
- **Weak link detection**: -80 dBm
- **EWMA smoothing**: 15/16 factor
- **Vấn đề**: Hysteresis gap chỉ 5 dBm có thể chưa đủ trong môi trường nhiễu

### Timeout Configuration
- **Direct neighbor timeout**: 30 seconds
- **Multi-hop route timeout**: 90 seconds
- **RSSI validity window**: 60 seconds
- **HELLO interval**: ~5 seconds (4-8s với jitter)
- **UPDATE interval**: ~15 seconds (với jitter)

## 2. Các Vấn đề Gây Bảng Định tuyến Không Ổn định

### A. Route Flapping (Dao động tuyến đường)
**Nguyên nhân:**
- RSSI dao động quanh threshold (-70/-75 dBm)
- Hysteresis gap quá nhỏ (5 dBm)
- TX power thấp làm RSSI không ổn định

**Triệu chứng:**
- Routes thay đổi next_hop liên tục
- Bảng định tuyến "nhấp nháy"
- Packet loss cao

### B. Premature Route Expiration (Routes hết hạn sớm)
**Nguyên nhân:**
- TX power -20 dBm → HELLO/UPDATE packets bị mất
- Neighbor không nhận được HELLO trong 30s → route expired
- Cascading invalidation: 1 neighbor timeout → tất cả routes qua neighbor đó invalid

**Triệu chứng:**
- Routes biến mất đột ngột
- Phải học lại routes từ đầu
- Network convergence chậm

### C. Inconsistent Route Selection (Chọn route không nhất quán)
**Nguyên nhân:**
- RSSI measurement không đủ smooth
- Cùng seq_num + cùng hop_count nhưng RSSI chênh lệch < 5 dBm
- Routes được update dựa trên RSSI dao động

## 3. Đề xuất Cải thiện

### ✅ Giải pháp 1: Tăng TX Power (QUAN TRỌNG NHẤT)
```conf
# Thay đổi trong prj.conf
# CONFIG_BT_CTLR_TX_PWR_MINUS_20=y  # Xóa dòng này
CONFIG_BT_CTLR_TX_PWR_0=y           # Thêm dòng này (0 dBm - standard power)
```

**Lợi ích:**
- Tăng phạm vi phủ sóng
- Giảm packet loss
- RSSI ổn định hơn
- Routes ít timeout hơn

**Lưu ý:** Nếu test trong môi trường nhỏ, có thể dùng `-4 dBm` hoặc `-8 dBm`

### ✅ Giải pháp 2: Tăng RSSI Hysteresis Gap
```c
// Trong handle_dsdv_hello() - src/chat_cli.c dòng 336-343
// Hiện tại:
int8_t rssi_threshold = existing_route ? -75 : -70;  // Gap = 5 dBm

// Đề xuất:
int8_t rssi_threshold = existing_route ? -78 : -70;  // Gap = 8 dBm
```

**Lợi ích:**
- Giảm route flapping
- Routes ổn định hơn khi RSSI dao động nhỏ
- Ít thay đổi next_hop không cần thiết

### ✅ Giải pháp 3: Tăng RSSI Smoothing (Tùy chọn)
```c
// Trong rssi_ewma() - src/chat_cli.c dòng 323
// Hiện tại:
int32_t acc = (15 * (int32_t)prev + (int32_t)now) / 16;  // 93.75% old, 6.25% new

// Đề xuất (nếu cần smooth hơn nữa):
int32_t acc = (31 * (int32_t)prev + (int32_t)now) / 32;  // 96.875% old, 3.125% new
```

**Lợi ích:**
- RSSI measurement mượt mà hơn
- Ít bị ảnh hưởng bởi spike/dip tạm thời

**Nhược điểm:**
- Phản ứng chậm hơn với thay đổi thực sự

### ✅ Giải pháp 4: Tăng RSSI Comparison Threshold
```c
// Trong dsdv_upsert() - src/chat_cli.c dòng 379-387
// Hiện tại:
if (new_rssi != -127 && old_rssi != -127 && new_rssi >= old_rssi + 5) {
    // Switch to new route
}

// Đề xuất:
if (new_rssi != -127 && old_rssi != -127 && new_rssi >= old_rssi + 8) {
    // Switch to new route (cần RSSI tốt hơn 8 dBm mới switch)
}
```

**Lợi ích:**
- Chỉ switch route khi có cải thiện đáng kể
- Giảm "ping-pong" giữa 2 routes tương đương

### ⚠️ Giải pháp 5: Điều chỉnh Timeout (Nếu cần)
```c
// Trong src/chat_cli.c dòng 28-30
// Nếu TX power tăng lên 0 dBm, có thể GIẢM timeout để phát hiện lỗi nhanh hơn:
#define DSDV_NEIGHBOR_TIMEOUT_MS  20000  // Giảm từ 30s xuống 20s
#define DSDV_ROUTE_TIMEOUT_MS     60000  // Giảm từ 90s xuống 60s
```

**Chỉ áp dụng khi:**
- TX power đã tăng lên 0 dBm
- HELLO packets ổn định
- Muốn network phản ứng nhanh hơn với node failures

## 4. Thứ tự Ưu tiên Thực hiện

### Bước 1: Tăng TX Power (BẮT BUỘC)
- Thay đổi `prj.conf`: `-20 dBm` → `0 dBm`
- Rebuild và flash lại firmware
- **Đây là thay đổi quan trọng nhất**

### Bước 2: Tăng RSSI Hysteresis (KHUYẾN NGHỊ)
- Thay đổi threshold từ `-75/-70` → `-78/-70`
- Tăng comparison threshold từ `+5 dBm` → `+8 dBm`

### Bước 3: Test và Quan sát
- Chạy network trong 5-10 phút
- Quan sát bảng định tuyến với `chat routes`
- Kiểm tra RSSI với `chat neighbors`
- Xem log để phát hiện route changes

### Bước 4: Fine-tuning (Nếu cần)
- Nếu vẫn flapping: Tăng EWMA smoothing lên 31/32
- Nếu routes timeout: Kiểm tra packet loss
- Nếu convergence chậm: Giảm UPDATE interval jitter

## 5. Monitoring Commands

```bash
# Xem bảng định tuyến
chat routes

# Xem RSSI của neighbors
chat neighbors

# Verify route cụ thể
chat verify_route 0x0002

# Gửi metrics để test
chat metrics_to 0x0002
```

## 6. Dấu hiệu Bảng Định tuyến Ổn định

✅ **Routes không thay đổi next_hop** trong 30-60 giây
✅ **Age của routes tăng dần** (không reset về 0 liên tục)
✅ **RSSI values ổn định** (dao động < 3 dBm)
✅ **Không có log "No valid route"** khi gửi packets
✅ **Metrics delivery thành công** với RTT nhất quán

## 7. Dấu hiệu Vẫn Còn Vấn đề

❌ Routes có cùng dest nhưng next_hop thay đổi liên tục
❌ Age của routes reset về 0-5s thường xuyên
❌ RSSI dao động > 5 dBm trong vài giây
❌ Log có nhiều "Route expired" hoặc "INVALID"
❌ Packet loss > 10%

## 8. Kết luận

**Nguyên nhân chính:** TX Power -20 dBm quá thấp
**Giải pháp chính:** Tăng lên 0 dBm
**Giải pháp bổ sung:** Tăng RSSI hysteresis và comparison threshold

Sau khi áp dụng các thay đổi trên, bảng định tuyến sẽ ổn định hơn đáng kể.
