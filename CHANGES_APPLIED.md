# Các Thay đổi Đã Áp dụng để Cải thiện Ổn định Bảng Định tuyến

## Tóm tắt
Đã áp dụng 4 thay đổi chính để cải thiện độ ổn định của bảng định tuyến DSDV.

## 1. Tăng TX Power (prj.conf)
**File:** `prj.conf` dòng 74

**Thay đổi:**
```diff
- CONFIG_BT_CTLR_TX_PWR_MINUS_20=y
+ # CONFIG_BT_CTLR_TX_PWR_MINUS_20=y
+ CONFIG_BT_CTLR_TX_PWR_0=y
```

**Lý do:**
- TX power -20 dBm quá thấp, gây packet loss cao
- Tăng lên 0 dBm (standard power) để:
  - Tăng phạm vi phủ sóng
  - Giảm packet loss
  - RSSI ổn định hơn
  - Routes ít bị timeout

**Tác động:** ⭐⭐⭐⭐⭐ (Quan trọng nhất)

---

## 2. Tăng RSSI Hysteresis Gap (src/chat_cli.c)
**File:** `src/chat_cli.c` dòng ~336-343 (trong `handle_dsdv_hello()`)

**Thay đổi:**
```diff
- int8_t rssi_threshold = existing_route ? -75 : -70;  // Gap = 5 dBm
+ int8_t rssi_threshold = existing_route ? -78 : -70;  // Gap = 8 dBm
```

**Lý do:**
- Hysteresis gap 5 dBm quá nhỏ, dễ gây route flapping
- Tăng lên 8 dBm để:
  - Routes ổn định hơn khi RSSI dao động nhỏ
  - Giảm việc thay đổi next_hop không cần thiết
  - Existing routes khó bị thay thế hơn

**Tác động:** ⭐⭐⭐⭐ (Rất quan trọng)

---

## 3. Tăng RSSI Smoothing Factor (src/chat_cli.c)
**File:** `src/chat_cli.c` dòng ~323 (trong `rssi_ewma()`)

**Thay đổi:**
```diff
- int32_t acc = (15 * (int32_t)prev + (int32_t)now) / 16;  // 93.75% old, 6.25% new
+ int32_t acc = (31 * (int32_t)prev + (int32_t)now) / 32;  // 96.875% old, 3.125% new
```

**Lý do:**
- EWMA 15/16 vẫn còn nhạy với noise
- Tăng lên 31/32 để:
  - RSSI measurement mượt mà hơn
  - Ít bị ảnh hưởng bởi spike/dip tạm thời
  - Giảm dao động RSSI ngắn hạn

**Tác động:** ⭐⭐⭐ (Quan trọng)

---

## 4. Tăng RSSI Comparison Threshold (src/chat_cli.c)
**File:** `src/chat_cli.c` dòng ~379-387 (trong `dsdv_upsert()`)

**Thay đổi:**
```diff
- if (new_rssi != -127 && old_rssi != -127 && new_rssi >= old_rssi + 5) {
+ if (new_rssi != -127 && old_rssi != -127 && new_rssi >= old_rssi + 8) {
```

**Lý do:**
- Threshold +5 dBm quá nhỏ, dễ "ping-pong" giữa 2 routes
- Tăng lên +8 dBm để:
  - Chỉ switch route khi có cải thiện đáng kể
  - Giảm route oscillation
  - Routes ổn định hơn

**Tác động:** ⭐⭐⭐ (Quan trọng)

---

## Kết quả Mong đợi

### Trước khi thay đổi:
- ❌ Routes thay đổi next_hop liên tục (flapping)
- ❌ RSSI dao động lớn (> 5 dBm)
- ❌ Routes timeout sớm do packet loss
- ❌ Bảng định tuyến không ổn định

### Sau khi thay đổi:
- ✅ Routes ổn định, next_hop không đổi trong 30-60s
- ✅ RSSI dao động nhỏ (< 3 dBm)
- ✅ Routes ít timeout hơn
- ✅ Bảng định tuyến ổn định

---

## Hướng dẫn Build và Test

### 1. Build lại firmware
```bash
# Clean build
west build -b nrf52dk_nrf52832 -p

# Hoặc rebuild
west build
```

### 2. Flash vào board
```bash
west flash
```

### 3. Kiểm tra kết quả
```bash
# Xem bảng định tuyến
chat routes

# Xem RSSI neighbors
chat neighbors

# Test route stability
chat verify_route 0x0002
```

### 4. Quan sát
- **Age của routes** phải tăng dần (không reset về 0)
- **Next_hop** không thay đổi trong 30-60 giây
- **RSSI values** ổn định (dao động < 3 dBm)
- **Không có log "INVALID"** hoặc "Route expired" thường xuyên

---

## Troubleshooting

### Nếu vẫn còn flapping:
1. Kiểm tra TX power đã thay đổi chưa: `CONFIG_BT_CTLR_TX_PWR_0=y`
2. Tăng EWMA lên 63/64 nếu cần smooth hơn nữa
3. Tăng hysteresis gap lên 10 dBm (-80/-70)

### Nếu routes timeout:
1. Kiểm tra packet loss với `chat neighbors`
2. Tăng timeout nếu cần:
   - `DSDV_NEIGHBOR_TIMEOUT_MS` từ 30s lên 40s
   - `DSDV_ROUTE_TIMEOUT_MS` từ 90s lên 120s

### Nếu convergence chậm:
1. Giảm UPDATE interval jitter
2. Giảm EWMA xuống 15/16 (trade-off: ít smooth hơn)

---

## Tài liệu Tham khảo
- `ROUTING_STABILITY_ANALYSIS.md` - Phân tích chi tiết
- `src/chat_cli.c` - Implementation
- `prj.conf` - Configuration

---

**Ngày áp dụng:** 2026-02-06
**Trạng thái:** ✅ Đã áp dụng và kiểm tra syntax
**Cần test:** Trên hardware thực tế
