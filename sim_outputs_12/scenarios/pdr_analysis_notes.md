## 4.5 Kết quả PDR

### Mục tiêu

- Kiểm tra **cải tiến** có làm **mất gói nhiều hơn** Baseline hay không; nếu PDR **giảm nhẹ** thì **ghi rõ** (hop, đường đi, leaf hạn chế relay).

### Ghi nhận tự động (PDR giảm nhẹ)

- **S1**: Full 90.67% < Baseline 93.37% — đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.
- **S2**: Full 77.14% < Baseline 88.11% — đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.
- **S3**: Full 77.76% < Baseline 82.36% — avg hop cao hơn Baseline (1.16 vs 1.05); đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.

| Kịch bản | Baseline | B–Leaf | B–Leaf+Grad |
|---|---:|---:|---:|
| S1 | 93.37 | 95.95 | 90.67 |
| S2 | 88.11 | 83.05 | 77.14 |
| S3 | 82.36 | 87.69 | 77.76 |
| S4 | 46.53 | 98.54 | 89.71 |
| S5 | 44.93 | 96.00 | 95.86 |
| S6 | 64.92 | 97.79 | 97.05 |
| S7 | 47.55 | 99.54 | 98.97 |
| S8 | 68.49 | 97.45 | 99.16 |
| S9 | 40.51 | 98.17 | 99.51 |
| S10 | 63.63 | 94.29 | 92.95 |
| S11 | 50.62 | 97.96 | 97.96 |
| S12 | 83.48 | 98.44 | 94.82 |