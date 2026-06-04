## 4.5 Kết quả PDR

### Mục tiêu

- Kiểm tra **cải tiến** có làm **mất gói nhiều hơn** Baseline hay không; nếu PDR **giảm nhẹ** thì **ghi rõ** (hop, đường đi, leaf hạn chế relay).

### Ghi nhận tự động (PDR giảm nhẹ)

- **S1**: Full 85.12% < Baseline 93.37% — đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.
- **S2**: Full 75.53% < Baseline 88.11% — đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.
- **S3**: Full 71.25% < Baseline 82.36% — avg hop cao hơn Baseline (1.17 vs 1.05); đường chuyển tiếp dài hơn / backbone-forward-only; leaf hạn chế relay đa chặng.

| Kịch bản | Baseline | B–Leaf | B–Leaf+Grad |
|---|---:|---:|---:|
| S1 | 93.37 | 91.67 | 85.12 |
| S2 | 88.11 | 91.27 | 75.53 |
| S3 | 82.36 | 81.05 | 71.25 |
| S4 | 46.53 | 98.23 | 86.48 |
| S5 | 44.93 | 94.43 | 86.94 |
| S6 | 64.92 | 94.59 | 92.98 |
| S7 | 47.55 | 95.81 | 95.81 |
| S8 | 68.49 | 99.54 | 96.89 |
| S9 | 40.51 | 97.85 | 97.75 |
| S10 | 63.63 | 92.05 | 89.11 |
| S11 | 50.62 | 100.00 | 100.00 |
| S12 | 83.48 | 97.22 | 94.49 |