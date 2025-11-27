# BÁO CÁO KỸ THUẬT: TÍCH HỢP BỘ LỌC THÍCH NGHI ĐA TẦNG (MULTI-STAGE ADAPTIVE FILTER) VÀO GLOBAL TRAJECTORY ESTIAMTION (GTE)

## 1. Đặt vấn đề và Phạm vi nghiên cứu

### 1.1. Bối cảnh
Trong quy trình xử lý chuyển động 3D (3D Motion Capture) từ video, dữ liệu đầu vào thường đi qua các bước tam giác hóa (triangulation) từ không gian 2D sang 3D. Mặc dù quy trình **Global Trajectory Estimation (GTE)** đã giải quyết tốt các vấn đề về hình học không gian (căn chỉnh trục Z, khôi phục tỉ lệ mét, định hướng trục X), dữ liệu `kpts3d_world` (hay `X_world`) vẫn tồn tại các nhiễu tín hiệu theo thời gian (Temporal Artifacts):
1.  **Jitter (Rung nhiễu tần số cao):** Các khớp xương dao động quanh vị trí thực dù chủ thể đứng yên.
2.  **Lag (Độ trễ):** Khi áp dụng lọc mượt quá mức, chuyển động nhanh bị trễ so với thực tế.
3.  **Missing Data/Outliers:** Các điểm dữ liệu bị mất hoặc nhảy vọt do che khuất (occlusion).

### 1.2. Mục tiêu
Xây dựng một hệ thống **Adaptive Filter (Bộ lọc thích nghi)** thông minh, không sử dụng một tham số cố định cho toàn bộ chuỗi, mà tự động nhận biết trạng thái chuyển động (Đứng yên / Chậm / Nhanh / Rung) để điều chỉnh mức độ lọc.

---

## 2. Cơ sở lý thuyết và Kiến trúc bộ lọc

Hệ thống đề xuất sử dụng kiến trúc **Hybird Filter** kết hợp 3 thành phần cốt lõi chạy song song cho mỗi bậc tự do (Degree of Freedom) của từng khớp xương.

### 2.1. Thành phần 1: Kalman Filter (Dự đoán và Bù đắp)
*   **Nguyên lý:** Sử dụng mô hình *Constant Velocity* (Vận tốc không đổi).
*   **Trạng thái ($x$):** Bao gồm vị trí ($p$) và vận tốc ($v$).
*   **Vai trò trong hệ thống:** Kalman Filter chịu trách nhiệm chính trong việc xử lý **dữ liệu bị mất** hoặc độ tin cậy thấp. Nhờ mô hình vật lý, nó có thể "đoán" vị trí tiếp theo ngay cả khi sensor không ghi nhận được.
*   **Tham số chính:**
    *   `Measurement Noise` ($R$): Độ tin cậy vào dữ liệu đo đạc (Input).
    *   `Process Noise` ($Q$): Độ tin cậy vào mô hình dự đoán.

### 2.2. Thành phần 2: One Euro Filter (Thích ứng vận tốc)
*   **Nguyên lý:** Là bộ lọc thông thấp (Low-pass filter) bậc một, nhưng tần số cắt ($f_c$) thay đổi tuyến tính theo vận tốc chuyển động.
    $$f_c = f_{min} + \beta \cdot |\dot{x}|$$
*   **Vai trò trong hệ thống:** Đây là bộ lọc chính cho các **chuyển động tự nhiên của con người**.
    *   Khi chuyển động chậm: $f_c \approx f_{min}$ (Lọc mạnh, khử rung tốt).
    *   Khi chuyển động nhanh: $f_c$ tăng lên (Lọc ít, giảm độ trễ tối đa).
*   **Tham số chính:** `min_cutoff` (1.0 Hz) và `beta` (0.007).

### 2.3. Thành phần 3: Exponential Moving Average - EMA (Ổn định tĩnh)
*   **Nguyên lý:** Trung bình động lũy thừa.
    $$S_t = \alpha \cdot Y_t + (1 - \alpha) \cdot S_{t-1}$$
*   **Vai trò trong hệ thống:** Chuyên biệt cho trạng thái **Static (Đứng yên)**. EMA có chi phí tính toán cực thấp và khả năng "ghim" chặt vị trí (anchor) tốt hơn Kalman hay 1Euro khi chủ thể pose tĩnh.
*   **Tham số chính:** `alpha` (0.3).

---

## 3. Thuật toán pha trộn trọng số (Dynamic Weight Blending)

Đây là "bộ não" của hệ thống. Thay vì chọn 1 trong 3, ta tính toán vị trí cuối cùng ($P_{final}$) bằng tổng trọng số:

$$P_{final} = w_k \cdot P_{Kalman} + w_o \cdot P_{OneEuro} + w_e \cdot P_{EMA}$$

Trong đó $\sum w = 1$. Trọng số được quyết định bởi **Máy trạng thái (State Machine)** dựa trên vận tốc tức thời ($v$):

| Trạng thái | Điều kiện ($v$ m/s) | $w_k$ (Kalman) | $w_o$ (1Euro) | $w_e$ (EMA) | Lý giải kỹ thuật |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **STATIC (Tĩnh)** | $v < 0.1$ | 0.20 | 0.20 | **0.60** | Khi đứng yên, nhiễu jitter rất dễ thấy. EMA với alpha thấp sẽ loại bỏ hoàn toàn dao động, giữ tư thế vững. |
| **SLOW (Chậm)** | $0.1 \le v < 1.0$ | 0.25 | **0.55** | 0.20 | Chuyển động chậm (ví dụ: vươn vai). One Euro hoạt động tốt nhất ở đây để cân bằng giữa mượt mà và độ trễ. |
| **FAST (Nhanh)** | $v \ge 1.0$ | **0.50** | 0.40 | 0.10 | Chuyển động nhanh (ví dụ: đấm, đá). Cần Kalman để dự đoán quỹ đạo quán tính và giảm độ trễ của EMA. |
| **UNCERTAIN** | Confidence $\approx 0$ | **0.90** | 0.10 | 0.00 | Khi mất tín hiệu, tin tưởng tuyệt đối vào mô hình dự đoán của Kalman. |


## 4. Phân tích hiệu quả và Tinh chỉnh

### 4.1. So sánh kết quả
*   **Raw Data:** Đường đi của khớp (Trajectory) có dạng răng cưa nhỏ (jitter).
*   **Simple Smoothing (EMA only):** Đường đi mượt nhưng bị trễ (lag) ở các đoạn cua gắt hoặc đổi hướng nhanh.
*   **Adaptive Filter:**
    *   Đoạn đứng yên: Đường đi phẳng lì (do EMA chiếm ưu thế).
    *   Đoạn vung tay nhanh: Đường đi bám sát tín hiệu gốc, không bị trễ (do Kalman/OneEuro chiếm ưu thế).

### 4.2. Hướng dẫn tinh chỉnh (Tuning Guide)
Nếu kết quả chưa ưng ý, hãy điều chỉnh các tham số trong `__init__` của `AdaptiveFilterNode`:

1.  **Nếu vẫn còn rung lắc khi đứng yên:**
    *   Giảm `kalman_process_noise` (ví dụ: `1e-3`).
    *   Tăng trọng số EMA cho trường hợp Static (ví dụ: `[0.1, 0.1, 0.8]`).

2.  **Nếu bị trễ (lag) khi di chuyển nhanh:**
    *   Tăng `one_euro_beta` (ví dụ: `0.01` hoặc `0.1`).
    *   Tăng trọng số Kalman cho trường hợp Fast.

3.  **Nếu dự đoán bị vọt lố (Overshoot) sau khi mất tín hiệu:**
    *   Tăng `kalman_measurement_noise` để bộ lọc ít tin vào dự đoán hơn khi có tín hiệu trở lại.

## 5. Kết luận
Việc tích hợp **Adaptive Filter** vào quy trình GTE giúp nâng cao chất lượng dữ liệu đầu ra lên mức "Production-ready". Hệ thống không chỉ giải quyết bài toán hình học (như GTE gốc) mà còn giải quyết triệt để bài toán động học thời gian, đảm bảo tính ổn định và chân thực cho chuyển động 3D.
