Dưới đây là tài liệu kỹ thuật chi tiết bằng tiếng Việt cho đoạn mã **Global Trajectory Estimation (GTE)** mà bạn cung cấp. Tài liệu này được cấu trúc theo phong cách chuyên nghiệp giống như file mẫu tiếng Anh bạn đã gửi.

---

# Tài Liệu Kỹ Thuật: Hệ Thống Ước Lượng Quỹ Đạo Toàn Cục (Global Trajectory Estimation - GTE)

**Tác giả**: Dự án V-AIM  
**Ngày**: Tháng 11 năm 2025  
**Phiên bản**: 1.0

---

## Mục lục

1. [Tóm tắt điều hành](#tóm-tắt-điều-hành)
2. [Đặt vấn đề](#đặt-vấn-đề)
3. [Cơ sở lý thuyết](#cơ-sở-lý-thuyết)
4. [Các thành phần thuật toán](#các-thành-phần-thuật-toán)
5. [Quy trình xử lý chi tiết](#quy-trình-xử-lý-chi-tiết)
6. [Ước lượng tư thế Camera](#ước-lượng-tư-thế-camera)
7. [Tham số cấu hình](#tham-số-cấu-hình)
8. [Tài liệu tham khảo](#tài-liệu-tham-khảo)

---

## Tóm tắt điều hành

Tài liệu này trình bày hệ thống Ước lượng Quỹ đạo Toàn cục (GTE), một đường ống xử lý hậu kỳ (post-processing pipeline) được thiết kế để chuẩn hóa dữ liệu chuyển động 3D thô (thường thu được từ quá trình tam giác hóa đa góc nhìn). Hệ thống thực hiện các phép biến đổi hình học tự động để đưa dữ liệu về không gian thế giới (world space) chuẩn: căn chỉnh mặt đất (Z-up), khôi phục tỷ lệ mét thực tế và định hướng quỹ đạo di chuyển. Ngoài ra, hệ thống còn ước lượng lại thông số camera dựa trên mô hình phối cảnh yếu (weak perspective).

**Đóng góp chính:**
*   Tự động căn chỉnh trục thẳng đứng dựa trên phân tích mặt phẳng đất.
*   Khôi phục tỷ lệ mét (metric scale) dựa trên chiều cao nhân trắc học tiêu chuẩn.
*   Căn chỉnh hướng di chuyển chính (Heading alignment) sử dụng PCA.
*   Ước lượng ma trận camera ngược chiều từ dữ liệu 3D và 2D.

---

## Đặt vấn đề

### Đặc điểm dữ liệu đầu vào

Dữ liệu chuyển động 3D thô (`kpts3d`) thu được sau khi tam giác hóa các điểm 2D từ nhiều camera thường gặp các vấn đề sau:

1.  **Hệ tọa độ tùy ý**: Gốc tọa độ và các trục xoay phụ thuộc vào cách hiệu chuẩn camera đầu tiên hoặc thuật toán tam giác hóa, không thẳng hàng với mặt đất thực tế.
2.  **Sai lệch tỷ lệ (Scale ambiguity)**: Dữ liệu thường không có đơn vị mét chuẩn (scale = 1.0 có thể không tương ứng với 1 mét).
3.  **Nhiễu và dữ liệu thiếu (NaNs)**: Các điểm bị che khuất hoặc nhiễu sensor dẫn đến giá trị không xác định.
4.  **Hướng di chuyển ngẫu nhiên**: Chủ thể có thể di chuyển theo bất kỳ hướng nào trong không gian 3D, gây khó khăn cho việc trực quan hóa hoặc huấn luyện mô hình học máy.

### Mục tiêu giải quyết

Hệ thống GTE cần biến đổi dữ liệu sao cho:
*   Mặt đất nằm trên mặt phẳng XY ($z \approx 0$).
*   Trục Z hướng lên trên (Z-up).
*   Đơn vị đo lường tương ứng với mét thực tế.
*   Hướng di chuyển chính của chủ thể trùng với trục X dương.

---

## Cơ sở lý thuyết

### 3.1. Ước lượng mặt phẳng (Plane Fitting)

Để tìm mặt đất, chúng ta giả định rằng các điểm thấp nhất của cơ thể trong suốt quá trình chuyển động sẽ nằm trên mặt đất. Bài toán tìm vector pháp tuyến $\mathbf{n}$ của mặt phẳng đất được giải quyết bằng phương pháp Phân rã giá trị suy biến (SVD) trên tập hợp các điểm ứng cử viên:

$$ \mathbf{P} - \bar{\mathbf{P}} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^T $$

Vector pháp tuyến $\mathbf{n}$ tương ứng với hàng cuối cùng của $\mathbf{V}^T$ (thành phần chính nhỏ nhất).

### 3.2. Ma trận quay từ Vectors

Để xoay không gian sao cho pháp tuyến mặt đất $\mathbf{n}$ trùng với trục Z $\mathbf{z} = [0, 0, 1]$, ta sử dụng công thức Rodrigues hoặc phương pháp trực tiếp để tạo ma trận quay $\mathbf{R}$ sao cho:

$$ \mathbf{R} \cdot \mathbf{n} = \mathbf{z} $$

### 3.3. Phân tích thành phần chính (PCA) cho hướng di chuyển

Để căn chỉnh hướng di chuyển (Yaw alignment), ta áp dụng PCA lên quỹ đạo di chuyển của trọng tâm cơ thể trên mặt phẳng XY. Vector riêng (eigenvector) ứng với trị riêng lớn nhất sẽ chỉ ra hướng di chuyển chính.

---

## Các thành phần thuật toán

Hệ thống bao gồm các module tiện ích cốt lõi sau:

### 4.1. `fit_plane_svd(P)`
*   **Chức năng**: Tìm vector pháp tuyến và độ lệch của mặt phẳng khớp nhất với tập điểm $P$.
*   **Đầu vào**: Tập hợp các điểm 3D $(N, 3)$.
*   **Phương pháp**: Sử dụng `np.linalg.svd`.
*   **Đầu ra**: Vector pháp tuyến đơn vị $\mathbf{n}$ và khoảng cách $d$.

### 4.2. `rotation_matrix_from_vectors(a, b)`
*   **Chức năng**: Tính ma trận quay $3 \times 3$ để xoay vector $\mathbf{a}$ trùng với vector $\mathbf{b}$.
*   **Xử lý đặc biệt**: Xử lý các trường hợp vector song song hoặc đối song song (góc 180 độ) để tránh lỗi chia cho 0.

### 4.3. `robust_person_height_z(k3)`
*   **Chức năng**: Ước lượng chiều cao thống kê của chủ thể để phục vụ việc khôi phục tỷ lệ.
*   **Thuật toán**: Tính khoảng cách giữa phân vị thứ 95 (đầu) và phân vị thứ 5 (chân) của trục Z trên mỗi khung hình, sau đó lấy trung vị (median) của toàn bộ chuỗi thời gian để loại bỏ nhiễu (outliers).

### 4.4. `estimate_weak_perspective_camera(X, U)`
*   **Chức năng**: Ước lượng camera model ánh xạ điểm 3D ($X$) sang 2D ($U$).
*   **Mô hình**: Weak Perspective (Phối cảnh yếu - Giả định độ sâu của đối tượng nhỏ so với khoảng cách đến camera).
*   **Công thức**: Giải bài toán bình phương tối thiểu tuyến tính (Linear Least Squares) để tìm ma trận chiếu $\mathbf{M}$, sau đó dùng SVD để tách thành phần quay $\mathbf{R}$ và tỷ lệ $s$.

---

## Quy trình xử lý chi tiết

Hệ thống GTE thực hiện tuần tự 4 bước chính:

### Bước 1: Căn chỉnh trục Z (World Z-up Alignment)

1.  **Lọc dữ liệu**: Loại bỏ các điểm `NaN`.
2.  **Tìm điểm mặt đất**: Lấy mẫu các điểm có giá trị Z thấp nhất (top 5% hoặc 10%) từ toàn bộ dữ liệu.
3.  **Khớp mặt phẳng**: Sử dụng `fit_plane_svd` để tìm pháp tuyến mặt đất hiện tại.
4.  **Xoay trục**: Tính ma trận quay $\mathbf{R}_{zup}$ để đưa pháp tuyến mặt đất về trục $[0, 0, 1]$.
5.  **Dịch chuyển**: Dịch chuyển toàn bộ hệ tọa độ sao cho mặt đất nằm tại $z=0$.

```python
# Pseudo-code logic
n, d = fit_plane_svd(ground_candidates)
Rzup = rotation_matrix_from_vectors(n, [0, 0, 1])
X_rot = apply_rtscale(X_raw, R=Rzup)
```

### Bước 2: Khôi phục tỷ lệ mét (Metric Scale Recovery)

1.  **Tính chiều cao**: Sử dụng `robust_person_height_z` để đo chiều cao hiện tại của chủ thể trong đơn vị thô ($H_{est}$).
2.  **Hệ số tỷ lệ**: So sánh với chiều cao mục tiêu (mặc định 1.70m).
    $$ s_{metric} = \frac{\text{TARGET\_HEIGHT\_M}}{H_{est}} $$
3.  **Giới hạn**: Hệ số tỷ lệ được kẹp trong khoảng $[0.25, 4.0]$ để tránh các giá trị biến dạng quá mức.

### Bước 3: Căn chỉnh hệ tọa độ thế giới (Heading/Yaw Alignment)

1.  **Tính quỹ đạo**: Tính trung bình vị trí các khớp (centroid) theo thời gian, nội suy các giá trị thiếu.
2.  **PCA trên XY**: Tìm vector hướng chính $\mathbf{v}_{xy}$ của quỹ đạo trên mặt phẳng 2D.
3.  **Xoay hướng**: Tính ma trận quay $\mathbf{R}_{yaw}$ để đưa $\mathbf{v}_{xy}$ trùng với trục $+X$.
4.  **Dịch chuyển gốc**: Đưa vị trí centroid tại frame đầu tiên về gốc tọa độ $(0,0,0)$ trên mặt phẳng XY.
5.  **Xoay bổ sung**: Xoay thêm $-90^\circ$ (nếu cần thiết theo quy ước) và áp dụng phản chiếu (Mirroring) nếu được cấu hình.

### Bước 4: Ước lượng Camera

Nếu có dữ liệu 2D (`kpts2d`), hệ thống sẽ ước lượng vị trí tương đối của từng camera so với không gian thế giới mới được chuẩn hóa.

1.  **Đồng bộ dữ liệu**: Ghép cặp các điểm 3D và 2D hợp lệ (không phải NaN).
2.  **Giải phương trình**: Tìm ma trận quay $\mathbf{R}$, dịch chuyển $\mathbf{t}$, và tỷ lệ $s$ cho từng camera.

---

## Kết quả đầu ra

Sau khi xử lý, hệ thống trả về:

1.  **`kpts3d_filtered` (hay `X_world`)**: Dữ liệu chuyển động 3D đã được chuẩn hóa (Z-up, Metric Scale, Heading Aligned).
2.  **`camera_models`**: Danh sách các tham số camera (Rotation, Translation, Scale) cho phép chiếu điểm 3D ngược lại hình ảnh 2D.
3.  **`s_metric`**: Hệ số tỷ lệ đã áp dụng.
4.  **`GTE_summary`**: Dictionary chứa các ma trận biến đổi ($\mathbf{R}_{zup}, \mathbf{R}_{yaw}, \mathbf{t}_{ground}, \dots$) để phục vụ việc debug hoặc đảo ngược quá trình.

---

## Phụ lục A: Tham số mặc định

| Tham số | Giá trị | Mô tả |
|---------|---------|-------|
| `TARGET_HEIGHT_M` | 1.70 | Chiều cao giả định của chủ thể (mét) |
| `min_points` (Camera Est) | 50 | Số điểm tối thiểu để ước lượng camera |
| `Ground Percentile` | 5.0 | Phần trăm điểm thấp nhất được coi là mặt đất |
| `APPLY_MIRROR_X` | False | Lật dữ liệu qua trục X |
| `APPLY_MIRROR_Y` | False | Lật dữ liệu qua trục Y |

---

## Tài liệu tham khảo

1.  **Horn, B. K. P.** (1987). "Closed-form solution of absolute orientation using unit quaternions". *JOSA A*. (Cơ sở cho việc tìm ma trận quay tối ưu).
2.  **Golub, G. H., & Reinsch, C.** (1970). "Singular value decomposition and least squares solutions". *Numerische Mathematik*. (Cơ sở cho SVD và PCA).
3.  **Forsyth, D. A., & Ponce, J.** (2002). *Computer Vision: A Modern Approach*. (Lý thuyết về Weak Perspective Projection).

---
**Phiên bản tài liệu**: 1.0
**Cập nhật lần cuối**: 27/11/2025