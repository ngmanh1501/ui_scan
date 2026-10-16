# Scan3D Studio

**Local-first web interface for a structured-light 3D scanner**  
**Nền tảng:** Windows 11 và Ubuntu · **Frontend:** React / TypeScript / Vite · **Backend:** Python / FastAPI · **Engine:** C++ / OpenCV

Scan3D Studio cung cấp giao diện web chạy **trên chính máy tính của người dùng** để điều khiển các bước trong hệ thống quét 3D bằng ánh sáng cấu trúc (structured light). Hệ thống dự kiến sử dụng camera **Intel RealSense D435**, máy chiếu **HY300** và có thể kết hợp bộ điều khiển **ESP32**. Kết quả tái tạo được xuất dưới dạng point cloud **`.ply`** để xem trong Scan3D Studio, CloudCompare hoặc MeshLab.

> **Trạng thái dự án:** Giao diện local web, API backend, phát hiện màn hình và thử chiếu ảnh lên HY300 đã được kiểm tra trên Windows 11. Điều này **không đồng nghĩa** toàn bộ quy trình calibration → capture → reconstruction đã được xác nhận hoạt động hoàn chỉnh. Một số thao tác của engine C++ vẫn có thể yêu cầu cửa sổ OpenCV hoặc tương tác trực tiếp.

## 1. Kiến trúc

```text
Browser (localhost)
   │ HTTP / WebSocket
   ▼
Frontend: React + TypeScript + Vite
   │ /api/*
   ▼
Backend: FastAPI (backend/server.py)
   ├── GET /api/health, /api/devices, /api/status
   ├── GET/PUT /api/config
   ├── POST /api/jobs, /api/jobs/cancel
   ├── GET /api/files, /api/files/{path}
   └── WS /api/ws/logs
          │ subprocess (chỉ các lệnh cho phép)
          ▼
C++ engine: scan3d_engine
   ├── Camera / projector calibration
   ├── Structured-light capture / Gray Code
   ├── Triangulation / multi-pose reconstruction
   └── Point cloud (*.ply)
```

Web hoạt động trên `localhost`, **không cần mua tên miền hay thuê hosting**. Đây là giao diện điều khiển phần mềm chạy tại máy, không phải dịch vụ cloud. Không mở backend ra mạng công cộng nếu chưa bổ sung xác thực và kiểm soát truy cập.

## 2. Chức năng

| Chức năng | Ghi chú |
|---|---|
| Dashboard | Trạng thái engine, job, tệp PLY và phần cứng |
| Camera Calibration | Gọi routine tương ứng trong C++ engine |
| Projector / Stereo Calibration | Gọi routine calibration; **không phải** nút chiếu thử độc lập |
| Object Capture | Gọi routine chiếu/capture của engine |
| Single-pose / Multi-pose Reconstruction | Gọi routine tái tạo của engine |
| Turntable Auto Calibration / Reference Model / Locate | Các tác vụ được khai báo trong giao diện |
| 3D Viewer | Mở và xem point cloud `.ply` bằng Three.js |
| Settings | Đọc/sửa `engine/config.json`, có tạo bản sao lưu |
| Logs | Theo dõi trạng thái và output của engine qua WebSocket |
| Display detection | Liệt kê các màn hình, kích thước, tọa độ và màn hình chính qua `screeninfo` |

**Phân biệt:** Phát hiện `Monitor 1` và chọn nó trên giao diện chưa có nghĩa engine C++ đã dùng màn hình đó để chiếu Gray Code. Việc chọn projector cần được nối với cấu hình/API điều khiển và logic chiếu của engine. Không mặc định màn hình phụ luôn là HY300.

## 3. Yêu cầu hệ thống

### Chung

- Git
- Python 3 và môi trường ảo (`venv` hoặc `uv`)
- Node.js và npm (để chạy/build frontend)
- CMake và trình biên dịch C++ hỗ trợ C++17
- OpenCV C++ gồm các module mà engine sử dụng, đặc biệt **`structured_light` từ opencv_contrib**
- Camera/projector chỉ cần kết nối khi thử nghiệm phần cứng; các chức năng thu nhận ảnh có thể cần driver và cấu hình riêng

### Windows 11

- Visual Studio Build Tools với **Desktop development with C++**, MSVC và Windows SDK
- CMake (có thể cài qua Visual Studio hoặc riêng)
- vcpkg (để quản lý OpenCV C++ và dependency)
- Nên dùng **x64 Native Tools Command Prompt** nếu cần công cụ MSVC như `nmake`; PowerShell vẫn dùng được với CMake generator Visual Studio

### Ubuntu

- `build-essential`, `cmake`, `pkg-config`, Python 3, Node.js/npm
- OpenCV development package có **structured_light** (thường cần bản OpenCV có contrib; không phải bản cài nào cũng có)
- Môi trường desktop X11/Wayland khi cần hiển thị projector

## 4. Clone repository

```bash
git clone https://github.com/ngmanh1501/ui_scan.git
cd ui_scan
```

Các lệnh tiếp theo giả định repository có `launcher.py`, `backend/`, `frontend/`, `engine/` như phiên bản đang phát triển. Kiểm tra README và cấu trúc thực tế nếu branch đã thay đổi.

## 5. Cài đặt và chạy trên Windows 11

Mở PowerShell tại thư mục dự án:

```powershell
cd C:\Users\manhn\Documents\ui_scan
```

> Đường dẫn trên là **ví dụ**. Hãy thay bằng vị trí bạn clone repository.

### 5.1. Cài đặt bằng launcher

```powershell
python launcher.py setup
```

Launcher được thiết kế để chuẩn bị các thành phần của ứng dụng. Nếu gặp lỗi OpenCV/CMake hoặc npm, xử lý theo các bước thủ công bên dưới rồi chạy lại khi cần.

### 5.2. Frontend thủ công

```powershell
cd frontend
npm.cmd ci --no-audit --no-fund
npm.cmd run build
cd ..
```

Nếu `npm` bị chặn bởi PowerShell Execution Policy, dùng `npm.cmd` như trên. Không cần đổi Execution Policy toàn hệ thống.

### 5.3. Cài OpenCV C++ qua vcpkg

Nếu repository đã có `vcpkg.json`, chạy vcpkg tại **thư mục gốc repository**:

```powershell
vcpkg install --triplet x64-windows
```

Nếu `vcpkg` chưa có trong `PATH`, dùng **đường dẫn thật** tới `vcpkg.exe`. Ví dụ trên một máy có Visual Studio Build Tools:

```powershell
& "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\vcpkg\vcpkg.exe" install --triplet x64-windows
```

Đường dẫn này không áp dụng cho mọi máy/phiên bản Visual Studio. Kiểm tra bằng `Get-Command vcpkg -ErrorAction SilentlyContinue` hoặc tìm `vcpkg.exe` trong thư mục cài đặt của bạn.

Đảm bảo manifest yêu cầu OpenCV contrib. Ví dụ phần dependency trong `vcpkg.json`:

```json
"dependencies": [
  { "name": "opencv4", "features": ["contrib"] }
]
```

Giữ nguyên các trường khác của manifest (như `name`, `version-string`, `builtin-baseline`) theo repository. Chạy lại `vcpkg install` sau khi sửa manifest.

Kiểm tra:

```powershell
vcpkg list | Select-String "opencv4|protobuf"
Test-Path .\vcpkg_installed\x64-windows\include\opencv2\structured_light.hpp
```

Kết quả `True` ở lệnh cuối cho thấy header cần thiết đã có. **OpenCV C++ từ vcpkg khác với package Python `opencv-python`.**

### 5.4. Build C++ engine

Với cấu hình Visual Studio 2026 đã dùng trong quá trình thử nghiệm:

```powershell
cmake -S engine -B engine/build `
  -G "Visual Studio 18 2026" `
  -A x64 `
  -DCMAKE_PREFIX_PATH="${PWD}/vcpkg_installed/x64-windows"

cmake --build engine/build --config Release
```

Chỉ dùng generator trên nếu Visual Studio tương ứng đã cài; xem danh sách generator bằng `cmake -G`. Khi CMake không tìm thấy dependency, ưu tiên tích hợp **vcpkg toolchain** phù hợp với bản vcpkg đang sử dụng, thay vì hardcode từng đường dẫn `OpenCV_DIR`/`Protobuf_DIR`.

### 5.5. Khởi động

```powershell
python launcher.py start
```

Mở địa chỉ local mà launcher in ra (trong quá trình phát triển thường là `http://127.0.0.1:5173`). Backend thường ở `http://127.0.0.1:8000`. **Không cần chạy `setup` mỗi lần**, nếu dependencies và engine đã sẵn sàng.

## 6. Cài đặt và chạy trên Ubuntu

```bash
sudo apt update
sudo apt install -y git build-essential cmake pkg-config python3 python3-venv nodejs npm

git clone https://github.com/ngmanh1501/ui_scan.git
cd ui_scan
python3 launcher.py setup
python3 launcher.py start
```

**Lưu ý:** Tên gói và phiên bản OpenCV trên Ubuntu khác nhau giữa các bản phát hành. Nếu build báo thiếu `opencv2/structured_light.hpp`, cần cài/build OpenCV với `opencv_contrib` trước khi build engine. Với Wayland, `screeninfo` có thể không liệt kê được màn hình dù desktop vẫn hoạt động; trạng thái display có phương án dự phòng dựa vào biến môi trường `DISPLAY` / `WAYLAND_DISPLAY`.

## 7. Kiểm tra projector HY300

### 7.1. Kết nối màn hình

1. Kết nối HY300 với máy tính (ví dụ HDMI) và bật projector.
2. Trên Windows, nhấn **Win + P → Extend**.
3. Mở **Settings → System → Display → Identify** để xác nhận màn hình nào là HY300.
4. Trong Scan3D Studio, mở **Dashboard → Connected hardware** để xem danh sách màn hình.

Một cấu hình đã kiểm tra trên Windows 11:

```text
Monitor 0: 1920x1080, x=0,    y=0, primary=True
Monitor 1: 1920x1080, x=1920, y=0, primary=False
```

Đây chỉ là **ví dụ**, không phải độ phân giải cố định của HY300 trên mọi máy. `display: true` chỉ chứng minh môi trường hiển thị tồn tại, không xác nhận projector đã được chọn đúng.

### 7.2. Kiểm tra API phát hiện màn hình

```powershell
Invoke-RestMethod http://127.0.0.1:8000/api/devices | ConvertTo-Json -Depth 5
```

Backend sử dụng `screeninfo` để trả về `monitors` với `id`, `width`, `height`, `x`, `y`, `primary`. Đảm bảo `screeninfo` nằm trong `backend/requirements.txt` và được cài vào **đúng môi trường Python mà launcher sử dụng**.

### 7.3. Thử chiếu độc lập bằng Python

Đã kiểm tra thành công việc chiếu ảnh trắng trên màn hình phụ bằng OpenCV Python. Để chạy thử riêng, cài trong virtual environment (không cài vào Python do `uv` quản lý ở cấp hệ thống):

```powershell
uv pip install --python .\.venv\Scripts\python.exe screeninfo numpy opencv-python
```

Nếu repository có file thử nghiệm `test_projector.py` hoặc `test_projector_patterns.py`, chạy:

```powershell
.\.venv\Scripts\python.exe test_projector.py
```

Các file `test_projector*.py` là **công cụ thử nghiệm độc lập** đã sử dụng trong quá trình phát triển; không mặc định chúng được commit vào repository. Không nhìn trực tiếp vào ống kính projector khi chiếu ảnh sáng.

### 7.4. Giới hạn hiện tại

- Dashboard đã có thể liệt kê màn hình từ API.
- Việc chiếu ảnh trắng bằng Python/OpenCV đã được kiểm tra riêng.
- **Chưa xác nhận** web có API `Test Projection`/`Stop Projection` điều khiển projector trực tiếp.
- Nút **Projector / Stereo Calibration** gọi lệnh C++ `projector`, **không phải** bài test hiển thị đơn giản.
- Để hiệu chuẩn Gray Code chính xác cần kiểm tra độ nét, độ phân giải, ánh xạ pixel và đồng bộ camera–projector; đặc biệt tại khoảng cách làm việc dự kiến dưới 0,5 m.

## 8. API chính

| Method | Endpoint | Chức năng |
|---|---|---|
| GET | `/api/health` | Trạng thái backend và file engine |
| GET | `/api/devices` | Thiết bị/màn hình được phát hiện |
| GET | `/api/status` | Job hiện tại |
| POST | `/api/jobs` | Chạy lệnh C++ nằm trong allowlist |
| POST | `/api/jobs/cancel` | Yêu cầu dừng job |
| GET / PUT | `/api/config` | Đọc/ghi `engine/config.json` |
| GET | `/api/files` | Liệt kê `.ply` |
| GET | `/api/files/{file_path}` | Tải `.ply` được phép truy cập |
| WebSocket | `/api/ws/logs` | Theo dõi log/trạng thái |

Ví dụ chạy job:

```json
{ "command": "camera" }
```

Các lệnh được cho phép: `camera`, `projector`, `capture`, `tri`, `tri_multi`, `autocalib`, `build_model`, `locate`, `reload`. Một số lệnh yêu cầu phần cứng, dữ liệu và cấu hình hợp lệ. **Không coi trạng thái “Engine Ready” là bằng chứng calibration/capture đã thành công.**

## 9. Xử lý lỗi thường gặp

| Lỗi | Cách kiểm tra / khắc phục |
|---|---|
| `npm.ps1 cannot be loaded` | Dùng `npm.cmd` trong PowerShell |
| `npm ERR! ECONNRESET` | `npm.cmd ping`; kiểm tra registry/proxy; chạy lại `npm.cmd ci --no-audit --no-fund --maxsockets=3` |
| `vcpkg: command not found` | Chạy trực tiếp `vcpkg.exe` bằng đường dẫn đầy đủ hoặc cấu hình PATH |
| CMake không tìm thấy OpenCV / Protobuf | Kiểm tra `vcpkg_installed`, manifest và CMake prefix/toolchain; xóa cache build khi đổi cấu hình |
| `opencv2/structured_light.hpp: No such file` | Cài OpenCV C++ với tính năng `contrib`, rồi cấu hình/build lại |
| `ModuleNotFoundError: cv2` | Cài `opencv-python` trong đúng Python virtual environment dùng để chạy script test |
| `externally-managed-environment` | Không dùng `--break-system-packages`; cài bằng `uv pip install --python <venv-python> ...` |
| Web hiển thị display nhưng không thấy HY300 | Kiểm tra Windows Extend, `GET /api/devices` và phần frontend đọc `monitors` |
| Cổng 8000 hoặc 5173 đang dùng | Xác định tiến trình chiếm cổng trước khi dừng; không kill bừa các tiến trình khác |
| Camera nodes = Not detected trên Windows | Backend hiện trả `video_nodes: []` trên Windows; kiểm tra camera qua cấu hình/OpenCV riêng |

**Kiểm tra cổng trên Windows:**

```powershell
Get-NetTCPConnection -LocalPort 8000,5173 -State Listen -ErrorAction SilentlyContinue |
  Select-Object LocalPort, OwningProcess
```

## 10. Cấu trúc thư mục tham khảo

```text
ui_scan/
├── backend/
│   ├── server.py             # FastAPI, thiết bị, jobs, files, WebSocket
│   └── requirements.txt
├── frontend/
│   ├── src/
│   │   ├── main.tsx          # React UI
│   │   └── style.css
│   ├── package.json
│   └── package-lock.json
├── engine/
│   ├── src/                  # C++ / OpenCV
│   ├── CMakeLists.txt
│   ├── config.json
│   └── build/                # Kết quả build, không nên commit
├── data/                     # Dữ liệu/PLY cục bộ
├── launcher.py
└── vcpkg.json                # Nếu dùng vcpkg manifest
```

Cấu trúc trên phản ánh các file đã được thảo luận; một số file hoặc thư mục có thể khác theo phiên bản repository.

## 11. Hướng phát triển

- [x] Local web với frontend/backend và launcher
- [x] Dashboard, Logs, 3D PLY Viewer
- [x] Liệt kê màn hình từ backend trên Windows 11
- [x] Kiểm tra chiếu ảnh trắng lên HY300 bằng script Python độc lập
- [ ] API chọn/lưu projector được engine sử dụng thực sự
- [ ] Nút **Test Projection / Stop Projection** trực tiếp trên web
- [ ] Preview Gray Code và điều khiển trình tự chiếu
- [ ] Kiểm tra camera RealSense D435 trên cả Windows và Ubuntu
- [ ] Đồng bộ camera–projector, calibration và capture hoàn toàn qua UI
- [ ] Kiểm thử end-to-end từ capture đến point cloud `.ply`
- [ ] Đóng gói cài đặt thuận tiện cho cả Windows 11 và Ubuntu

## 12. Ghi chú phát triển

- **Không commit** `.venv/`, `node_modules/`, `engine/build/`, `vcpkg_installed/` và dữ liệu scan dung lượng lớn, trừ khi có lý do cụ thể.
- Giữ `package-lock.json` và `vcpkg.json` để dễ tái lập môi trường.
- Các cổng và lệnh launcher có thể thay đổi theo phiên bản; ưu tiên output thực tế trong terminal.
- Khi thay đổi `frontend/src/main.tsx`, chạy `npm.cmd run build` để kiểm tra TypeScript.
- Khi thay đổi `backend/server.py`, khởi động lại backend để áp dụng code mới.
- Khi thay đổi C++ hoặc OpenCV, cấu hình/build lại engine trước khi chạy các job.

---

**Scan3D Studio** — Local structured-light 3D scanning workstation. No cloud required.
