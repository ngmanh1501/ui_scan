# Scan3D Studio

**Local Web UI cho hệ thống quét 3D Structured Light**, chạy trên **Ubuntu** và **Windows 11** với cùng mã nguồn và cùng bộ lệnh quản lý.

- **Frontend:** React, TypeScript, Vite, Three.js
- **Backend:** Python, FastAPI, Uvicorn
- **Scan engine:** C++17, OpenCV, CMake
- **Địa chỉ local:** http://127.0.0.1:5173/
- **API:** http://127.0.0.1:8000/api/health

> **Lưu ý phiên bản:** README này dành cho source **đã bổ sung `launcher.py`** trong gói `Scan3D_Studio_Windows_Ubuntu.zip`. Nếu repo vẫn ở bản cũ chỉ có `scripts/setup.sh` và `scripts/start.sh`, hãy cập nhật source trước khi thực hiện hướng dẫn. Phần mềm vẫn đang phát triển; khả năng quét với phần cứng thực tế cần kiểm thử riêng trên từng hệ điều hành.

## 1. Cấu trúc dự án

```text
ui_scan/
├── launcher.py                # Setup / check / start dùng chung
├── backend/
│   ├── server.py              # REST API + WebSocket
│   └── requirements.txt
├── frontend/
│   ├── src/
│   ├── package.json
│   └── vite.config.ts
├── engine/
│   ├── src/                   # Mã nguồn C++
│   ├── CMakeLists.txt
│   └── config.json
├── scripts/                   # Script Ubuntu cũ (không bắt buộc)
└── README.md
```

## 2. Yêu cầu hệ thống

| Công cụ | Ubuntu | Windows 11 |
|---|---|---|
| Python | Python 3.11+ (có `venv`, `pip`) | Python 3.11+ (thêm vào PATH) |
| Node.js / npm | Node.js 22 LTS + npm | Node.js 22 LTS + npm |
| CMake | 3.16+ | 3.16+ |
| C++ | GCC/Clang hỗ trợ C++17 | Visual Studio 2022 Build Tools (MSVC, C++ workload) |
| OpenCV | `libopencv-dev` | **OpenCV C++ SDK** qua vcpkg hoặc bản prebuilt |
| Git | Khuyến nghị | Khuyến nghị |

**Quan trọng:** `pip install opencv-python` không thay thế được OpenCV **C++ SDK** để CMake build engine.

## 3. Ubuntu — cài đặt lần đầu

### Bước 1: Cài công cụ

```bash
sudo apt update
sudo apt install -y git build-essential cmake pkg-config libopencv-dev python3 python3-venv python3-pip
```

Cài **Node.js 22 LTS** từ nguồn chính thức hoặc trình quản lý phiên bản Node; không mặc định dùng bản `nodejs` quá cũ trong kho apt.

Kiểm tra:

```bash
python3 --version
node --version
npm --version
cmake --version
pkg-config --modversion opencv4
```

### Bước 2: Lấy source

```bash
git clone https://github.com/ngmanh1501/ui_scan.git
cd ui_scan
```

Nếu sử dụng file ZIP thay vì clone, giải nén và mở terminal tại thư mục chứa `launcher.py`.

### Bước 3: Kiểm tra, cài và build

```bash
python3 launcher.py check
python3 launcher.py setup
```

Lệnh `setup` tạo `.venv`, cài Python dependencies, cấu hình và build engine C++, cài frontend bằng `npm ci`, rồi build frontend.

### Bước 4: Khởi động

```bash
python3 launcher.py start
```

Trình duyệt sẽ mở tại **http://127.0.0.1:5173/**. Nhấn **Ctrl+C** trong terminal để dừng.

## 4. Windows 11 — cài đặt lần đầu

### Bước 1: Cài phần mềm cần thiết

1. **Python 3.11+**: https://www.python.org/downloads/windows/ — bật tùy chọn thêm Python vào `PATH`.
2. **Node.js 22 LTS**: https://nodejs.org/
3. **Git for Windows**: https://git-scm.com/downloads/win
4. **Visual Studio 2022 Build Tools**: https://visualstudio.microsoft.com/downloads/ — chọn workload **Desktop development with C++**, bao gồm MSVC, Windows SDK và CMake.
5. **OpenCV C++ SDK**: khuyến nghị dùng **vcpkg** (https://github.com/microsoft/vcpkg) hoặc bản OpenCV prebuilt tương thích với MSVC. Đảm bảo CMake tìm được `OpenCVConfig.cmake`.

Mở **PowerShell mới** và kiểm tra:

```powershell
python --version
node --version
npm --version
cmake --version
git --version
```

Nếu CMake không tìm thấy compiler, mở **Developer PowerShell for VS 2022** hoặc **x64 Native Tools Command Prompt for VS 2022**.

### Bước 2: Tải mã nguồn

```powershell
cd $HOME\Documents
git clone https://github.com/ngmanh1501/ui_scan.git
cd ui_scan
```

Nếu dùng ZIP, giải nén rồi mở PowerShell tại thư mục chứa `launcher.py`.

### Bước 3: Cấu hình OpenCV cho CMake

Chọn **một** cách:

**Cách A — OpenCV prebuilt:** đặt `OpenCV_DIR` đến thư mục thực tế chứa `OpenCVConfig.cmake`, ví dụ (cần điều chỉnh theo máy):

```powershell
$env:OpenCV_DIR = "C:\opencv\build"
```

**Cách B — vcpkg:** sau khi cài vcpkg và OpenCV tương ứng, đặt toolchain trước khi build:

```powershell
$env:CMAKE_TOOLCHAIN_FILE = "C:\vcpkg\scripts\buildsystems\vcpkg.cmake"
```

Đây chỉ là **đường dẫn ví dụ**. Bạn phải thay bằng đường dẫn đã cài thực tế. Nếu đã cấu hình CMake thất bại trước đó, xóa `engine/build` rồi cấu hình lại. Đối với bản OpenCV DLL, phải bảo đảm các DLL tương thích có thể được Windows tìm thấy khi chạy engine (ví dụ qua `PATH`).

### Bước 4: Kiểm tra, cài và build

```powershell
python launcher.py check
python launcher.py setup
```

### Bước 5: Khởi động

```powershell
python launcher.py start
```

Mở **http://127.0.0.1:5173/**. Nhấn **Ctrl+C** để dừng.

## 5. Lệnh sử dụng hằng ngày (hai hệ điều hành)

| Mục đích | Windows 11 | Ubuntu |
|---|---|---|
| Kiểm tra môi trường | `python launcher.py check` | `python3 launcher.py check` |
| Cài dependencies + build | `python launcher.py setup` | `python3 launcher.py setup` |
| Chạy ứng dụng | `python launcher.py start` | `python3 launcher.py start` |
| Dừng ứng dụng | `Ctrl+C` | `Ctrl+C` |

> Hai hệ điều hành sử dụng **cùng `launcher.py` và cùng các tham số**. Sự khác biệt `python`/`python3` là tên lệnh Python theo cách cài đặt của hệ thống. Chỉ cần `setup` lần đầu hoặc sau khi đổi dependencies/source C++.

## 6. Cấu hình và dữ liệu

- Cấu hình scanner: `engine/config.json`.
- Engine được build riêng theo hệ điều hành:
  - Ubuntu: thường tại `engine/build/scan3d_engine`.
  - Windows (MSVC): thường tại `engine/build/Release/scan3d_engine.exe`.
- Camera index được cấu hình trong `engine/config.json` (`app.camera_url`). Index camera có thể khác nhau giữa Ubuntu và Windows.
- Máy chiếu, camera và ESP32 cần driver, quyền truy cập và cấu hình riêng tùy hệ điều hành.

## 7. Xử lý lỗi thường gặp

**`python` hoặc `python3` không tìm thấy:** kiểm tra Python đã cài và PATH; trên Windows có thể dùng `py` thay `python` nếu Python Launcher khả dụng.

**`npm` không tìm thấy / Vite không chạy:** cài Node.js 22 LTS, mở terminal mới và kiểm tra `node --version`, `npm --version`.

**`Could not find OpenCVConfig.cmake`:** cài OpenCV **C++ SDK**; cấu hình `OpenCV_DIR` hoặc `CMAKE_TOOLCHAIN_FILE` phù hợp. Xóa `engine/build` nếu cần cấu hình lại từ đầu.

**`No CMAKE_CXX_COMPILER could be found`:** Ubuntu cài `build-essential`; Windows cài MSVC Build Tools và chạy trong Developer PowerShell.

**Thiếu DLL khi chạy engine trên Windows:** thêm thư mục DLL OpenCV phù hợp vào `PATH` hoặc triển khai các DLL cần thiết cạnh `.exe`. Kiểm tra kiến trúc x64 và phiên bản compiler.

**Cổng 8000 hoặc 5173 đã sử dụng:** dừng phiên Scan3D trước bằng `Ctrl+C`. Có thể kiểm tra:

Ubuntu:

```bash
ss -ltnp | grep -E ':(8000|5173)\b'
```

Windows PowerShell:

```powershell
Get-NetTCPConnection -LocalPort 8000,5173 -ErrorAction SilentlyContinue
```

Không kết thúc tiến trình khác nếu chưa xác định đúng ứng dụng.

**UI mở nhưng Engine Status = Build required:** chạy lại `launcher.py setup`, kiểm tra lỗi CMake và vị trí binary trong `engine/build`.

**Camera không hoạt động:** kiểm tra driver, quyền camera trên Windows, camera index trong config và việc camera đang bị ứng dụng khác sử dụng.

## 8. Giới hạn và bảo mật

- Ứng dụng là **local web**: không cần thuê hosting hay mua tên miền. Chỉ truy cập trên máy đang chạy, qua `127.0.0.1`.
- Không public backend cổng `8000` ra Internet: API hiện chưa có xác thực người dùng.
- Một số bước trong engine C++ vẫn dùng cửa sổ OpenCV (`imshow`/`waitKey`) và nhập liệu terminal (`std::cin`); do đó **chưa bảo đảm thao tác quét hoàn toàn bằng trình duyệt**.
- Cần kiểm thử riêng trên phần cứng thực tế cho Windows 11 và Ubuntu. README này không khẳng định toàn bộ quy trình calibration/capture đã chạy thành công trên cả hai hệ điều hành.

## 9. Cập nhật source

```bash
git pull
```

Nếu source thay đổi dependencies hoặc C++, chạy lại `launcher.py setup` theo hệ điều hành đang dùng.

---

**Repository:** https://github.com/ngmanh1501/ui_scanA
