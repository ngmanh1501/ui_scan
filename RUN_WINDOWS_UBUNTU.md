# Scan3D Studio — Windows + Ubuntu

## Yêu cầu cài một lần
- Python 3.11+ với pip/venv; Node.js 22 LTS và npm; Git.
- CMake 3.16+ và C++17 compiler.
- OpenCV **C++ development SDK**, không chỉ `pip install opencv-python`.
- Ubuntu: `sudo apt install build-essential cmake pkg-config libopencv-dev python3-venv` và cài Node.js 22 LTS riêng.
- Windows: Visual Studio 2022 Build Tools (Desktop development with C++), CMake, OpenCV C++ SDK (vcpkg hoặc prebuilt). CMake phải tìm thấy OpenCVConfig.cmake. Nếu dùng vcpkg, cấu hình `CMAKE_TOOLCHAIN_FILE` khi chạy CMake (hoặc qua biến môi trường CMAKE_TOOLCHAIN_FILE); nếu dùng prebuilt, cấu hình `OpenCV_DIR`. Đảm bảo OpenCV DLL có trong PATH khi chạy engine.

## Lệnh dùng chung ở thư mục gốc
```bash
python launcher.py check
python launcher.py setup
python launcher.py start
```
Ubuntu nếu không có `python` thì dùng `python3` với cùng các tham số.
Web: http://127.0.0.1:5173/ ; API: http://127.0.0.1:8000/api/health . Dừng bằng Ctrl+C.

## Giới hạn cần biết
- Setup cần internet; app chạy localhost, không cần tên miền hoặc thuê hosting.
- C++ engine được build riêng cho mỗi hệ điều hành; không mang `.exe` sang Linux hoặc binary Linux sang Windows.
- Các thao tác OpenCV `imshow`/`waitKey` và `std::cin` trong engine vẫn có thể cần cửa sổ/bàn phím trực tiếp, nên web chưa thay thế hoàn toàn UI tương tác của engine.
- Camera, projector, ESP32, driver, OpenCV DLL trên Windows cần kiểm thử trên phần cứng thực tế.
- Hai cổng 8000 và 5173 phải trống; không public API ra Internet.
- File ZIP chứa source, không chứa dependencies, DLL hay executable đã build.
