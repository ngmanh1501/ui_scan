#pragma once
 
#include <opencv2/opencv.hpp>
#include <opencv2/structured_light.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
 
 
// ============================================================
//  ObjectCaptureConfig
//  Cấu hình cho việc chụp object (tách hoàn toàn khỏi calib)
// ============================================================
struct ObjectCaptureConfig
{
    // --- Projector resolution ---
    int proj_w = 1024;
    int proj_h = 768;
 
    // --- Capture ---
    int warmupFrames   = 5;       // bỏ qua N frame trước khi lấy ảnh
    int patternDelayMs = 250;     // delay sau khi hiện pattern (ms)
 
    // --- Preview window ---
    int previewW = 500;
    int previewH = 375;
    int previewX = 700;
    int previewY = 290;
 
    // --- Root folder chứa các object sessions ---
    std::string rootPath = "Object_data";
};
 
 
// ============================================================
//  Chụp 1 session object:
//  - Chiếu toàn bộ Gray-code patterns
//  - Chụp ảnh tương ứng từ camera
//  - Lưu vào:  rootPath/<session_name>/
//               pattern_0.png … pattern_N.png
//               white.png
//               black.png
//
//  session_name ví dụ: "obj_cup", "obj_box_pose2", ...
//  Trả về true nếu chụp thành công.
// ============================================================
bool captureObjectSession(
    cv::VideoCapture& cap,
    const std::vector<cv::Mat>& patterns,
    const ObjectCaptureConfig& cfg,
    const std::string& object_name,
    int pose_id);
 
 
// ============================================================
//  Vòng lặp tương tác chụp object
//
//  Người dùng nhập tên session trước mỗi lần chụp,
//  có thể chụp nhiều object / nhiều pose khác nhau.
//
//  Keys:
//    n : nhập tên và chụp session mới
//    l : liệt kê các session đã chụp
//    q : thoát
// ============================================================
void runObjectCapture(
    const ObjectCaptureConfig& cfg,
    int                        camera_url);
 







