#pragma once
 
#include <opencv2/opencv.hpp>
#include <opencv2/structured_light.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
 
// ============================================================
//  ProjectorCalibConfig
//  Tất cả các hằng số liên quan đến calib projector
// ============================================================
struct ProjectorCalibConfig
{
    // --- Projector resolution (không gian Gray-code gốc) ---
    int proj_w = 1024;
    int proj_h = 768;
 
    // --- Preview window (không gian hiển thị thực tế) ---
    // Phải giữ đúng tỉ lệ proj_w:proj_h để scale đồng đều
    // Ví dụ: 500x375 cho proj 1024x768 (tỉ lệ 4:3)
    int previewW = 1024;
    int previewH = 768;
    int previewX = 0;
    int previewY = 0;
 
    // --- Scale factor (tự tính từ preview / proj resolution) ---
    // scale_x = previewW / proj_w
    // scale_y = previewH / proj_h
    // Dùng trong computeProjectorCalib() VÀ trong triangulator để remap
    // proj coords từ hệ Gray-code [0..proj_w, 0..proj_h] sang hệ preview
    // [0..previewW, 0..previewH] — đây là không gian pixel mà projector
    // THỰC SỰ chiếu sáng (vùng preview trên desktop), nên Kp phải được
    // calib và dùng nhất quán trong không gian này.
    double scaleX() const { return (double)previewW / (double)proj_w; }
    double scaleY() const { return (double)previewH / (double)proj_h; }
 
    // --- Checkerboard ---
    cv::Size boardSize  = {9, 7};
    float    squareSize = 23.0f;   // mm
 
    // --- Decode ---
    int threshold   = 35;           // ngưỡng sáng/tối để nhận bit
    int patchRadius = 2;           // bán kính patch decode quanh corner
    int minDecodePoints = 15;      // số điểm tối thiểu để chấp nhận 1 pose

    // --- Số pose tối thiểu để computeProjectorCalib() chịu tính calib ---
    // (trước đây là const int MIN_POSES = 5; hard-code trong .cpp)
    int minPoses = 5;
 
    // --- Projector intrinsic guess ---
    // Tự tính từ previewW để phù hợp không gian calib thực tế.
    // Projector throw ratio thường cho FOV rộng hơn camera thông thường,
    // nên hệ số < 1.0 hợp lý hơn làm điểm khởi đầu cho solver
    // (0.6–1.0 tuỳ throw ratio thực tế của máy chiếu).
    double focalGuess() const { return previewW * 0.8; }
 
    // --- Capture ---
    int warmupFrames   = 5;        // bỏ qua N frame trước khi lấy ảnh
    int patternDelayMs = 300;      // delay sau khi hiện pattern
 
    // --- Output file ---
    std::string outputJson = "projector_calibration.json";
};
 
 
// ============================================================
//  PatternSet
//  Gói gọn tập ảnh chụp 1 pose (patterns + white + black)
// ============================================================
struct PatternSet
{
    std::vector<cv::Mat> images;   // [0..N-1] = Gray-code patterns
                                   // [N-2]    = white
                                   // [N-1]    = black
};
 
 
// ============================================================
//  Hàm decode 1 pose (ảnh → điểm camera ↔ điểm projector)
//
//  Trả về true nếu tìm đủ điểm hợp lệ.
//  Tọa độ proj_valid trả ra vẫn là hệ Gray-code gốc [0..proj_w, 0..proj_h].
//  Việc remap về không gian preview được thực hiện trong computeProjectorCalib().
// ============================================================
bool decodePose(
    const PatternSet&            ps,
    const ProjectorCalibConfig&  cfg,
    std::vector<cv::Point3f>&    obj_valid,
    std::vector<cv::Point2f>&    cam_valid,
    std::vector<cv::Point2f>&    proj_valid);
 
 
// ============================================================
//  Hàm chụp 1 pose từ camera + projector
// ============================================================
bool captureCalibPose(
    cv::VideoCapture&            cap,
    const std::vector<cv::Mat>&  patterns,
    const ProjectorCalibConfig&  cfg,
    const std::string&           root_path,
    int                          pose_id,
    std::vector<cv::Point3f>&    obj_valid,
    std::vector<cv::Point2f>&    cam_valid,
    std::vector<cv::Point2f>&    proj_valid);
 
 
// ============================================================
//  Tính calib projector + stereo từ dữ liệu điểm đã có
//
//  proj points sẽ được remap từ hệ Gray-code (proj_w x proj_h)
//  về hệ preview window (previewW x previewH) trước khi calib.
//  Kết quả Kp, dist_p, R, T lưu vào cfg.outputJson.
// ============================================================
bool computeProjectorCalib(
    const std::vector<std::vector<cv::Point3f>>& all_obj,
    const std::vector<std::vector<cv::Point2f>>& all_cam,
    const std::vector<std::vector<cv::Point2f>>& all_proj,
    const cv::Mat&                Kc,
    const cv::Mat&                dist_c,
    const ProjectorCalibConfig&   cfg);
 
 
// ============================================================
//  Load lại ảnh từ folder đã lưu, decode rồi calib lại
// ============================================================
bool recalibrateFromFolder(
    const std::string&           root_path,
    const cv::Mat&               Kc,
    const cv::Mat&               dist_c,
    const ProjectorCalibConfig&  cfg);
 
 
// ============================================================
//  Hàm chính: vòng lặp tương tác calib projector
// ============================================================
void runProjectorCalibration(
    const ProjectorCalibConfig& cfg,
    int                         camera_url);