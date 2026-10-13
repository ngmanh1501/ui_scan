#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/structured_light.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "app_config.h"   // DecodeConfig, FilterConfig — tham số đọc từ config.json

using namespace cv;
using namespace std;



struct Point3D
{
    float x, y, z;

    int cam_u;
    int cam_v;

    float proj_u;
    float proj_v;
};

 

class GraycodeTriangulator {
public:
    GraycodeTriangulator(
        const Mat& Kc_,
        const Mat& distC_,     // 🔥 THÊM
        const Mat& Kp_,
        const Mat& distP_,     // 🔥 THÊM
        const Mat& R_,
        const Mat& T_,
        const TriangulatorConfig& triCfg);   // z_min/z_max/x_min/x_max/y_min/y_max — 1 nguồn duy nhất

    // Cấu hình tỉ lệ remap proj_u/proj_v từ hệ Gray-code gốc
    // [0..proj_w, 0..proj_h] sang hệ preview [0..previewW, 0..previewH]
    // — hệ mà Kp/dist_p đã được calib. Đọc scale_x/scale_y trực tiếp
    // từ projector_calibration.json (xem main.cpp), KHÔNG hardcode ở đây
    // để khi đổi previewW/H hay proj_w/h thì không cần sửa code.
    // Mặc định = 1.0 (không remap) để tương thích ngược nếu không gọi.
    void setProjectorScale(float scale_x, float scale_y)
    {
        proj_scale_x = scale_x;
        proj_scale_y = scale_y;

    }

    vector<Point3D> reconstruct(
        const vector<Mat>& patterns,
        int proj_w,
        int proj_h
    );

    // ---- Overload MỚI: nhận DecodeConfig + FilterConfig từ ngoài ----
    // Dùng overload này nếu muốn tham số decode (threshold, maxLg, ...)
    // và tham số filter (reproj_max_err_px, spatial_max_jump, iqr_scale)
    // được đọc từ config.json thay vì hard-code trong triangulator.cpp.
    // Bản reconstruct() cũ ở trên vẫn giữ nguyên (dùng giá trị mặc định
    // hard-code) để không phá vỡ code cũ nếu có nơi khác đang gọi nó.
    vector<Point3D> reconstruct(
        const vector<Mat>& patterns,
        int proj_w,
        int proj_h,
        const DecodeConfig& decodeCfg,
        const FilterConfig& filterCfg
    );

    void savePLY(const vector<Point3D>& cloud, const string& filename);

private:
    Mat Kc, Kp, R, T;
    float fx, fy, cx, cy;
    float z_min, z_max;
    float x_min, x_max;
    float y_min, y_max;
    Mat distCoeffs_c, distCoeffs_p;
    float min_z_seen = 1e9f;
    float max_z_seen = -1e9f;
    float proj_scale_x = 1.0f;
    float proj_scale_y = 1.0f;

    struct TriStats
    {
        int fail_w = 0;
        int fail_zcam = 0;
        int fail_zproj = 0;
        int fail_xcam = 0;
        int fail_ycam = 0;
    };

    TriStats stats;

    bool triangulatePoint(int u, int v, float proj_u, float proj_v, Point3D& pt);
    vector<Point3D> filterReprojection(
        const vector<Point3D>& input,
        float max_err_px = 5.0f
    );

    vector<Point3D> filterSpatialConsistency(
        const vector<Point3D>& input,
        int width,
        int height,
        float max_jump = 5.0f
    );

    vector<Point3D> filterIQR(
        const vector<Point3D>& input,
        float scale = 3.0f
    );

};