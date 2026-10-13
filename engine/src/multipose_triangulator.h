#pragma once
 
#include "triangulator.h"    // GraycodeTriangulator, Point3D (đã include app_config.h)
#include "decode.h"          // decodeGrayPixel, loadCalibration
#include "app_config.h"      // TriangulatorConfig — định nghĩa DUY NHẤT ở đây,
                              // đọc giá trị từ config.json qua AppConfig::load()
#include <filesystem>
 
// TriangulatorConfig (z_min/z_max, ICP, turntable) giờ được định nghĩa
// trong app_config.h để gộp chung với DecodeConfig/FilterConfig/AppPaths
// vào một file config.json duy nhất. Không định nghĩa lại struct ở đây
// nữa — tham số mặc định xem trong app_config.h, GIÁ TRỊ THỰC TẾ DÙNG
// LÚC CHẠY đọc từ config.json (sửa file đó, không sửa code).
// ================================================================
//  Kết quả autoCalibrateTurntable() — xem hàm cùng tên bên dưới.
// ================================================================
struct TurntableCalibResult
{
    double cx = 0.0, cy = 0.0, cz = 0.0;
    float  step_deg = 0.0f;
    int    sign = 1;
    double score_mm_before = 0.0;   // điểm số với giá trị config HIỆN TẠI
    double score_mm_after  = 0.0;   // điểm số với giá trị tìm được (càng thấp càng khớp)
};

// ================================================================
//  MultiPoseTriangulator
//
//  Dùng GraycodeTriangulator (composition) để reconstruct từng pose,
//  sau đó ICP-align tất cả về hệ toạ độ pose_0 rồi merge.
//
// ================================================================
class MultiPoseTriangulator
{
public:
    MultiPoseTriangulator(
        const Mat& Kc,  const Mat& distC,
        const Mat& Kp,  const Mat& distP,
        const Mat& R,   const Mat& T,
        const TriangulatorConfig& triCfg = TriangulatorConfig(),
        const DecodeConfig& decodeCfg = DecodeConfig(),
        const FilterConfig& filterCfg = FilterConfig());
 
    // Reconstruct + align + merge toàn bộ pose trong folder.
    // out_normals (tuỳ chọn, mặc định nullptr): khi khác null, trả thêm
    // normal đã align song song 1-1 với cloud trả về (cùng thứ tự, cùng
    // độ dài) — dùng để build reference model cho Hướng B (xem
    // pose_estimator.h). Mặc định nullptr nên lời gọi cũ (cmdTriMulti...)
    // không cần sửa gì.
    vector<Point3D> reconstructFolder(
        const string& folder,
        const vector<string>& poses,
        int proj_w,
        int proj_h,
        vector<Vec3f>* out_normals = nullptr);

    // Tìm turntable_center (chỉ cx,cz — xem lý do bỏ cy trong .cpp) +
    // turntable_step_deg + chiều quay (sign) khớp nhất với DỮ LIỆU ĐÃ CHỤP,
    // bằng grid search thô rồi coordinate-descent tinh. KHÔNG dùng ICP —
    // chỉ đo mức khớp hình học thuần túy (nearest-neighbor giữa các pose
    // LIỀN KỀ). KHÔNG tự ghi vào cfg hay config.json — chỉ trả kết quả.
    // Ghi thêm 2 file PLY debug (truoc/sau) để so sánh trực quan.
    TurntableCalibResult autoCalibrateTurntable(
        const string& folder,
        const vector<string>& poses,
        int proj_w,
        int proj_h);

    // Áp transform "không ICP" (chỉ hình học: center+axis+step_deg+sign)
    // lên từng pose rồi merge + lưu PLY. Dùng để kiểm tra center/step/sign
    // có hợp lý không, TÁCH RIÊNG khỏi việc ICP tune đúng/sai — vì ICP có
    // thể tự sửa (hoặc tự trôi) nên nhìn mỗi kết quả sau-ICP không đủ để
    // biết tham số hình học gốc có đúng không.
    void exportNoICP(
        const vector<vector<Point3D>>& clouds,
        double cx, double cy, double cz,
        float step_deg, int sign,
        const string& filename) const;

    // Phải gọi trước reconstructFolder() — truyền scale từ projector_calibration.json
    void setProjectorScale(float sx, float sy)
    {
        proj_scale_x_ = sx;
        proj_scale_y_ = sy;
    }
 
    // Delegate savePLY sang base triangulator — const vì chỉ đọc member,
    // không sửa state, cho phép gọi từ các hàm const khác (exportNoICP...).
    void savePLY(const vector<Point3D>& cloud,
                 const string& filename) const;

    // ---- Hướng B (object pose bằng registration model↔live) ----
    // 4 hàm dưới đây TRƯỚC LÀ private, chuyển lên public NGUYÊN VẸN (không
    // đổi logic/chữ ký) vì pose_estimator.cpp cần gọi trực tiếp — không
    // viết lại thuật toán, chỉ tái dùng. Xem comment gốc từng hàm trong
    // multipose_triangulator.cpp.

    // Tạo ma trận xoay 4×4 (CV_64F) quanh trục tùy ý, góc tính bằng radian
    Mat makeRotAxis(float ax, float ay, float az, double angle_rad) const;

    // Tính normal xấp xỉ tại mỗi điểm bằng finite-difference giữa hàng xóm
    // liền kề trong lưới (cam_u,cam_v). width/height = kích thước ảnh
    // camera gốc (không phải projector).
    vector<Vec3f> computeNormals(
        const vector<Point3D>& cloud,
        int width, int height) const;

    // Point-to-plane ICP (linearized, Low 2004) với initial guess T_init
    // (4×4 CV_64F). Trả về T4x4 tổng (CV_64F) sao cho T4x4 * src ≈ ref —
    // dùng cho Hướng B với src=model, ref=live (xem pose_estimator.cpp).
    Mat icpAlign(
        const vector<Point3D>& src,
        const vector<Vec3f>& src_normals,
        const vector<Point3D>& ref,
        const vector<Vec3f>& ref_normals,
        const Mat& T_init) const;

    // Áp T4x4 lên cloud (chỉ x,y,z — giữ nguyên cam_u/v, proj_u/v)
    vector<Point3D> applyTransform(
        const vector<Point3D>& cloud,
        const Mat& T4x4) const;

    // Config đầy đủ (z range, ICP, turntable) — sửa trực tiếp nếu cần
    // tinh chỉnh sau khi khởi tạo, hoặc truyền sẵn vào constructor.
    TriangulatorConfig cfg;
 
private:
    // Params lưu lại để tạo GraycodeTriangulator cho từng pose
    Mat Kc_, distC_, Kp_, distP_, R_, T_;

    // DecodeConfig/FilterConfig lưu lại để mỗi pose dùng cùng tham số
    // decode/filter (đọc từ config.json), thay vì GraycodeTriangulator
    // tự dùng giá trị mặc định hard-code riêng cho từng pose.
    DecodeConfig decodeCfg_;
    FilterConfig filterCfg_;

    // Scale remap proj coords: Gray-code [0..proj_w/h] → preview [0..previewW/H].
    // Phải set qua setProjectorScale() trước khi gọi reconstructFolder(),
    // vì mỗi GraycodeTriangulator tạo bên trong cần scale này.
    float proj_scale_x_ = 1.0f;
    float proj_scale_y_ = 1.0f;
 
    // Đọc ảnh từ pose_path/pattern_i.png + white.png + black.png
    bool loadPoseImages(
        const string& pose_path,
        int proj_w, int proj_h,
        vector<Mat>& images) const;

    // Reconstruct point cloud + normal THÔ (chưa align) cho MỌI pose trong
    // folder — dùng chung bởi reconstructFolder() và autoCalibrateTurntable(),
    // tránh lặp lại vòng lặp load-ảnh + triangulate + computeNormals ở 2 nơi.
    bool reconstructAllPoses(
        const string& folder,
        const vector<string>& poses,
        int proj_w, int proj_h,
        vector<vector<Point3D>>& clouds,
        vector<vector<Vec3f>>& normals) const;
 
    // Áp phần xoay (3×3 top-left) của T4x4 lên normal — KHÔNG cộng
    // translation. Vec3f(0,0,0) (invalid) xoay ra vẫn là (0,0,0), nên
    // không cần xử lý riêng.
    vector<Vec3f> rotateNormals(
        const vector<Vec3f>& normals,
        const Mat& T4x4) const;

    // ---- Helper cho autoCalibrateTurntable() ----

    // Nx3 CV_32F Mat từ cloud — dùng để build cv::flann::Index.
    Mat cloudToMat(const vector<Point3D>& cloud) const;

    // Random subsample (seed cố định — kết quả lặp lại được giữa các lần
    // chạy) mỗi pose xuống tối đa maxPts điểm, dùng cho giai đoạn THÔ của
    // autoCalibrateTurntable() để nhanh hơn nhiều so với full cloud.
    vector<vector<Point3D>> subsampleClouds(
        const vector<vector<Point3D>>& clouds, int maxPts) const;

    // Phần TÌM KIẾM thuần của autoCalibrateTurntable() — nhận thẳng clouds
    // (không tự đọc ảnh), nên test được bằng dữ liệu synthetic. Const vì
    // chỉ đọc cfg + gọi scoreCandidatePairwise/exportNoICP (đều const).
    TurntableCalibResult runAutoCalibSearch(
        const vector<vector<Point3D>>& clouds) const;

    // Điểm số cho 1 bộ (cx,cy,cz,step_deg,sign): áp transform "không ICP"
    // lên mọi pose, rồi với TỪNG CẶP LIỀN KỀ (i,i+1) đo percentile-30 của
    // khoảng cách nearest-neighbor (mm) — dùng cặp liền kề (không phải luôn
    // so với pose_0) vì luôn có overlap thật bất kể tổng góc quay, khác với
    // so mọi pose với pose_0 (pose ở 150-180° gần như không overlap thật
    // với pose_0 dù center/step có đúng hay không). Điểm càng THẤP càng khớp.
    double scoreCandidatePairwise(
        const vector<vector<Point3D>>& clouds,
        double cx, double cy, double cz,
        float step_deg, int sign) const;
};