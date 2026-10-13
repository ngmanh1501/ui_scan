#pragma once

#include "multipose_triangulator.h"   // MultiPoseTriangulator (icpAlign/computeNormals/
                                       // applyTransform/makeRotAxis giờ public), Point3D
#include "app_config.h"               // PoseEstimationConfig, TriangulatorConfig
#include <string>
#include <vector>

// ================================================================
//  pose_estimator.h  —  Hướng B: định vị vật đã biết (reference model
//  quét bằng tri_multi/build_model) trong 1 cloud live chụp nhanh
//  (1 pose, KHÔNG quay bàn xoay).
//
//  Giả định (đã xác nhận với Viet trước khi code): vật luôn đặt CÙNG 1
//  MẶT xuống bàn xoay trong 1 lần build model — chỉ khác góc xoay quanh
//  trục đứng. Vật có thể đặt NHIỀU MẶT khác nhau giữa các lần build:
//  giải pháp là build MỖI MẶT 1 ReferenceModel riêng (nhiều file .yml
//  trong cùng 1 thư mục nhóm — xem PoseEstimationConfig::modelRoot),
//  rồi gọi estimatePoseByRegistration() cho TỪNG model, giữ kết quả
//  fitness thấp nhất. Vòng lặp "thử từng mặt" nằm ở main.cpp (cmdLocate)
//  KHÔNG nằm trong file này — hàm ở đây chỉ xử lý ĐÚNG 1 model/1 lần gọi.
//
//  Không có ICP nào mới ở đây — toàn bộ fine alignment dùng lại
//  MultiPoseTriangulator::icpAlign() nguyên vẹn.
// ================================================================

// Model tham chiếu: points + normals cùng độ dài/thứ tự (points[i] <->
// normals[i]), lấy THẲNG từ output reconstructFolder(..., &out_normals)
// — hệ toạ độ "pose_0" của riêng lần build đó, KHÔNG chuẩn hoá lại trục
// (xem trao đổi trước khi code — registration vẫn đúng bất kể model
// "hướng" thế nào trong hệ riêng của nó).
struct ReferenceModel
{
    std::vector<Point3D>   points;
    std::vector<cv::Vec3f> normals;
};

// Lưu/đọc ReferenceModel bằng cv::FileStorage (giống cách camera/projector
// calibration đang lưu — xem loadCalibration() trong decode.h) — 2 Mat
// Nx3 CV_32F: "points", "normals". Trả false nếu lỗi (in lý do ra cout).
bool saveReferenceModel(const std::string& path, const ReferenceModel& model);
bool loadReferenceModel(const std::string& path, ReferenceModel& model);

// Kết quả định vị 1 model trong 1 live cloud.
struct ObjectPose
{
    // false nếu: live cloud quá ít điểm, model rỗng, hoặc fitness sau
    // fine ICP vượt PoseEstimationConfig::pose_max_accept_fitness_mm.
    // T4x4/R/position vẫn được điền dù valid=false (giá trị "tốt nhất
    // tìm được", để debug/log — nhưng KHÔNG nên tin dùng khi valid=false).
    bool valid = false;

    cv::Point3f position;   // mm, hệ camera = cột dịch chuyển của T4x4
    cv::Mat      R;          // 3x3 CV_64F = khối xoay của T4x4
    cv::Mat      T4x4;       // 4x4 CV_64F đầy đủ (model -> live): T4x4*model ≈ live
    double       fitness = 0.0;   // percentile-30 NN residual (mm) SAU fine ICP —
                                   // càng thấp càng khớp; dùng để so sánh giữa các mặt.
};

// Định vị model trong liveCloud:
//   1) Coarse: quét K giả thuyết góc (bước cfg.pose_coarse_angle_step_deg,
//      đủ 360°) quanh triCfg.turntable_axis, tâm triCfg.turntable_center —
//      KHÔNG chạy ICP ở bước này, chỉ đo percentile-30 nearest-neighbor
//      (giống scoreCandidatePairwise() đang dùng cho autoCalibrateTurntable()),
//      giữ giả thuyết điểm số thấp nhất làm initial guess.
//   2) Fine: tri.icpAlign() đầy đủ từ initial guess đó — model là "src",
//      liveCloud là "ref" (T4x4 * model ≈ live = đúng nghĩa pose vật
//      trong hệ camera hiện tại).
//
// liveNormals: lấy từ tri.computeNormals(liveCloud, camWidth, camHeight)
// — camWidth/camHeight là kích thước ảnh CAMERA (không phải projector)
// dùng để chụp liveCloud.
ObjectPose estimatePoseByRegistration(
    const std::vector<Point3D>&   liveCloud,
    const std::vector<cv::Vec3f>& liveNormals,
    const ReferenceModel&         model,
    const MultiPoseTriangulator&  tri,
    const TriangulatorConfig&     triCfg,
    const PoseEstimationConfig&   poseCfg);