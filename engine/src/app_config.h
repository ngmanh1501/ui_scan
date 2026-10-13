#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <filesystem>

// ================================================================
//  app_config.h
//
//  TOÀN BỘ tham số tuning của pipeline (triangulation, decode,
//  filter, ICP, turntable, đường dẫn, kích thước projector...)
//  được gom vào MỘT struct AppConfig duy nhất, đọc/ghi bằng JSON
//  qua cv::FileStorage (giống cách camera_calibration.json đang
//  được load trong decode.h).
//
//  MỤC TIÊU: sửa tham số chỉ cần mở "config.json" bằng Notepad,
//  KHÔNG cần sửa bất kỳ file .cpp/.h nào rồi build lại.
//
//  CÁCH DÙNG:
//      AppConfig cfg;
//      AppConfig::load("config.json", cfg);   // tự tạo file mặc định
//                                              // nếu chưa tồn tại
//      ... dùng cfg.tri.z_min, cfg.decode.threshold, cfg.app.proj_w ...
//
//  Nếu thêm tham số mới: thêm field vào struct tương ứng bên dưới
//  + thêm 1 dòng đọc/ghi trong load()/save(). Mọi nơi khác trong
//  code chỉ cần đọc qua cfg, không hard-code số nữa.
// ================================================================

// ---------------------------------------------------------------
//  DecodeConfig — tham số decode Gray-code (trước đây hard-code
//  trong GraycodeTriangulator::reconstruct())
// ---------------------------------------------------------------
struct DecodeConfig
{
    int bits                  = 10;
    int threshold              = 10;     // sàn dưới threshold adaptive
    bool useAdaptiveThreshold  = true;   // bật max(threshold, Ld/10)
    int maxLg                  = 230;    // loại pixel bão hoà (Lg > maxLg), 0 = tắt
    int whiteIndex             = 40;
    int blackIndex             = 41;
    int horizontalStart        = 0;
    int verticalStart          = 20;
};

// ---------------------------------------------------------------
//  FilterConfig — ngưỡng các bước lọc point cloud (trước đây
//  hard-code ngay tại lời gọi filterReprojection/filterSpatial/
//  filterIQR trong reconstruct())
// ---------------------------------------------------------------
struct FilterConfig
{
    float reproj_max_err_px   = 15.0f;   // filterReprojection
    float spatial_max_jump    = 5.0f;    // filterSpatialConsistency
    float iqr_scale           = 1.2f;    // filterIQR
};

// ---------------------------------------------------------------
//  TriangulatorConfig — z range, ICP, turntable
//  (giữ nguyên ý nghĩa như bản cũ trong multipose_triangulator.h,
//  chỉ khác là giá trị mặc định giờ được NẠP TỪ JSON, không còn
//  hard-code trong struct)
// ---------------------------------------------------------------
struct TriangulatorConfig
{
    // ---- Z range (mm) ----
    float z_min = 300.0f;
    float z_max = 2000.0f;

    // ---- X/Y range (mm, hệ camera) — cùng cơ chế với Z range: cắt
    // điểm ngoài bounding-box ngay lúc triangulate, trước các bước lọc
    // thống kê (IQR/reproj/spatial). Mặc định ±1e6 = không giới hạn,
    // để config.json cũ (chưa có 4 key này) không đổi hành vi.
    float x_min = -1e6f;
    float x_max = 1e6f;
    float y_min = -1e6f;
    float y_max = 1e6f;

    // ---- Tuning ICP ----
    int   icp_iter    = 80;

    // Trước đây là ngưỡng CHÍNH để chọn correspondence (khoảng cách tuyệt
    // đối, mm). Giờ chỉ còn vai trò TRẦN AN TOÀN tuyệt đối — loại correspondence
    // rõ ràng vô lý (khớp nhầm sang vùng khác của vật thể) dù đã lọt qua
    // icp_trim_ratio bên dưới. Vai trò "ngưỡng chính" chuyển sang
    // icp_trim_ratio, vì 1 số mm cố định không có "giá trị đúng" độc lập —
    // độ phù hợp của nó phụ thuộc residual còn lại sau initial guess, thứ
    // thay đổi theo từng bộ dữ liệu/từng vòng lặp.
    float icp_dist_mm = 15.0f;

    // Số ứng viên nearest-neighbor xét cho MỖI điểm source (trước đây cứng
    // K=1). FLANN trả về K ứng viên đã sort tăng dần theo khoảng cách; lấy
    // ứng viên GẦN NHẤT trong số còn vượt qua icp_dist_mm + icp_normal_max_deg
    // bên dưới — tránh trường hợp điểm gần nhất tuyệt đối lại là nhiễu nằm
    // gần bề mặt thật (nhiễu rời rạc, hoặc bề mặt có "độ dày" chứ không phải
    // 1 lớp mỏng lý tưởng).
    int icp_knn_k = 5;

    // Ngưỡng góc tối đa (độ) giữa normal của source (đã xoay theo transform
    // hiện tại của vòng lặp) và normal của ref tại điểm ứng viên — loại
    // ứng viên nếu góc vượt ngưỡng (dot < cos(ngưỡng)), dù đã qua icp_dist_mm.
    // Dùng dot CÓ DẤU (không lấy trị tuyệt đối): correspondence đúng (cùng
    // 1 bề mặt vật lý, sau khi align đúng hướng) phải có normal gần như
    // cùng hướng; khớp nhầm sang mặt khác (cạnh/góc hộp, 2 mặt đối nhau của
    // vật mỏng) cho dot thấp hoặc âm — lấy |dot| sẽ vô tình chấp nhận lại
    // đúng trường hợp này.
    float icp_normal_max_deg = 65.0f;

    // Tỷ lệ correspondence có RESIDUAL POINT-TO-PLANE (|n·(p-q)|, không
    // phải dist2 Point-to-Point như trước) nhỏ nhất được giữ lại mỗi vòng
    // lặp ICP, sau khi đã qua icp_dist_mm + icp_normal_max_deg ở trên — ví
    // dụ 0.7 = giữ 70% residual nhỏ nhất, bỏ 30% lớn nhất coi là sai/outlier.
    // Đổi từ dist2 sang residual vì đó mới là đại lượng ICP point-to-plane
    // thực sự đang tối ưu — phân biệt được cả trường hợp "gần nhau trong
    // không gian nhưng lệch dọc theo normal" (bề mặt dày) mà dist2 không
    // thấy. Tự thích nghi theo residual THỰC TẾ của chính vòng lặp đó: vòng
    // đầu residual còn lớn thì "70% nhỏ nhất" vẫn là khoảng lớn; càng hội tụ
    // thì tự co lại theo — không cần lịch trình ngưỡng thô→tinh thủ công.
    // Luôn giữ tối thiểu 6 cặp dù tỷ lệ nhỏ hơn 6/số_candidate.
    float icp_trim_ratio = 0.7f;

    float icp_conv_mm = 0.1f;

    // Regularization MỀM ép dịch chuyển dọc icp_reg_axis (xem field ngay
    // dưới) về gần 0 mỗi vòng lặp ICP — xem chi tiết công thức tại nơi
    // dùng (icpAlign(), bước 2, ngay trước khi solve()). Về mặt vật lý,
    // xoay đúng quanh trục quay thì toạ độ dọc trục đó không đổi giữa
    // các pose; nếu ICP liên tục dịch theo hướng này, thường là dấu hiệu
    // correspondence thiếu ràng buộc theo hướng đó (vd normal đa số nằm
    // ngang, gần vuông góc với trục), không phải chuyển động thật của vật.
    // 0 = tắt hẳn, giữ đúng hành vi cũ (không ép gì). Số càng lớn càng ép
    // chặt về 0, số nhỏ để ICP tự do hơn theo hướng trục — cần tự tinh
    // chỉnh theo dữ liệu thật, không có công thức tính sẵn đúng cho mọi
    // trường hợp vì còn phụ thuộc số lượng correspondence mỗi vòng lặp.
    float icp_reg_ty_weight = 20.0f;

    // Hướng bị ép ở trên — TÁCH RIÊNG khỏi turntable_axis (trước đây dùng
    // chung), vì 2 việc khác mục đích: turntable_axis quyết định T_init
    // (hình học góc quay ban đầu mỗi pose), còn cái này chỉ quyết định
    // regularization ép ICP RIÊNG. Đổi turntable_axis để tinh chỉnh T_init
    // sẽ vô tình đổi luôn hướng regularization nếu dùng chung 1 field —
    // dễ nhầm "chỉnh cho đẹp" trong khi thực ra đang sửa nhầm chỗ. Mặc
    // định vẫn (0,1,0) — khớp turntable_axis mặc định, hành vi không đổi
    // nếu bạn chưa từng đổi turntable_axis. Đo trực tiếp qua log "[LOOP]
    // khe ho" (in riêng x/y/z) trước khi chỉnh field này, đừng đoán.
    float icp_reg_axis[3] = {0.0f, 1.0f, 0.0f};

    // Bật/tắt bước loop closure (pose cuối -> pose 0) sau khi ghép sequential
    // xong — 0 = tắt hẳn (giữ đúng hành vi trước khi có bước này).
    int icp_loop_closure_enable = 1;

    // Số vòng global refinement (mỗi pose ICP lại với TOÀN BỘ pose còn lại,
    // pose 0 cố định) sau loop closure — 0 = bỏ qua hẳn bước này. Cần đủ
    // lớn để hiệu ứng loop closure (chỉ sửa 1 mình pose cuối) lan đều ra
    // toàn bộ vòng qua nhiều vòng lặp — xem log "[GLOBAL] round ... max
    // correction" để biết đã đủ vòng chưa (còn lớn thì tăng thêm).
    int icp_global_refine_iters = 2;

    // ---- Turntable params ----
    int   turntable_poses    = 12;
    float turntable_step_deg = 0.0f;     // 0 = tự tính 360/turntable_poses
    float turntable_axis[3]  = {0.0f, 1.0f, 0.0f};

    // +1 hoặc -1 — chiều quay thực tế của bàn xoay so với công thức
    // angle=-(i*step) đang dùng. Nếu autoCalibrateTurntable() báo sign=-1
    // nghĩa là chiều đang giả định bị ngược — sửa số này trong config.json,
    // KHÔNG cần sửa code.
    int turntable_sign = 1;

    // 1 điểm CỐ ĐỊNH nằm trên trục quay, hệ toạ độ camera (mm).
    // Đo/tính MỘT LẦN cho bộ gá (camera + bàn xoay) — dùng chung cho
    // MỌI vật đặt lên bàn xoay, không suy ra từ point cloud của vật
    // nữa. Thay thế cách cũ (centroid pose_0 + object_radius_mm),
    // vốn ngầm giả định vật đối xứng tròn quanh trục quay nên chỉ
    // đúng với hình trụ — sai với vật hình hộp (centroid bề mặt nhìn
    // thấy đổi theo mặt nào đang hướng ra camera, và khoảng cách từ
    // trục quay đến bề mặt không cố định).
    float turntable_center[3] = {0.0f, 0.0f, 0.0f};

    // KHÔNG còn dùng để tính tâm xoay nữa (xem turntable_center ở
    // trên) — giữ lại field để không phá config.json cũ nếu còn nơi
    // nào khác đọc tới.
    float object_radius_mm   = 40.0f;
};

// ---------------------------------------------------------------
//  AppPaths — đường dẫn & kích thước projector (trước đây hard-code
//  trong main())
// ---------------------------------------------------------------
struct AppPaths
{
    int    proj_w               = 1024;
    int    proj_h                = 768;
    int    camera_url            = 0;
    std::string objectDataRoot   = "Object_data";
    std::string cameraCalibFile  = "camera_calibration.json";
    std::string projectorCalibFile = "projector_calibration.json";
};

// ---------------------------------------------------------------
//  ProjectorCalibParams — tham số calib projector (trước đây hard-code
//  trong struct ProjectorCalibConfig ở projector_calib.h; main.cpp sẽ
//  copy các field này sang ProjectorCalibConfig trước khi gọi
//  runProjectorCalibration()/recalibrateFromFolder()).
//
//  KHÔNG lặp lại proj_w/proj_h ở đây — 2 field đó đã có trong AppPaths
//  và main.cpp đã gán projCfg.proj_w/proj_h = cfg.app.proj_w/proj_h.
//  Cũng KHÔNG có outputJson — main.cpp dùng thẳng cfg.app.projectorCalibFile
//  để tránh 2 đường dẫn độc lập cùng trỏ 1 file rồi lệch nhau khi sửa config.
// ---------------------------------------------------------------
struct ProjectorCalibParams
{
    // ---- Preview window (không gian projector THỰC SỰ chiếu sáng,
    // dùng để tính scaleX/scaleY và làm image size cho calibrateCamera()) ----
    int previewW = 1024;
    int previewH = 768;

    // ---- Checkerboard vật lý ----
    int   board_cols    = 9;      // số corner ngang = cv::Size(boardSize).width
    int   board_rows    = 7;      // số corner dọc   = cv::Size(boardSize).height
    float squareSize_mm = 23.0f;

    // ---- Decode tại patch quanh corner trong decodePose() ----
    // Lưu ý: threshold ở đây KHÁC decode.threshold của DecodeConfig
    // (threshold đó dùng cho decode toàn frame khi scan vật thể, còn
    // threshold này chỉ dùng khi decode patch quanh checkerboard corner
    // lúc calib projector). Nằm ở section riêng "projectorCalib" trong
    // config.json nên không đụng nhau.
    int threshold       = 35;
    int patchRadius     = 2;
    int minDecodePoints = 15;

    // ---- Capture timing khi chụp pose calib ----
    int warmupFrames   = 5;
    int patternDelayMs = 300;

    // ---- Số pose tối thiểu để computeProjectorCalib() chấp nhận tính
    // (trước đây là const int MIN_POSES = 5; hard-code trong .cpp) ----
    int minPoses = 5;
};

// ---------------------------------------------------------------
//  PoseEstimationConfig — Hướng B: định vị vật đã biết (reference
//  model quét bằng tri_multi/build_model) trong 1 cloud live chụp
//  nhanh (1 pose, không quay bàn xoay) — dùng bởi build_model/locate
//  trong main.cpp và pose_estimator.h/.cpp.
//
//  Giả định: vật luôn đặt CÙNG 1 MẶT xuống bàn xoay trong 1 lần build
//  model (chỉ khác góc xoay quanh trục đứng) — cho phép coarse search
//  chỉ cần dò 1 chiều (góc) quanh turntable_axis/turntable_center của
//  TriangulatorConfig, thay vì tìm kiếm toàn bộ hướng xoay 3D. Vật có
//  thể đặt nhiều mặt khác nhau giữa các lần: build MỖI MẶT 1 reference
//  model riêng (nhiều file .yml trong cùng 1 thư mục nhóm — xem
//  modelRoot); locate() thử lần lượt từng model, giữ model khớp nhất
//  (vòng lặp "thử từng mặt" nằm ở main.cpp, không nằm trong config này).
// ---------------------------------------------------------------
struct PoseEstimationConfig
{
    // Bước góc (độ) giữa các giả thuyết coarse search, quét đủ 360°
    // quanh turntable_axis, tâm turntable_center. Không cần chạy ICP ở
    // bước này (giống cách scoreCandidatePairwise() đang chấm điểm cho
    // autoCalibrateTurntable()) — chỉ đo percentile-30 nearest-neighbor
    // (mm) giữa model đã xoay theo giả thuyết và live cloud, càng thấp
    // càng khớp. Giả thuyết tốt nhất làm initial guess cho fine ICP.
    float pose_coarse_angle_step_deg = 15.0f;

    // Dưới ngưỡng này (số điểm live cloud sau reconstruct) thì từ chối
    // luôn, không đủ tin cậy để chạy registration.
    int pose_min_live_points = 500;

    // Vượt ngưỡng fitness (mm, percentile-30 residual SAU fine ICP) này
    // thì coi là KHÔNG khớp đủ tin cậy — locate() báo lỗi thay vì trả
    // bừa ra model tốt-nhất-trong-những-cái-tệ khi thử qua nhiều mặt.
    double pose_max_accept_fitness_mm = 5.0;

    // Thư mục gốc chứa reference model đã build — mỗi vật 1 thư mục con
    // (tên do người dùng đặt khi build_model), mỗi mặt đã build 1 file
    // .yml trong đó.
    std::string modelRoot = "Object_models";
};

// ---------------------------------------------------------------
//  AppConfig — gộp tất cả nhóm trên vào 1 chỗ
// ---------------------------------------------------------------
struct AppConfig
{
    DecodeConfig         decode;
    FilterConfig         filter;
    TriangulatorConfig   tri;
    AppPaths             app;
    ProjectorCalibParams projCalib;
    PoseEstimationConfig poseEst;

    // ============================================================
    //  save() — ghi toàn bộ config hiện tại ra file JSON.
    //  Dùng để tự sinh config.json mặc định lần đầu chạy, hoặc khi
    //  muốn "xuất" lại config hiện tại ra file.
    // ============================================================
    static bool save(const std::string& path, const AppConfig& cfg)
    {
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        if (!fs.isOpened())
        {
            std::cout << "[CONFIG] Khong the tao file " << path << "\n";
            return false;
        }

        fs << "decode" << "{";
        fs << "bits"                 << cfg.decode.bits;
        fs << "threshold"            << cfg.decode.threshold;
        fs << "useAdaptiveThreshold" << cfg.decode.useAdaptiveThreshold;
        fs << "maxLg"                << cfg.decode.maxLg;
        fs << "whiteIndex"           << cfg.decode.whiteIndex;
        fs << "blackIndex"           << cfg.decode.blackIndex;
        fs << "horizontalStart"      << cfg.decode.horizontalStart;
        fs << "verticalStart"        << cfg.decode.verticalStart;
        fs << "}";

        fs << "filter" << "{";
        fs << "reproj_max_err_px" << cfg.filter.reproj_max_err_px;
        fs << "spatial_max_jump"  << cfg.filter.spatial_max_jump;
        fs << "iqr_scale"         << cfg.filter.iqr_scale;
        fs << "}";

        fs << "triangulator" << "{";
        fs << "z_min"               << cfg.tri.z_min;
        fs << "z_max"               << cfg.tri.z_max;
        fs << "x_min"               << cfg.tri.x_min;
        fs << "x_max"               << cfg.tri.x_max;
        fs << "y_min"               << cfg.tri.y_min;
        fs << "y_max"               << cfg.tri.y_max;
        fs << "icp_iter"            << cfg.tri.icp_iter;
        fs << "icp_dist_mm"         << cfg.tri.icp_dist_mm;
        fs << "icp_knn_k"           << cfg.tri.icp_knn_k;
        fs << "icp_normal_max_deg"  << cfg.tri.icp_normal_max_deg;
        fs << "icp_trim_ratio"      << cfg.tri.icp_trim_ratio;
        fs << "icp_conv_mm"         << cfg.tri.icp_conv_mm;
        fs << "icp_reg_ty_weight"   << cfg.tri.icp_reg_ty_weight;
        fs << "icp_reg_axis_x"      << cfg.tri.icp_reg_axis[0];
        fs << "icp_reg_axis_y"      << cfg.tri.icp_reg_axis[1];
        fs << "icp_reg_axis_z"      << cfg.tri.icp_reg_axis[2];
        fs << "icp_loop_closure_enable"  << cfg.tri.icp_loop_closure_enable;
        fs << "icp_global_refine_iters"  << cfg.tri.icp_global_refine_iters;
        fs << "turntable_poses"     << cfg.tri.turntable_poses;
        fs << "turntable_step_deg"  << cfg.tri.turntable_step_deg;
        fs << "turntable_sign"      << cfg.tri.turntable_sign;
        fs << "turntable_axis_x"    << cfg.tri.turntable_axis[0];
        fs << "turntable_axis_y"    << cfg.tri.turntable_axis[1];
        fs << "turntable_axis_z"    << cfg.tri.turntable_axis[2];
        fs << "turntable_center_x"  << cfg.tri.turntable_center[0];
        fs << "turntable_center_y"  << cfg.tri.turntable_center[1];
        fs << "turntable_center_z"  << cfg.tri.turntable_center[2];
        fs << "object_radius_mm"    << cfg.tri.object_radius_mm;
        fs << "}";

        fs << "app" << "{";
        fs << "proj_w"                << cfg.app.proj_w;
        fs << "proj_h"                << cfg.app.proj_h;
        fs << "camera_url"            << cfg.app.camera_url;
        fs << "objectDataRoot"        << cfg.app.objectDataRoot;
        fs << "cameraCalibFile"       << cfg.app.cameraCalibFile;
        fs << "projectorCalibFile"    << cfg.app.projectorCalibFile;
        fs << "}";

        fs << "projectorCalib" << "{";
        fs << "previewW"         << cfg.projCalib.previewW;
        fs << "previewH"         << cfg.projCalib.previewH;
        fs << "board_cols"       << cfg.projCalib.board_cols;
        fs << "board_rows"       << cfg.projCalib.board_rows;
        fs << "squareSize_mm"    << cfg.projCalib.squareSize_mm;
        fs << "threshold"        << cfg.projCalib.threshold;
        fs << "patchRadius"      << cfg.projCalib.patchRadius;
        fs << "minDecodePoints"  << cfg.projCalib.minDecodePoints;
        fs << "warmupFrames"     << cfg.projCalib.warmupFrames;
        fs << "patternDelayMs"   << cfg.projCalib.patternDelayMs;
        fs << "minPoses"         << cfg.projCalib.minPoses;
        fs << "}";

        fs << "poseEstimation" << "{";
        fs << "pose_coarse_angle_step_deg"  << cfg.poseEst.pose_coarse_angle_step_deg;
        fs << "pose_min_live_points"        << cfg.poseEst.pose_min_live_points;
        fs << "pose_max_accept_fitness_mm"  << cfg.poseEst.pose_max_accept_fitness_mm;
        fs << "modelRoot"                   << cfg.poseEst.modelRoot;
        fs << "}";

        fs.release();
        std::cout << "[CONFIG] Da ghi " << path << "\n";
        return true;
    }

    // ============================================================
    //  load() — đọc config từ file JSON.
    //  Nếu file KHÔNG TỒN TẠI: tự tạo file mới với giá trị mặc định
    //  (định nghĩa ngay trong các struct ở trên) rồi đọc lại, để lần
    //  chạy đầu tiên không bị crash vì thiếu file.
    //  Nếu file tồn tại nhưng THIẾU một vài key: field tương ứng giữ
    //  nguyên giá trị mặc định (FileStorage không ghi đè nếu node
    //  không tồn tại), nên thêm field mới vào struct không làm vỡ
    //  config.json cũ.
    // ============================================================
    static bool load(const std::string& path, AppConfig& cfg)
    {
        if (!std::filesystem::exists(path))
        {
            std::cout << "[CONFIG] Khong thay " << path
                       << " -> tao file mac dinh.\n";
            save(path, cfg);   // cfg đang ở giá trị mặc định của struct
            return true;
        }

        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            std::cout << "[CONFIG] Khong mo duoc " << path
                       << " -> dung gia tri mac dinh.\n";
            return false;
        }

        cv::FileNode dn = fs["decode"];
        if (!dn.empty())
        {
            if (!dn["bits"].empty())                 dn["bits"]                 >> cfg.decode.bits;
            if (!dn["threshold"].empty())             dn["threshold"]            >> cfg.decode.threshold;
            if (!dn["useAdaptiveThreshold"].empty())  dn["useAdaptiveThreshold"] >> cfg.decode.useAdaptiveThreshold;
            if (!dn["maxLg"].empty())                 dn["maxLg"]                >> cfg.decode.maxLg;
            if (!dn["whiteIndex"].empty())            dn["whiteIndex"]           >> cfg.decode.whiteIndex;
            if (!dn["blackIndex"].empty())            dn["blackIndex"]           >> cfg.decode.blackIndex;
            if (!dn["horizontalStart"].empty())       dn["horizontalStart"]      >> cfg.decode.horizontalStart;
            if (!dn["verticalStart"].empty())         dn["verticalStart"]        >> cfg.decode.verticalStart;
        }

        cv::FileNode fn = fs["filter"];
        if (!fn.empty())
        {
            if (!fn["reproj_max_err_px"].empty()) fn["reproj_max_err_px"] >> cfg.filter.reproj_max_err_px;
            if (!fn["spatial_max_jump"].empty())  fn["spatial_max_jump"]  >> cfg.filter.spatial_max_jump;
            if (!fn["iqr_scale"].empty())         fn["iqr_scale"]         >> cfg.filter.iqr_scale;
        }

        cv::FileNode tn = fs["triangulator"];
        if (!tn.empty())
        {
            if (!tn["z_min"].empty())              tn["z_min"]              >> cfg.tri.z_min;
            if (!tn["z_max"].empty())              tn["z_max"]              >> cfg.tri.z_max;
            if (!tn["x_min"].empty())              tn["x_min"]              >> cfg.tri.x_min;
            if (!tn["x_max"].empty())              tn["x_max"]              >> cfg.tri.x_max;
            if (!tn["y_min"].empty())              tn["y_min"]              >> cfg.tri.y_min;
            if (!tn["y_max"].empty())              tn["y_max"]              >> cfg.tri.y_max;
            if (!tn["icp_iter"].empty())           tn["icp_iter"]           >> cfg.tri.icp_iter;
            if (!tn["icp_dist_mm"].empty())        tn["icp_dist_mm"]        >> cfg.tri.icp_dist_mm;
            if (!tn["icp_knn_k"].empty())          tn["icp_knn_k"]          >> cfg.tri.icp_knn_k;
            if (!tn["icp_normal_max_deg"].empty()) tn["icp_normal_max_deg"] >> cfg.tri.icp_normal_max_deg;
            if (!tn["icp_trim_ratio"].empty())     tn["icp_trim_ratio"]     >> cfg.tri.icp_trim_ratio;
            if (!tn["icp_conv_mm"].empty())        tn["icp_conv_mm"]        >> cfg.tri.icp_conv_mm;
            if (!tn["icp_reg_ty_weight"].empty())  tn["icp_reg_ty_weight"]  >> cfg.tri.icp_reg_ty_weight;
            if (!tn["icp_reg_axis_x"].empty())     tn["icp_reg_axis_x"]     >> cfg.tri.icp_reg_axis[0];
            if (!tn["icp_reg_axis_y"].empty())     tn["icp_reg_axis_y"]     >> cfg.tri.icp_reg_axis[1];
            if (!tn["icp_reg_axis_z"].empty())     tn["icp_reg_axis_z"]     >> cfg.tri.icp_reg_axis[2];
            if (!tn["icp_loop_closure_enable"].empty())  tn["icp_loop_closure_enable"]  >> cfg.tri.icp_loop_closure_enable;
            if (!tn["icp_global_refine_iters"].empty())  tn["icp_global_refine_iters"]  >> cfg.tri.icp_global_refine_iters;
            if (!tn["turntable_poses"].empty())    tn["turntable_poses"]    >> cfg.tri.turntable_poses;
            if (!tn["turntable_step_deg"].empty()) tn["turntable_step_deg"] >> cfg.tri.turntable_step_deg;
            if (!tn["turntable_sign"].empty())     tn["turntable_sign"]     >> cfg.tri.turntable_sign;
            if (!tn["turntable_axis_x"].empty())   tn["turntable_axis_x"]   >> cfg.tri.turntable_axis[0];
            if (!tn["turntable_axis_y"].empty())   tn["turntable_axis_y"]   >> cfg.tri.turntable_axis[1];
            if (!tn["turntable_axis_z"].empty())   tn["turntable_axis_z"]   >> cfg.tri.turntable_axis[2];
            if (!tn["turntable_center_x"].empty()) tn["turntable_center_x"] >> cfg.tri.turntable_center[0];
            if (!tn["turntable_center_y"].empty()) tn["turntable_center_y"] >> cfg.tri.turntable_center[1];
            if (!tn["turntable_center_z"].empty()) tn["turntable_center_z"] >> cfg.tri.turntable_center[2];
            if (!tn["object_radius_mm"].empty())   tn["object_radius_mm"]   >> cfg.tri.object_radius_mm;
        }

        cv::FileNode an = fs["app"];
        if (!an.empty())
        {
            if (!an["proj_w"].empty())             an["proj_w"]             >> cfg.app.proj_w;
            if (!an["proj_h"].empty())             an["proj_h"]             >> cfg.app.proj_h;
            if (!an["camera_url"].empty())         an["camera_url"]         >> cfg.app.camera_url;
            if (!an["objectDataRoot"].empty())     an["objectDataRoot"]     >> cfg.app.objectDataRoot;
            if (!an["cameraCalibFile"].empty())    an["cameraCalibFile"]    >> cfg.app.cameraCalibFile;
            if (!an["projectorCalibFile"].empty()) an["projectorCalibFile"] >> cfg.app.projectorCalibFile;
        }

        cv::FileNode pn = fs["projectorCalib"];
        if (!pn.empty())
        {
            if (!pn["previewW"].empty())        pn["previewW"]        >> cfg.projCalib.previewW;
            if (!pn["previewH"].empty())        pn["previewH"]        >> cfg.projCalib.previewH;
            if (!pn["board_cols"].empty())      pn["board_cols"]      >> cfg.projCalib.board_cols;
            if (!pn["board_rows"].empty())      pn["board_rows"]      >> cfg.projCalib.board_rows;
            if (!pn["squareSize_mm"].empty())   pn["squareSize_mm"]   >> cfg.projCalib.squareSize_mm;
            if (!pn["threshold"].empty())       pn["threshold"]       >> cfg.projCalib.threshold;
            if (!pn["patchRadius"].empty())     pn["patchRadius"]     >> cfg.projCalib.patchRadius;
            if (!pn["minDecodePoints"].empty()) pn["minDecodePoints"] >> cfg.projCalib.minDecodePoints;
            if (!pn["warmupFrames"].empty())    pn["warmupFrames"]    >> cfg.projCalib.warmupFrames;
            if (!pn["patternDelayMs"].empty())  pn["patternDelayMs"]  >> cfg.projCalib.patternDelayMs;
            if (!pn["minPoses"].empty())        pn["minPoses"]        >> cfg.projCalib.minPoses;
        }

        cv::FileNode pen = fs["poseEstimation"];
        if (!pen.empty())
        {
            if (!pen["pose_coarse_angle_step_deg"].empty()) pen["pose_coarse_angle_step_deg"] >> cfg.poseEst.pose_coarse_angle_step_deg;
            if (!pen["pose_min_live_points"].empty())       pen["pose_min_live_points"]       >> cfg.poseEst.pose_min_live_points;
            if (!pen["pose_max_accept_fitness_mm"].empty()) pen["pose_max_accept_fitness_mm"] >> cfg.poseEst.pose_max_accept_fitness_mm;
            if (!pen["modelRoot"].empty())                  pen["modelRoot"]                  >> cfg.poseEst.modelRoot;
        }

        fs.release();
        std::cout << "[CONFIG] Da doc " << path << "\n";
        return true;
    }

    // In ra toàn bộ config hiện tại (debug nhanh khi chạy chương trình,
    // để biết chắc chương trình đang dùng giá trị nào)
    void print() const
    {
        std::cout << "\n========== APP CONFIG ==========\n";
        std::cout << "[decode] bits=" << decode.bits
                   << " threshold=" << decode.threshold
                   << " adaptive=" << decode.useAdaptiveThreshold
                   << " maxLg=" << decode.maxLg
                   << " whiteIdx=" << decode.whiteIndex
                   << " blackIdx=" << decode.blackIndex
                   << " hStart=" << decode.horizontalStart
                   << " vStart=" << decode.verticalStart << "\n";

        std::cout << "[filter] reproj_max_err_px=" << filter.reproj_max_err_px
                   << " spatial_max_jump=" << filter.spatial_max_jump
                   << " iqr_scale=" << filter.iqr_scale << "\n";

        std::cout << "[triangulator] z=[" << tri.z_min << ", " << tri.z_max << "]"
                   << " x=[" << tri.x_min << ", " << tri.x_max << "]"
                   << " y=[" << tri.y_min << ", " << tri.y_max << "]"
                   << " icp_iter=" << tri.icp_iter
                   << " icp_dist_mm=" << tri.icp_dist_mm
                   << " icp_knn_k=" << tri.icp_knn_k
                   << " icp_normal_max_deg=" << tri.icp_normal_max_deg
                   << " icp_trim_ratio=" << tri.icp_trim_ratio
                   << " icp_conv_mm=" << tri.icp_conv_mm
                   << " icp_reg_ty_weight=" << tri.icp_reg_ty_weight
                   << " icp_reg_axis=(" << tri.icp_reg_axis[0] << "," << tri.icp_reg_axis[1]
                   << "," << tri.icp_reg_axis[2] << ")"
                   << " icp_loop_closure_enable=" << tri.icp_loop_closure_enable
                   << " icp_global_refine_iters=" << tri.icp_global_refine_iters
                   << " turntable_poses=" << tri.turntable_poses
                   << " turntable_step_deg=" << tri.turntable_step_deg
                   << " turntable_sign=" << tri.turntable_sign
                   << " axis=(" << tri.turntable_axis[0] << ","
                                 << tri.turntable_axis[1] << ","
                                 << tri.turntable_axis[2] << ")"
                   << " center=(" << tri.turntable_center[0] << ","
                                   << tri.turntable_center[1] << ","
                                   << tri.turntable_center[2] << ")"
                   << " object_radius_mm=" << tri.object_radius_mm << "\n";

        std::cout << "[app] proj=" << app.proj_w << "x" << app.proj_h
                   << " camera_url=" << app.camera_url
                   << " objectDataRoot=" << app.objectDataRoot
                   << " cameraCalibFile=" << app.cameraCalibFile
                   << " projectorCalibFile=" << app.projectorCalibFile << "\n";

        std::cout << "[projectorCalib] preview=" << projCalib.previewW << "x" << projCalib.previewH
                   << " board=" << projCalib.board_cols << "x" << projCalib.board_rows
                   << " squareSize_mm=" << projCalib.squareSize_mm
                   << " threshold=" << projCalib.threshold
                   << " patchRadius=" << projCalib.patchRadius
                   << " minDecodePoints=" << projCalib.minDecodePoints
                   << " warmupFrames=" << projCalib.warmupFrames
                   << " patternDelayMs=" << projCalib.patternDelayMs
                   << " minPoses=" << projCalib.minPoses << "\n";

        std::cout << "[poseEstimation] angle_step_deg=" << poseEst.pose_coarse_angle_step_deg
                   << " min_live_points=" << poseEst.pose_min_live_points
                   << " max_accept_fitness_mm=" << poseEst.pose_max_accept_fitness_mm
                   << " modelRoot=" << poseEst.modelRoot << "\n";
        std::cout << "=================================\n\n";
    }
};