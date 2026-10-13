#include "multipose_triangulator.h"
#include <numeric>   // std::iota — dùng trong subsampleClouds()
#include <algorithm> // std::shuffle-style partial swap — dùng trong subsampleClouds()

// ================================================================
//  Constructor
// ================================================================
MultiPoseTriangulator::MultiPoseTriangulator(
    const Mat& Kc,  const Mat& distC,
    const Mat& Kp,  const Mat& distP,
    const Mat& R,   const Mat& T,
    const TriangulatorConfig& triCfg,
    const DecodeConfig& decodeCfg,
    const FilterConfig& filterCfg)
    : Kc_(Kc.clone()), distC_(distC.clone()),
      Kp_(Kp.clone()), distP_(distP.clone()),
      R_(R.clone()),   T_(T.clone()),
      cfg(triCfg),
      decodeCfg_(decodeCfg), filterCfg_(filterCfg)
{
    cout << "[MultiPose] init OK (z=[" << cfg.z_min << ", " << cfg.z_max << "])\n";
}
 
// ================================================================
//  savePLY  – delegate sang GraycodeTriangulator tạm thời. const vì chỉ
//  đọc member (Kc_...cfg), không sửa state — cho phép gọi từ exportNoICP()
//  và các hàm const khác.
// ================================================================
void MultiPoseTriangulator::savePLY(
    const vector<Point3D>& cloud, const string& filename) const
{
    GraycodeTriangulator tri(Kc_, distC_, Kp_, distP_, R_, T_, cfg);
    tri.savePLY(cloud, filename);
}
 
// ================================================================
//  loadPoseImages
// ================================================================
bool MultiPoseTriangulator::loadPoseImages(
    const string& pose_path,
    int proj_w, int proj_h,
    vector<Mat>& images) const
{
    const int BITS          = (int)ceil(log2(max(proj_w, proj_h)));
    const int pattern_count = BITS * 4;
 
    images.clear();
 
    for (int i = 0; i < pattern_count; i++)
    {
        string f = pose_path + "/pattern_" + to_string(i) + ".png";
        Mat img  = imread(f, IMREAD_GRAYSCALE);
        if (img.empty())
        {
            cout << "[ERROR] Missing " << f << "\n";
            return false;
        }
        images.push_back(img);
    }
 
    Mat white = imread(pose_path + "/white.png", IMREAD_GRAYSCALE);
    if (white.empty()) { cout << "[ERROR] Missing white.png in " << pose_path << "\n"; return false; }
    images.push_back(white);
 
    Mat black = imread(pose_path + "/black.png", IMREAD_GRAYSCALE);
    if (black.empty()) { cout << "[ERROR] Missing black.png in " << pose_path << "\n"; return false; }
    images.push_back(black);
 
    return true;
}
 
// ================================================================
//  Tìm ma trận xoay 4×4 (CV_64F) quanh trục (ax,ay,az) với góc angle_rad
//  makeRotAxis  –  Rodrigues: xoay angle_rad quanh trục (ax,ay,az)
//  Trả về ma trận 4×4 CV_64F (translation = 0)
// ================================================================
Mat MultiPoseTriangulator::makeRotAxis(
    float ax, float ay, float az, double angle_rad) const
{
    double len = sqrt((double)ax*ax + (double)ay*ay + (double)az*az); // vector trục quay
    if (len < 1e-9) return Mat::eye(4, 4, CV_64F);
    double ux = ax/len, uy = ay/len, uz = az/len; // chuẩn hóa
 
    double c = cos(angle_rad), s = sin(angle_rad), t = 1.0 - c;
 
    // Ma trận xoay 3×3 (Rodrigues) rồi ghép vào 4×4
    Mat R4 = Mat::eye(4, 4, CV_64F);
    R4.at<double>(0,0) = t*ux*ux + c;
    R4.at<double>(0,1) = t*ux*uy - s*uz;
    R4.at<double>(0,2) = t*ux*uz + s*uy;
 
    R4.at<double>(1,0) = t*ux*uy + s*uz;
    R4.at<double>(1,1) = t*uy*uy + c;
    R4.at<double>(1,2) = t*uy*uz - s*ux;
 
    R4.at<double>(2,0) = t*ux*uz - s*uy;
    R4.at<double>(2,1) = t*uy*uz + s*ux;
    R4.at<double>(2,2) = t*uz*uz + c;
 
    return R4;
}
 
// ================================================================
//  computeNormals  –  finite-difference normal trên lưới (cam_u,cam_v).
//  width/height = kích thước ảnh camera (không phải projector).
// ================================================================
vector<Vec3f> MultiPoseTriangulator::computeNormals(
    const vector<Point3D>& cloud,
    int width, int height) const
{
    vector<int> idx_map(width * height, -1); 
    for (int i = 0; i < (int)cloud.size(); i++) // lưu chỉ số point cloud theo pixel (cam_u,cam_v)
        idx_map[cloud[i].cam_v * width + cloud[i].cam_u] = i;

    auto at = [&](int u, int v) -> const Point3D* // duyệt theo pixel (cam_u,cam_v) trả về con trỏ Point3D trong cloud hoặc nullptr
    {
        if (u < 0 || v < 0 || u >= width || v >= height)
            return nullptr;
        int idx = idx_map[v * width + u];
        return (idx < 0) ? nullptr : &cloud[idx];
    };

    vector<Vec3f> normals(cloud.size(), Vec3f(0.0f, 0.0f, 0.0f));

    // Tính normal bằng finite-difference giữa các hàng xóm liền kề trong lưới (cam_u,cam_v)
    for (int i = 0; i < (int)cloud.size(); i++)
    {
        const Point3D& c = cloud[i];

        const Point3D* pu0 = at(c.cam_u - 1, c.cam_v);
        const Point3D* pu1 = at(c.cam_u + 1, c.cam_v);
        const Point3D* pv0 = at(c.cam_u, c.cam_v - 1);
        const Point3D* pv1 = at(c.cam_u, c.cam_v + 1);

        Vec3f tu, tv;
        bool okU = true, okV = true;

        if (pu0 && pu1) tu = Vec3f(pu1->x - pu0->x, pu1->y - pu0->y, pu1->z - pu0->z);
        else if (pu1)   tu = Vec3f(pu1->x - c.x,     pu1->y - c.y,     pu1->z - c.z);
        else if (pu0)   tu = Vec3f(c.x - pu0->x,     c.y - pu0->y,     c.z - pu0->z);
        else            okU = false;

        if (pv0 && pv1) tv = Vec3f(pv1->x - pv0->x, pv1->y - pv0->y, pv1->z - pv0->z);
        else if (pv1)   tv = Vec3f(pv1->x - c.x,     pv1->y - c.y,     pv1->z - c.z);
        else if (pv0)   tv = Vec3f(c.x - pv0->x,     c.y - pv0->y,     c.z - pv0->z);
        else            okV = false;

        if (!okU || !okV)
            continue;   // giữ Vec3f(0,0,0) = invalid

        Vec3f n = tu.cross(tv);
        double len = norm(n);
        if (len < 1e-9)
            continue;   // 3 điểm gần thẳng hàng / suy biến

        n *= (float)(1.0 / len);

        // Ép normal quay về phía camera: camera ở gốc toạ độ (P1 = [I|0]
        // trong triangulatePoint), nên vector từ camera đến điểm chính là
        // toạ độ điểm đó — dùng làm view direction để chọn chiều nhất quán.
        Vec3f viewDir(c.x, c.y, c.z);
        if (n.dot(viewDir) > 0) // đảo chiều so cho normal hướng vào camera
            n = -n;

        normals[i] = n;
    }

    return normals;
}

// ================================================================
//  icpAlign  –  point-to-plane ICP (linearized, Low 2004) với initial
//  guess T_init: ước lượng ban đầu từ góc bàn xoay (4×4 CV_64F).
//  Mỗi vòng lặp giải [rx,ry,rz,tx,ty,tz] từ hệ:
//      [ (p×n) | n ] · [r;t] = -(p-q)·n
//  với p = điểm src hiện tại (cur), q = điểm ref khớp gần nhất,
//  n = normal của ref TẠI q (ref_normals) — hệ phương trình không cần
//  normal của src, nhưng src_normals VẪN cần để lọc correspondence theo
//  góc (icp_normal_max_deg) trước khi đưa vào hệ này — xem bước 1.
//  Trả về T4x4 tổng (CV_64F) sao cho T4x4 * src ≈ ref.
// ================================================================
Mat MultiPoseTriangulator::icpAlign(
    const vector<Point3D>& src,
    const vector<Vec3f>& src_normals,
    const vector<Point3D>& ref,
    const vector<Vec3f>& ref_normals,
    const Mat& T_init) const
{
    // chuẩn bị KD-tree cho nearest-neighbor search (FLANN)
    Mat refMat((int)ref.size(), 3, CV_32F);
    for (int i = 0; i < (int)ref.size(); i++)
    {
        refMat.at<float>(i,0) = ref[i].x;
        refMat.at<float>(i,1) = ref[i].y;
        refMat.at<float>(i,2) = ref[i].z;
    }
    cv::flann::KDTreeIndexParams idx_params(4);
    cv::flann::Index kd(refMat, idx_params);
 
    // Áp initial guess trước khi bắt đầu iterate — cur_normals đi song song
    // với cur, xoay CÙNG T4x4 mỗi vòng lặp (chỉ phần rotation, xem
    // rotateNormals()) để so góc với ref_normals ở bước 1 bên dưới.
    Mat T_acc = T_init.clone();
    vector<Point3D> cur         = applyTransform(src, T_init);
    vector<Vec3f>   cur_normals = rotateNormals(src_normals, T_init);

    // K ứng viên nearest-neighbor lấy từ config, kẹp trong [1, ref.size()]
    // để tránh flann trả kết quả không hợp lệ khi ref có ít điểm hơn K.
    const int   K          = max(1, min(cfg.icp_knn_k, (int)ref.size()));
    const float cos_thresh = (float)cos(cfg.icp_normal_max_deg * CV_PI / 180.0);

    // Trục regularization đã chuẩn hoá (đơn vị) — dùng làm hướng cho
    // regularization ở bước 2 (ép dịch chuyển DỌC TRỤC về gần 0). Đọc từ
    // icp_reg_axis — TÁCH RIÊNG khỏi turntable_axis (xem comment field
    // trong app_config.h) để đổi cái này không vô tình đổi luôn T_init.
    // Nếu axis cấu hình sai (độ dài ~0) thì tắt regularization luôn (coi
    // như icp_reg_ty_weight=0) thay vì chia cho 0.
    Vec3f reg_axis(cfg.icp_reg_axis[0], cfg.icp_reg_axis[1], cfg.icp_reg_axis[2]);
    const float reg_axis_len = (float)norm(reg_axis);
    const float reg_weight   = (reg_axis_len > 1e-6f) ? cfg.icp_reg_ty_weight : 0.0f;
    if (reg_axis_len > 1e-6f) reg_axis *= (1.0f / reg_axis_len);

    for (int iter = 0; iter < cfg.icp_iter; iter++)
    {
        // ---- 1. Nearest-neighbor (K ứng viên/điểm, K=cfg.icp_knn_k) —
        // loại theo trần an toàn tuyệt đối (icp_dist_mm), normal tồn tại
        // (cả source lẫn ref), và góc normal source/ref (icp_normal_max_deg)
        // — trong số còn lại, chọn ứng viên GẦN NHẤT (K đã sort tăng dần) ----
        struct Cand { int si, ri; float dist2; float residual; };
        vector<Cand> cands;
        const float thresh2 = cfg.icp_dist_mm * cfg.icp_dist_mm;

        for (int i = 0; i < (int)cur.size(); i++)
        {
            // Source point không có normal hợp lệ (gần biên/lỗ trong lưới
            // của chính pose src) thì không đủ thông tin để đánh giá
            // normal-consistency — bỏ qua điểm này luôn, không tìm NN nữa.
            const Vec3f& n_src = cur_normals[i];
            if (n_src.dot(n_src) < 0.5f)
                continue;

            // tìm K nearest neighbor trong ref cho cur[i]
            Mat q = (Mat_<float>(1,3) << cur[i].x, cur[i].y, cur[i].z);
            vector<int>   idx(K);
            vector<float> dist(K);
            kd.knnSearch(q, idx, dist, K, cv::flann::SearchParams(32));

            int   chosen_ri    = -1;
            float chosen_dist2 = 0.0f;

            for (int k = 0; k < K; k++)
            {
                if (dist[k] >= thresh2)   // đã sort tăng dần -> các k sau cũng vượt trần
                    break;

                // ref_normals[idx[k]] == (0,0,0): computeNormals() không đủ
                // hàng xóm để tính tại điểm đó (bị filter/ở biên) — thử ứng viên kế.
                const Vec3f& n_ref = ref_normals[idx[k]];
                if (n_ref.dot(n_ref) < 0.5f)
                    continue;

                // Normal consistency — dot CÓ DẤU (không lấy trị tuyệt đối):
                // correspondence đúng (cùng 1 bề mặt vật lý) phải có normal
                // cùng hướng; khớp nhầm sang mặt khác cho dot thấp/âm.
                if (n_src.dot(n_ref) < cos_thresh)
                    continue;

                chosen_ri    = idx[k];
                chosen_dist2 = dist[k];
                break;   // gần nhất trong số ứng viên hợp lệ (K đã sort tăng dần)
            }

            if (chosen_ri < 0)
                continue;   // không ứng viên nào trong K đạt cả 3 điều kiện

            Vec3f p(cur[i].x, cur[i].y, cur[i].z);
            Vec3f qp(ref[chosen_ri].x, ref[chosen_ri].y, ref[chosen_ri].z);
            float residual = (float)fabs(ref_normals[chosen_ri].dot(p - qp));

            cands.push_back({i, chosen_ri, chosen_dist2, residual});
        }

        if ((int)cands.size() < 6)
        {
            cout << "[ICP] iter " << iter << " : too few pairs ("
                 << cands.size() << "), stop\n";
            break;
        }

        // ---- 1b. Trim theo tỷ lệ: giữ icp_trim_ratio phần RESIDUAL POINT-
        // TO-PLANE (|n·(p-q)|) nhỏ nhất trong số candidate đã qua bước 1 ở
        // trên — khớp đúng đại lượng ICP point-to-plane thực sự đang tối ưu
        // (đổi từ dist2 Point-to-Point trước đây), tự thích nghi theo
        // residual thực tế của chính vòng lặp này. Luôn giữ tối thiểu 6 cặp.
        sort(cands.begin(), cands.end(),
             [](const Cand& a, const Cand& b) { return a.residual < b.residual; });

        int keep = max(6, (int)(cfg.icp_trim_ratio * (float)cands.size()));
        keep = min(keep, (int)cands.size());

        vector<pair<int,int>> pairs;
        pairs.reserve(keep);
        for (int k = 0; k < keep; k++)
            pairs.emplace_back(cands[k].si, cands[k].ri);

        // ---- 2. Point-to-plane linearized: A^T A x = A^T b ----
        // +1 hàng cuối dành cho regularization ty (xem sau vòng lặp dưới).
        Mat A = Mat::zeros((int)pairs.size() + 1, 6, CV_64F);
        Mat b = Mat::zeros((int)pairs.size() + 1, 1, CV_64F);
        
        for (int k = 0; k < (int)pairs.size(); k++)
        {
            int si = pairs[k].first;
            int ri = pairs[k].second;

            Vec3f p(cur[si].x, cur[si].y, cur[si].z);
            Vec3f q(ref[ri].x, ref[ri].y, ref[ri].z);
            Vec3f n = ref_normals[ri];

            Vec3f pxn = p.cross(n);

            A.at<double>(k,0) = pxn[0];
            A.at<double>(k,1) = pxn[1];
            A.at<double>(k,2) = pxn[2];
            A.at<double>(k,3) = n[0];
            A.at<double>(k,4) = n[1];
            A.at<double>(k,5) = n[2];

            b.at<double>(k,0) = -(double)(p - q).dot(n);
        }

        // ---- 2b. Regularization: 1 "hàng ảo" ép thành phần dịch chuyển
        // DỌC TRỤC QUAY (reg_axis, chuẩn hoá ở trên) về gần 0 — tương
        // đương thêm số hạng (reg_weight * (t·reg_axis))^2 vào tổng bình
        // phương residual đang tối thiểu hoá. reg_weight=0 (config tắt,
        // hoặc axis độ dài ~0) thì hàng này toàn 0, không ảnh hưởng gì —
        // giữ nguyên hành vi cũ. Không đụng gì tới 3 cột đầu (rx,ry,rz).
        int reg_row = (int)pairs.size();
        A.at<double>(reg_row,3) = reg_weight * reg_axis[0];
        A.at<double>(reg_row,4) = reg_weight * reg_axis[1];
        A.at<double>(reg_row,5) = reg_weight * reg_axis[2];
        b.at<double>(reg_row,0) = 0.0;   // mục tiêu: dịch chuyển dọc trục = 0

        // DECOMP_SVD: vẫn ra nghiệm least-squares hợp lệ (pseudo-inverse)
        // nếu A^T A gần suy biến (vd correspondence dồn hết vào 1 mặt
        // phẳng ở vài iteration đầu) — an toàn hơn DECOMP_CHOLESKY, vốn
        // cần ma trận positive-definite thật sự.
        Mat x;
        if (!solve(A.t() * A, A.t() * b, x, DECOMP_SVD))
        {
            cout << "[ICP] iter " << iter << " : linear solve failed, stop\n";
            break;
        }

        double rx = x.at<double>(0), ry = x.at<double>(1), rz = x.at<double>(2);
        double tx = x.at<double>(3), ty = x.at<double>(4), tz = x.at<double>(5);

        // ---- 3. (r,t) -> Tstep. Rodrigues cho phần xoay (thay vì I+[r]_x
        //         thô) để R_step luôn là ma trận trực giao hợp lệ. ----
        Mat rvec = (Mat_<double>(3,1) << rx, ry, rz);
        Mat R_step;
        Rodrigues(rvec, R_step);

        Mat Tstep = Mat::eye(4, 4, CV_64F);
        R_step.copyTo(Tstep(Rect(0,0,3,3)));
        Tstep.at<double>(0,3) = tx;
        Tstep.at<double>(1,3) = ty;
        Tstep.at<double>(2,3) = tz;
 
        T_acc       = Tstep * T_acc;
        cur         = applyTransform(cur, Tstep);
        cur_normals = rotateNormals(cur_normals, Tstep);
 
        // ---- 4. Convergence ----
        double dt = sqrt(tx*tx + ty*ty + tz*tz);
        if (dt < cfg.icp_conv_mm)
        {
            cout << "[ICP] converged iter=" << iter << " dt=" << dt << "\n";
            break;
        }
    }
 
    return T_acc;
}
 
// ================================================================
//  applyTransform
// ================================================================
vector<Point3D> MultiPoseTriangulator::applyTransform(
    const vector<Point3D>& cloud, const Mat& T4x4) const
{
    vector<Point3D> out = cloud;
    for (auto& p : out)
    {
        Mat v = (Mat_<double>(4,1) << p.x, p.y, p.z, 1.0);
        Mat r = T4x4 * v;
        p.x = (float)r.at<double>(0);
        p.y = (float)r.at<double>(1);
        p.z = (float)r.at<double>(2);
    }
    return out;
}

// ================================================================
//  rotateNormals  –  áp phần xoay (3×3 top-left) của T4x4 lên normal,
//  KHÔNG cộng translation. Vec3f(0,0,0) (invalid) xoay ra vẫn (0,0,0).
// ================================================================
vector<Vec3f> MultiPoseTriangulator::rotateNormals(
    const vector<Vec3f>& normals, const Mat& T4x4) const
{
    Mat R = T4x4(Rect(0, 0, 3, 3));

    vector<Vec3f> out(normals.size());
    for (int i = 0; i < (int)normals.size(); i++)
    {
        Mat v = (Mat_<double>(3,1) << normals[i][0], normals[i][1], normals[i][2]);
        Mat r = R * v;
        out[i] = Vec3f((float)r.at<double>(0), (float)r.at<double>(1), (float)r.at<double>(2));
    }
    return out;
}
 
// ================================================================
//  reconstructAllPoses  –  reconstruct RAW (chưa align) cloud+normal cho
//  mọi pose. Dùng chung bởi reconstructFolder() và autoCalibrateTurntable().
// ================================================================
bool MultiPoseTriangulator::reconstructAllPoses(
    const string& folder,
    const vector<string>& poses,
    int proj_w, int proj_h,
    vector<vector<Point3D>>& clouds,
    vector<vector<Vec3f>>& normals) const
{
    clouds.clear();
    normals.clear();

    vector<string> pose_dirs;
    for (const auto& name : poses)
        pose_dirs.push_back(folder + "/" + name);

    if (pose_dirs.empty())
    {
        cout << "[ERROR] No pose directories in " << folder << "\n";
        return false;
    }
    cout << "[INFO] " << pose_dirs.size() << " poses found\n";

    for (int pid = 0; pid < (int)pose_dirs.size(); pid++)
    {
        cout << "\n--- Pose " << pid << " : " << pose_dirs[pid] << " ---\n";

        vector<Mat> images;
        if (!loadPoseImages(pose_dirs[pid], proj_w, proj_h, images))
        {
            cout << "[WARN] Skip\n";
            continue;
        }

        GraycodeTriangulator tri(Kc_, distC_, Kp_, distP_, R_, T_, cfg);
        // Truyền scale từ projector_calibration.json để remap proj coords
        // từ hệ Gray-code [0..proj_w/h] sang hệ preview [0..previewW/H]
        // trước khi dùng Kp/dist_p undistort. Nếu thiếu dòng này,
        // proj_u/v sẽ ở hệ graycode (0..1024) trong khi Kp calib ở (0..500)
        // → triangulation cho ra điểm 3D sai hoàn toàn.
        tri.setProjectorScale(proj_scale_x_, proj_scale_y_);

        vector<Point3D> cloud = tri.reconstruct(images, proj_w, proj_h,
                                                 decodeCfg_, filterCfg_);
        if (cloud.empty()) { cout << "[WARN] Empty cloud, skip\n"; continue; }

        cout << "[INFO] Pose " << pid << " : " << cloud.size() << " pts\n";

        // images[0] = pattern_0, cùng kích thước ảnh camera với mọi pattern/
        // white/black khác trong pose này — dùng làm width/height cho lưới
        // (cam_u,cam_v) khi tính normal.
        vector<Vec3f> normal = computeNormals(cloud, images[0].cols, images[0].rows);

        clouds.push_back(cloud);
        normals.push_back(normal);
    }

    if (clouds.empty()) { cout << "[ERROR] No valid poses\n"; return false; }
    return true;
}

// ================================================================
//  reconstructFolder  – entry point chính
// ================================================================
vector<Point3D> MultiPoseTriangulator::reconstructFolder(
    const string& folder,
    const vector<string>& poses,
    int proj_w,
    int proj_h,
    vector<Vec3f>* out_normals)
{
    cout << "\n====== MULTI-POSE RECONSTRUCTION ======\n";
 
    vector<vector<Point3D>> clouds;
    // normals[pid] song song với clouds[pid], tính trên lưới (cam_u,cam_v)
    // CỦA POSE ĐÓ (hệ local, trước khi align) — dùng cho ICP point-to-plane.
    vector<vector<Vec3f>> normals;

    if (!reconstructAllPoses(folder, poses, proj_w, proj_h, clouds, normals))
        return {};
 
    // ================================================================
    //  ICP ALIGNMENT  (Tier 1 — accumulated-cloud reference)
    //  - Pose i align vào accum_cloud = TOÀN BỘ pose 0..i-1 đã align nối
    //    lại — không chỉ riêng pose i-1 — để giảm sai số cộng dồn khi
    //    overlap với RIÊNG pose liền trước yếu (vd cạnh/góc vật hình hộp).
    //  - Initial guess = xoay theo góc bàn xoay đã calibrate (turntable_
    //    center/step_deg/sign trong config.json), KHÔNG phụ thuộc kết quả
    //    ICP của pose trước.
    //  - Tâm xoay = turntable_center (đo/calibrate MỘT LẦN qua
    //    autoCalibrateTurntable(), không phải centroid tự tính từ cloud).
    //  - accum_cloud tới cuối vòng lặp CHÍNH LÀ point cloud hoàn chỉnh;
    //    bước 3 chỉ lặp lại đúng nội dung đó để trả về (giữ nguyên logic
    //    cũ, không phụ thuộc tính đúng của accum_cloud).
    // ================================================================

    cout << "\n====== ICP ALIGNMENT ======\n";
 
    int   N      = (int)clouds.size();
    float step_d = (cfg.turntable_step_deg > 0.0f)
                       ? cfg.turntable_step_deg
                       : 360.0f / (float)cfg.turntable_poses;
    double step_rad = step_d * CV_PI / 180.0;
 
    cout << "[INFO] Turntable step = " << step_d << " deg/pose (sign=" << cfg.turntable_sign << ")\n";
    cout << "[INFO] Axis = ("
         << cfg.turntable_axis[0] << ", "
         << cfg.turntable_axis[1] << ", "
         << cfg.turntable_axis[2] << ")\n";
 
    // 1. TÂM XOAY — điểm cố định trên trục quay của BỘ GÁ (camera + bàn
    // xoay), đo/hiệu chỉnh MỘT LẦN và ghi vào config.json (turntable_center),
    // dùng chung cho mọi vật đặt lên bàn xoay.
    double cx = cfg.turntable_center[0];
    double cy = cfg.turntable_center[1];
    double cz = cfg.turntable_center[2];

    cout << "[INFO] Rotation center (turntable_center, config.json) = ("
         << cx << ", " << cy << ", " << cz << ") mm\n";
 
    Mat T_to   = Mat::eye(4, 4, CV_64F);
    T_to.at<double>(0,3) = -cx;
    T_to.at<double>(1,3) = -cy;
    T_to.at<double>(2,3) = -cz;
 
    Mat T_back = Mat::eye(4, 4, CV_64F);
    T_back.at<double>(0,3) = cx;
    T_back.at<double>(1,3) = cy;
    T_back.at<double>(2,3) = cz;
 
    // Lưu point cloud + normal đã align vào hệ toạ độ toàn cục (hệ pose_0).
    // aligned_clouds/aligned_normals: vẫn giữ riêng từng pose (dùng cho
    // bước merge cuối, không đổi). accum_cloud/accum_normals: nối DẦN sau
    // mỗi pose — đây mới là ref truyền vào icpAlign() của pose kế tiếp
    // (Tier 1, xem vòng lặp bên dưới).
    vector<vector<Point3D>> aligned_clouds(N);
    vector<vector<Vec3f>>   aligned_normals(N);
    aligned_clouds[0]  = clouds[0];
    aligned_normals[0] = normals[0];

    vector<Point3D> accum_cloud   = clouds[0];
    vector<Vec3f>   accum_normals = normals[0];
 
    // 2. GHÉP NỐI TIẾP (SEQUENTIAL ICP)
    for (int i = 1; i < N; i++)
    {
        // Initial guess: Đưa pose_i (local) về hệ toạ độ toàn cục của pose_0.
        // turntable_sign cho phép đảo chiều quay qua config.json (xem
        // autoCalibrateTurntable()) mà không cần đổi dấu trực tiếp trong code.
        double angle    = cfg.turntable_sign * -(double)i * step_rad;
        Mat R_guess     = makeRotAxis(cfg.turntable_axis[0],
                                      cfg.turntable_axis[1],
                                      cfg.turntable_axis[2], angle);
        Mat T_init = T_back * R_guess * T_to;
 
        // QUAN TRỌNG (Tier 1): ref là TOÀN BỘ pose 0..i-1 đã align, nối
        // trong accum_cloud/accum_normals — không chỉ riêng pose i-1 —
        // đỡ được trường hợp overlap với RIÊNG pose liền trước yếu (mặt/
        // cạnh vật hình hộp) vì vẫn còn overlap tốt với pose xa hơn.
        const vector<Point3D>& ref_cloud   = accum_cloud;
        const vector<Vec3f>&   ref_normal  = accum_normals;
 
        cout << "[ICP] pose " << i
             << " (" << clouds[i].size() << " pts)"
             << " ==> ref accumulated pose 0.." << i-1 << " (" << ref_cloud.size() << " pts)"
             << " | init angle = " << (int)(cfg.turntable_sign * -i * step_d) << " deg\n";
 
        // ICP (point-to-plane) sẽ tinh chỉnh T_init để pose_i khớp khít accum_cloud
        Mat T4x4 = icpAlign(clouds[i], normals[i], ref_cloud, ref_normal, T_init);

        // Lưu pose_i đã align vào mảng toàn cục — normal xoay theo CÙNG T4x4
        // (chỉ phần rotation, applyTransform/rotateNormals tự xử lý đúng phần đó)
        aligned_clouds[i]  = applyTransform(clouds[i], T4x4);
        aligned_normals[i] = rotateNormals(normals[i], T4x4);

        // Nối pose_i vừa align xong vào accum_cloud — pose i+1 (nếu có) sẽ
        // thấy pose_i này trong ref của nó. Nối SAU khi align để pose i
        // không tự tham chiếu chính nó.
        accum_cloud.insert(accum_cloud.end(), aligned_clouds[i].begin(), aligned_clouds[i].end());
        accum_normals.insert(accum_normals.end(), aligned_normals[i].begin(), aligned_normals[i].end());
 
        cout << "[ICP] pose " << i << " aligned OK\n";
    }

    // ================================================================
    // 3. LOOP CLOSURE: pose cuối -> pose 0 (chỉ chạy khi bật trong config
    //    và đủ ít nhất 3 pose — dưới 3 thì "pose cuối" đã kề pose 0 sẵn
    //    trong vòng lặp sequential, khép vòng riêng không còn ý nghĩa).
    // ================================================================
    if (N >= 3 && cfg.icp_loop_closure_enable)
    {
        cout << "\n====== LOOP CLOSURE ======\n";
        Mat T_loop = icpAlign(
            aligned_clouds[N - 1],
            aligned_normals[N - 1],
            aligned_clouds[0],
            aligned_normals[0],
            Mat::eye(4, 4, CV_64F)
        );

        // Đo độ lớn correction TRƯỚC khi áp — biết khe hở lúc khép vòng to
        // cỡ nào, và để so sánh với correction mỗi vòng GLOBAL REFINEMENT
        // bên dưới (còn lớn tương đương thì có thể cần thêm vòng).
        double lt_x = T_loop.at<double>(0,3), lt_y = T_loop.at<double>(1,3), lt_z = T_loop.at<double>(2,3);
        double loop_dt = sqrt(lt_x*lt_x + lt_y*lt_y + lt_z*lt_z);
        Mat rvec_loop;
        Rodrigues(T_loop(Rect(0,0,3,3)), rvec_loop);
        double loop_dr_deg = norm(rvec_loop) * 180.0 / CV_PI;
        cout << "[LOOP] khe ho luc khep: dich=" << loop_dt << " mm (x=" << lt_x
             << " y=" << lt_y << " z=" << lt_z << "), xoay=" << loop_dr_deg << " deg\n";

        aligned_clouds[N - 1] = applyTransform(
            aligned_clouds[N - 1], T_loop
        );
        aligned_normals[N - 1] = rotateNormals(
            aligned_normals[N - 1], T_loop
        );
        cout << "[LOOP] pose " << N - 1 << " -> pose 0 corrected\n";
    }

    // ================================================================
    // 4. GLOBAL REFINEMENT
    //    Pose 0 cố định, mỗi pose được ICP với toàn bộ các pose còn lại.
    //    icp_global_refine_iters=0 -> bỏ qua hẳn bước này.
    // ================================================================
    for (int round = 0; round < cfg.icp_global_refine_iters; round++)
    {
        cout << "\n====== GLOBAL REFINEMENT "
             << (round + 1) << "/" << cfg.icp_global_refine_iters
             << " ======\n";
        // Snapshot để tất cả pose trong cùng một round
        // đều tham chiếu cùng một trạng thái trước khi cập nhật.
        vector<vector<Point3D>> ref_clouds = aligned_clouds;
        vector<vector<Vec3f>> ref_normals = aligned_normals;

        double max_dt = 0.0, max_dr_deg = 0.0;   // correction lớn nhất trong vòng này

        for (int i = 1; i < N; i++)
        {
            vector<Point3D> global_ref;
            vector<Vec3f> global_ref_normals;
            size_t total_pts = 0;
            for (int j = 0; j < N; j++)
                if (j != i)
                    total_pts += ref_clouds[j].size();
            global_ref.reserve(total_pts);
            global_ref_normals.reserve(total_pts);
            for (int j = 0; j < N; j++)
            {
                if (j == i)
                    continue;
                global_ref.insert(
                    global_ref.end(),
                    ref_clouds[j].begin(),
                    ref_clouds[j].end()
                );
                global_ref_normals.insert(
                    global_ref_normals.end(),
                    ref_normals[j].begin(),
                    ref_normals[j].end()
                );
            }
            // Tất cả cloud hiện tại đã ở global frame,
            // nên refinement bắt đầu từ Identity.
            Mat T_refine = icpAlign(
                ref_clouds[i],
                ref_normals[i],
                global_ref,
                global_ref_normals,
                Mat::eye(4, 4, CV_64F)
            );

            double rt_x = T_refine.at<double>(0,3), rt_y = T_refine.at<double>(1,3), rt_z = T_refine.at<double>(2,3);
            double dt = sqrt(rt_x*rt_x + rt_y*rt_y + rt_z*rt_z);
            Mat rvec_r;
            Rodrigues(T_refine(Rect(0,0,3,3)), rvec_r);
            double dr_deg = norm(rvec_r) * 180.0 / CV_PI;
            max_dt     = max(max_dt, dt);
            max_dr_deg = max(max_dr_deg, dr_deg);

            aligned_clouds[i] = applyTransform(
                ref_clouds[i], T_refine
            );
            aligned_normals[i] = rotateNormals(
                ref_normals[i], T_refine
            );
            cout << "[GLOBAL] pose " << i << " refined (dich=" << dt
                 << " mm, xoay=" << dr_deg << " deg)\n";
        }

        cout << "[GLOBAL] round " << (round + 1) << " max correction: dich="
             << max_dt << " mm, xoay=" << max_dr_deg << " deg"
             << (max_dt < cfg.icp_conv_mm ? "  (da on dinh)" : "  (co the can them vong)")
             << "\n";
    }

    // 5. GỘP (MERGE) TOÀN BỘ POINT CLOUD
    vector<Point3D> merged;
    for (auto& c : aligned_clouds)
        merged.insert(merged.end(), c.begin(), c.end());

    // Gộp normal THEO ĐÚNG THỨ TỰ trên (cùng vòng lặp pose 0..N-1) để
    // out_normals[i] tương ứng đúng merged[i] — dùng cho Hướng B build
    // reference model (xem pose_estimator.h). Không tính gì mới, chỉ
    // trả ra normal đã có sẵn (aligned_normals) mà trước đây bị bỏ đi.
    if (out_normals)
    {
        out_normals->clear();
        out_normals->reserve(merged.size());
        for (auto& n : aligned_normals)
            out_normals->insert(out_normals->end(), n.begin(), n.end());
    }
 
    cout << "\n[DONE] Final cloud: " << merged.size() << " pts\n";
    return merged;
}

// ================================================================
//  cloudToMat  –  Nx3 CV_32F Mat từ cloud, dùng để build cv::flann::Index.
// ================================================================
Mat MultiPoseTriangulator::cloudToMat(const vector<Point3D>& cloud) const
{
    Mat m((int)cloud.size(), 3, CV_32F);
    for (int i = 0; i < (int)cloud.size(); i++)
    {
        m.at<float>(i,0) = cloud[i].x;
        m.at<float>(i,1) = cloud[i].y;
        m.at<float>(i,2) = cloud[i].z;
    }
    return m;
}

// ================================================================
//  subsampleClouds  –  random subsample mỗi pose xuống tối đa maxPts điểm.
//  Seed CỐ ĐỊNH (12345) để kết quả autoCalibrateTurntable() lặp lại được
//  giữa các lần chạy trên cùng dữ liệu — không phải để bảo mật/ngẫu nhiên
//  thật, chỉ cần phân bố đều để đại diện cho cloud gốc.
// ================================================================
vector<vector<Point3D>> MultiPoseTriangulator::subsampleClouds(
    const vector<vector<Point3D>>& clouds, int maxPts) const
{
    cv::RNG rng(12345);
    vector<vector<Point3D>> out(clouds.size());

    for (size_t i = 0; i < clouds.size(); i++)
    {
        const auto& c = clouds[i];
        if ((int)c.size() <= maxPts) { out[i] = c; continue; }

        vector<int> idx(c.size());
        std::iota(idx.begin(), idx.end(), 0);

        // Partial Fisher-Yates: chỉ cần maxPts phần tử đầu ngẫu nhiên đều,
        // không cần shuffle toàn bộ mảng.
        for (int k = 0; k < maxPts; k++)
        {
            int j = k + rng.uniform(0, (int)idx.size() - k);
            std::swap(idx[k], idx[j]);
        }

        out[i].reserve(maxPts);
        for (int k = 0; k < maxPts; k++)
            out[i].push_back(c[idx[k]]);
    }
    return out;
}

// ================================================================
//  scoreCandidatePairwise  –  điểm số hình học thuần (không ICP) cho 1 bộ (cx,cy,cz,step_deg,sign). 
// Xoay các pose (không ICP) rồi xét pose_i với pose_i+1, tính khoảng cách nearest-neighbor giữa chúng, 
// lấy percentile-30 của khoảng cách đó, rồi trung bình qua mọi cặp pose liền kề. Càng thấp càng khớp tốt. 
// Dùng để autoCalibrateTurntable() tìm tham số bàn xoay tốt nhất.
// ================================================================
double MultiPoseTriangulator::scoreCandidatePairwise(
    const vector<vector<Point3D>>& clouds,
    double cx, double cy, double cz,
    float step_deg, int sign) const
{
    int N = (int)clouds.size();
    if (N < 2) return 1e18;

    double step_rad = step_deg * CV_PI / 180.0;

    Mat T_to = Mat::eye(4, 4, CV_64F);
    T_to.at<double>(0,3) = -cx; T_to.at<double>(1,3) = -cy; T_to.at<double>(2,3) = -cz;
    Mat T_back = Mat::eye(4, 4, CV_64F);
    T_back.at<double>(0,3) = cx; T_back.at<double>(1,3) = cy; T_back.at<double>(2,3) = cz;

    vector<vector<Point3D>> cur(N);
    cur[0] = clouds[0];
    for (int i = 1; i < N; i++)
    {
        double angle = sign * -(double)i * step_rad;
        Mat R_guess = makeRotAxis(cfg.turntable_axis[0], cfg.turntable_axis[1],
                                   cfg.turntable_axis[2], angle);
        Mat T = T_back * R_guess * T_to;
        cur[i] = applyTransform(clouds[i], T);
    }

    double total = 0.0;
    int count = 0;

    for (int i = 0; i + 1 < N; i++)
    {
        Mat m = cloudToMat(cur[i]);
        if (m.rows < 4) continue;   // KD-tree cần tối thiểu vài điểm

        cv::flann::Index kd(m, cv::flann::KDTreeIndexParams(4));

        vector<float> dists;
        dists.reserve(cur[i+1].size());
        for (auto& p : cur[i+1])
        {
            Mat q = (Mat_<float>(1,3) << p.x, p.y, p.z);
            vector<int> idx(1); vector<float> dist(1);
            kd.knnSearch(q, idx, dist, 1, cv::flann::SearchParams(32));
            dists.push_back(dist[0]);   // squared L2
        }
        if (dists.empty()) continue;

        sort(dists.begin(), dists.end());
        int p30 = max(0, (int)(0.3 * dists.size()) - 1);
        total += sqrt((double)dists[p30]);
        count++;
    }

    return (count > 0) ? (total / count) : 1e18;
}

// ================================================================
//  exportNoICP  – Áp transform "không ICP" (chỉ hình học: center+axis+step_deg+sign)
// lên từng pose rồi merge + lưu PLY
// ================================================================
void MultiPoseTriangulator::exportNoICP(
    const vector<vector<Point3D>>& clouds,
    double cx, double cy, double cz,
    float step_deg, int sign,
    const string& filename) const
{
    int N = (int)clouds.size();
    if (N == 0) { cout << "[WARN] exportNoICP: cloud rong\n"; return; }

    double step_rad = step_deg * CV_PI / 180.0;

    Mat T_to = Mat::eye(4, 4, CV_64F);
    T_to.at<double>(0,3) = -cx; T_to.at<double>(1,3) = -cy; T_to.at<double>(2,3) = -cz;
    Mat T_back = Mat::eye(4, 4, CV_64F);
    T_back.at<double>(0,3) = cx; T_back.at<double>(1,3) = cy; T_back.at<double>(2,3) = cz;

    vector<Point3D> merged = clouds[0];
    for (int i = 1; i < N; i++)
    {
        double angle = sign * -(double)i * step_rad;
        Mat R_guess = makeRotAxis(cfg.turntable_axis[0], cfg.turntable_axis[1],
                                   cfg.turntable_axis[2], angle);
        Mat T = T_back * R_guess * T_to;
        vector<Point3D> cur = applyTransform(clouds[i], T);
        merged.insert(merged.end(), cur.begin(), cur.end());
    }

    savePLY(merged, filename);
}

// ================================================================
//  autoCalibrateTurntable  –  xem giải thích đầy đủ trong .h.
//
//  Chỉ tìm cx, cz (KHÔNG tìm cy) + step_deg + sign. Lý do bỏ cy: với
//  T_init = T_back * R(axis,angle) * T_to, nếu axis=(0,1,0) thì dịch
//  điểm P0 dọc theo CHÍNH trục quay (tức đổi cy) không đổi kết quả biến đổi 
// ================================================================
TurntableCalibResult MultiPoseTriangulator::autoCalibrateTurntable(
    const string& folder,
    const vector<string>& poses,
    int proj_w,
    int proj_h)
{
    cout << "\n====== AUTO-CALIBRATE TURNTABLE (center x,z + step + sign) ======\n";

    vector<vector<Point3D>> clouds;
    vector<vector<Vec3f>> normals;   // không dùng ở đây, chỉ để tái dùng reconstructAllPoses
    if (!reconstructAllPoses(folder, poses, proj_w, proj_h, clouds, normals))
        return TurntableCalibResult{};

    return runAutoCalibSearch(clouds);
}

// ================================================================
//  runAutoCalibSearch  –  phần THUẬT TOÁN thuần của autoCalibrateTurntable,
//  nhận thẳng clouds (đã reconstruct) — tách riêng khỏi việc đọc ảnh để
//  test được bằng dữ liệu synthetic độc lập với ảnh chụp thật.
//  Áp transform "không ICP" khi chỉnh sửa cx,cz,step_deg,sign để đánh giá điểm số hình học thuần.
// ================================================================
TurntableCalibResult MultiPoseTriangulator::runAutoCalibSearch(
    const vector<vector<Point3D>>& clouds) const
{
    TurntableCalibResult result;
    result.cx = cfg.turntable_center[0];
    result.cy = cfg.turntable_center[1];    
    result.cz = cfg.turntable_center[2];
    result.step_deg = (cfg.turntable_step_deg > 0.0f)
                           ? cfg.turntable_step_deg
                           : 360.0f / (float)cfg.turntable_poses;
    result.sign = cfg.turntable_sign;

    if (clouds.size() < 3)
    {
        cout << "[ERROR] Can it nhat 3 pose hop le de autocalib. Dung.\n";
        return result;
    }

    double cx0 = cfg.turntable_center[0];
    double cy0 = cfg.turntable_center[1];
    double cz0 = cfg.turntable_center[2];
    float  step0 = result.step_deg;

    // ---- Điểm số HIỆN TẠI (để so sánh, dùng full cloud cho công bằng) ----
    result.score_mm_before = scoreCandidatePairwise(clouds, cx0, cy0, cz0, step0, cfg.turntable_sign);
    cout << "[AUTOCALIB] Diem so config HIEN TAI: " << result.score_mm_before << " mm"
         << " (center=(" << cx0 << "," << cy0 << "," << cz0 << ") step=" << step0
         << " sign=" << cfg.turntable_sign << ")\n";

    exportNoICP(clouds, cx0, cy0, cz0, step0, cfg.turntable_sign, "debug_before_autocalib.ply");

    // ---- GIAI ĐOẠN THÔ: subsample nhanh, quét lưới quanh giá trị hiện tại ----
    vector<vector<Point3D>> sub = subsampleClouds(clouds, 300);

    // Xác định sign trước (rẻ — chỉ 2 lần đánh giá) để không nhân đôi lưới thô.
    double scorePlus  = scoreCandidatePairwise(sub, cx0, cy0, cz0, step0, +1);
    double scoreMinus = scoreCandidatePairwise(sub, cx0, cy0, cz0, step0, -1);
    int bestSign = (scoreMinus < scorePlus) ? -1 : +1;
    cout << "[AUTOCALIB] Kiem tra chieu quay: sign=+1 -> " << scorePlus
         << " mm | sign=-1 -> " << scoreMinus << " mm  => chon sign=" << bestSign << "\n";

    const double centerRangeCoarse = 20.0, centerStepCoarse = 5.0;
    const float  stepRangeCoarse   = 2.0f,  stepStepCoarse  = 0.5f;

    double bestCx = cx0, bestCz = cz0;
    float  bestStep = step0;
    double bestScore = 1e18;

    for (double dx = -centerRangeCoarse; dx <= centerRangeCoarse + 1e-6; dx += centerStepCoarse)
    for (double dz = -centerRangeCoarse; dz <= centerRangeCoarse + 1e-6; dz += centerStepCoarse)
    for (float  ds = -stepRangeCoarse;   ds <= stepRangeCoarse + 1e-6f;  ds += stepStepCoarse)
    {
        double s = scoreCandidatePairwise(sub, cx0 + dx, cy0, cz0 + dz, step0 + ds, bestSign);
        if (s < bestScore) { bestScore = s; bestCx = cx0 + dx; bestCz = cz0 + dz; bestStep = step0 + ds; }
    }

    cout << "[AUTOCALIB] Xong giai doan THO: center=(" << bestCx << "," << cy0 << "," << bestCz
         << ") step=" << bestStep << " score=" << bestScore << " mm\n";

    // ---- GIAI ĐOẠN TINH: coordinate-descent quanh kết quả thô, full cloud ----
    // Mỗi vòng: tinh chỉnh lần lượt cx, rồi cz, rồi step_deg (giữ 2 cái kia
    // cố định ở giá trị tốt nhất vừa tìm) — rẻ hơn nhiều so với grid đầy đủ
    // 3 chiều, và đủ tốt vì đã xuất phát gần optimum từ giai đoạn thô.
    const double centerRangeFine = 5.0, centerStepFine = 1.0;
    const float  stepRangeFine   = 0.5f, stepStepFine  = 0.1f;

    double curCx = bestCx, curCz = bestCz;
    float  curStep = bestStep;
    double curScore = scoreCandidatePairwise(clouds, curCx, cy0, curCz, curStep, bestSign);

    for (int round = 0; round < 2; round++)
    {
        // --- tinh chỉnh cx, giữ cz/step cố định ---
        for (double dx = -centerRangeFine; dx <= centerRangeFine + 1e-6; dx += centerStepFine)
        {
            double s = scoreCandidatePairwise(clouds, bestCx + dx, cy0, curCz, curStep, bestSign);
            if (s < curScore) { curScore = s; curCx = bestCx + dx; }
        }
        bestCx = curCx;

        // --- tinh chỉnh cz, giữ cx/step cố định ---
        for (double dz = -centerRangeFine; dz <= centerRangeFine + 1e-6; dz += centerStepFine)
        {
            double s = scoreCandidatePairwise(clouds, curCx, cy0, bestCz + dz, curStep, bestSign);
            if (s < curScore) { curScore = s; curCz = bestCz + dz; }
        }
        bestCz = curCz;

        // --- tinh chỉnh step_deg, giữ cx/cz cố định ---
        for (float ds = -stepRangeFine; ds <= stepRangeFine + 1e-6f; ds += stepStepFine)
        {
            double s = scoreCandidatePairwise(clouds, curCx, cy0, curCz, bestStep + ds, bestSign);
            if (s < curScore) { curScore = s; curStep = bestStep + ds; }
        }
        bestStep = curStep;
    }

    result.cx       = curCx;
    result.cy       = cy0;   // không tìm — xem giải thích đầu hàm
    result.cz       = curCz;
    result.step_deg = curStep;
    result.sign     = bestSign;
    result.score_mm_after = curScore;

    cout << "[AUTOCALIB] Xong giai doan TINH: center=(" << result.cx << "," << result.cy
         << "," << result.cz << ") step=" << result.step_deg
         << " sign=" << result.sign << " score=" << result.score_mm_after << " mm\n";

    exportNoICP(clouds, result.cx, result.cy, result.cz, result.step_deg, result.sign,
                "debug_after_autocalib.ply");

    cout << "\n====== KET QUA AUTOCALIB ======\n";
    cout << "  Truoc : score=" << result.score_mm_before << " mm  (config hien tai)\n";
    cout << "  Sau   : score=" << result.score_mm_after  << " mm  (gia tri tim duoc)\n";
    if (result.score_mm_after < result.score_mm_before)
        cout << "  -> Cai thien " << (result.score_mm_before - result.score_mm_after) << " mm.\n";
    else
        cout << "  -> KHONG tot hon config hien tai — co the config da kha tot roi, hoac\n"
             << "     van de nam o huong truc (turntable_axis) chu khong phai center/step.\n";
    cout << "  2 file debug_before_autocalib.ply / debug_after_autocalib.ply da luu de xem truc quan.\n";

    return result;
}