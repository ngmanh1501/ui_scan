#include "triangulator.h"
#include "decode.h"


GraycodeTriangulator::GraycodeTriangulator(
    const Mat& Kc_, const Mat& distC_, const Mat& Kp_, const Mat& distP_,      
    const Mat& R_, const Mat& T_, const TriangulatorConfig& triCfg)
{
    Kc = Kc_.clone();
    Kp = Kp_.clone();    
    R  = R_.clone();     
    T  = T_.clone();     

    distCoeffs_c = distC_.clone();   
    distCoeffs_p = distP_.clone();    

    fx = Kc.at<double>(0,0);
    fy = Kc.at<double>(1,1);
    cx = Kc.at<double>(0,2);
    cy = Kc.at<double>(1,2);

    z_min = triCfg.z_min;
    z_max = triCfg.z_max;
    x_min = triCfg.x_min;
    x_max = triCfg.x_max;
    y_min = triCfg.y_min;
    y_max = triCfg.y_max;

    cout << "[INIT] Triangulator OK\n";
    cout << "[INIT] Graycode Triangulator\n";
    cout << "fx=" << fx << " fy=" << fy << endl;
    cout << "cx=" << cx << " cy=" << cy << endl;
}


bool GraycodeTriangulator::triangulatePoint(int u, int v, float proj_u, float proj_v, Point3D& pt)
{
    // ===== 1. Undistort về hệ tọa độ chuẩn hóa (Normalized Coordinates) =====
    vector<Point2f> cam_pt = { Point2f((float)u, (float)v) };
    vector<Point2f> proj_pt = { Point2f(proj_u, proj_v) };

    vector<Point2f> cam_undist, proj_undist;

    // BỎ tham số Kc, Kp ở cuối. Hàm sẽ trả về tọa độ (x, y) chuẩn hóa
    undistortPoints(cam_pt, cam_undist, Kc, distCoeffs_c);
    undistortPoints(proj_pt, proj_undist, Kp, distCoeffs_p);

    // ===== 2. Projection matrix (Hệ chuẩn hóa) =====
    // P1 = [I | 0]
    Mat P1 = Mat::eye(3, 4, CV_64F);

    // P2 = [R | T]
    Mat P2;
    hconcat(R, T, P2);
    
    // ===== 3. Triangulate =====
    Mat pts1 = (Mat_<double>(2,1) << cam_undist[0].x, cam_undist[0].y);
    Mat pts2 = (Mat_<double>(2,1) << proj_undist[0].x, proj_undist[0].y);

    Mat p4D;
    triangulatePoints(P1, P2, pts1, pts2, p4D);

    // ===== 4. Normalize =====
    double w = p4D.at<double>(3,0);
    if (fabs(w) < 1e-6){
        stats.fail_w++;
        return false;
    }  

    pt.x = p4D.at<double>(0,0) / w;
    pt.y = p4D.at<double>(1,0) / w;
    pt.z = p4D.at<double>(2,0) / w;

    min_z_seen = min(min_z_seen, pt.z);
    max_z_seen = max(max_z_seen, pt.z);

    if (pt.z < z_min || pt.z > z_max)
    {
        stats.fail_zcam++;
        return false;
    }

    if (pt.x < x_min || pt.x > x_max)
    {
        stats.fail_xcam++;
        return false;
    }

    if (pt.y < y_min || pt.y > y_max)
    {
        stats.fail_ycam++;
        return false;
    }

    Mat X = (Mat_<double>(3,1) <<
    pt.x,
    pt.y,
    pt.z);

    Mat Xp = R * X + T;

    double z_proj = Xp.at<double>(2,0);

    if (z_proj <= 0.0)
    {
        stats.fail_zproj++;
        return false;
    }
    return true;
}


// ================================================================
//  reconstruct() — overload CŨ (tương thích ngược)
//  Dùng giá trị mặc định của DecodeConfig/FilterConfig (định nghĩa
//  trong app_config.h) — các giá trị này TRÙNG với giá trị hard-code
//  cũ trước đây, nên hành vi không đổi nếu gọi overload này.
// ================================================================
vector<Point3D> GraycodeTriangulator::reconstruct(const vector<Mat>& patterns, int proj_w, int proj_h)
{
    return reconstruct(patterns, proj_w, proj_h, DecodeConfig(), FilterConfig());
}

// ================================================================
//  reconstruct() — overload MỚI
//  Nhận DecodeConfig + FilterConfig từ ngoài (đọc từ config.json
//  qua AppConfig::load() ở main.cpp) — KHÔNG còn hard-code threshold,
//  maxLg, reproj_max_err_px... trong file .cpp này nữa. Muốn đổi
//  tham số chỉ cần sửa config.json rồi chạy lại, không cần build.
// ================================================================
vector<Point3D> GraycodeTriangulator::reconstruct(
    const vector<Mat>& patterns, int proj_w, int proj_h,
    const DecodeConfig& decodeCfg, const FilterConfig& filterCfg)
{
    vector<Point3D> cloud;

    int total = 0, pass_decode = 0;

    // Map DecodeConfig (app_config.h) -> sl::DecodeParams (decode.h)
    sl::DecodeParams dp;
    dp.bits                 = decodeCfg.bits;
    dp.threshold            = decodeCfg.threshold;
    dp.useAdaptiveThreshold = decodeCfg.useAdaptiveThreshold;
    dp.maxLg                = decodeCfg.maxLg;
    dp.whiteIndex           = decodeCfg.whiteIndex;
    dp.blackIndex           = decodeCfg.blackIndex;
    dp.horizontalStart      = decodeCfg.horizontalStart;
    dp.verticalStart        = decodeCfg.verticalStart;

    for (int v = 0; v < patterns[0].rows; v++) {
        for (int u = 0; u < patterns[0].cols; u++) {

            total++;

            float proj_u_raw, proj_v_raw;
            bool ok = sl::decodeGrayPixel(
                patterns, u, v, proj_w, proj_h, dp,
                proj_u_raw, proj_v_raw);

            if (!ok)
            {
                // decodeGrayPixel gộp chung mọi lý do fail (Ld thấp, Lg cao,
                // gray-bit lỗi, out-of-bounds) thành 1 false. Không còn tách
                // riêng từng loại fail như code cũ — nếu cần debug chi tiết,
                // xem [DEBUG] |Pattern-Inverse| Statistics in ra bởi
                // sl::printDiffStatistics() ở cuối hàm.
                continue;
            }

            pass_decode++;

            // Remap về hệ preview — đây mới là không gian mà Kp/dist_p
            // (đọc từ projector_calibration.json) thực sự được calib.
            // proj_scale_x/y mặc định = 1.0 nếu setProjectorScale() chưa
            // được gọi, nên không phá vỡ pipeline cũ nếu ai đó quên gọi.
            float proj_u = proj_u_raw * proj_scale_x;
            float proj_v = proj_v_raw * proj_scale_y;

            // ===== Triangulation =====
            Point3D pt;
            if (triangulatePoint(u, v, proj_u, proj_v, pt))
            {
                pt.cam_u = u;
                pt.cam_v = v;

                pt.proj_u = proj_u;
                pt.proj_v = proj_v;

                cloud.push_back(pt);
            }
        }
    }

    sl::printDiffStatistics();

    cout << "\n[DEBUG]\n";
    cout << "Total: " << total << endl;
    cout << "After decode: " << pass_decode << endl<<endl;

    cout << "\n[TRIANGULATION STATS]\n";
    cout << "fail w      : " << stats.fail_w << endl;
    cout << "fail z cam  : " << stats.fail_zcam << endl;
    cout << "fail z proj : " << stats.fail_zproj << endl;
    cout << "fail x cam  : " << stats.fail_xcam << endl;
    cout << "fail y cam  : " << stats.fail_ycam << endl;

    cout << "z range seen: "
     << min_z_seen
     << " -> "
     << max_z_seen
     << endl<<endl;

    cout << "[INFO] raw cloud: "
        << cloud.size()
        << endl;

    cloud = filterReprojection(
        cloud,
        filterCfg.reproj_max_err_px
    );

    cloud = filterSpatialConsistency(
        cloud,
        patterns[0].cols,
        patterns[0].rows,
        filterCfg.spatial_max_jump
    );

    cloud = filterIQR(
        cloud,
        filterCfg.iqr_scale
    );

    cout << "[INFO] final cloud: "
        << cloud.size()
        << endl;

    return cloud;
}


vector<Point3D> GraycodeTriangulator::filterReprojection(
    const vector<Point3D>& input,
    float max_err_px)
{
    vector<Point3D> out;
    double sum_cam = 0;
    double sum_proj = 0;

    float max_cam = 0;
    float max_proj = 0; 

    Mat rvec_cam = Mat::zeros(3,1,CV_64F);
    Mat tvec_cam = Mat::zeros(3,1,CV_64F);

    Mat rvec_proj;
    Rodrigues(R, rvec_proj);

    int removed = 0;

    for (auto& p : input)
    {
        vector<Point3f> obj = {
            Point3f(p.x, p.y, p.z)
        };

        vector<Point2f> reproj_cam;
        vector<Point2f> reproj_proj;

        projectPoints(
            obj,
            rvec_cam,
            tvec_cam,
            Kc,
            distCoeffs_c,
            reproj_cam
        );

        projectPoints(
            obj,
            rvec_proj,
            T,
            Kp,
            distCoeffs_p,
            reproj_proj
        );

        float err_cam =
            norm(reproj_cam[0] - Point2f(p.cam_u, p.cam_v));

        float err_proj =
            norm(reproj_proj[0] - Point2f(p.proj_u, p.proj_v));

        if (err_cam <= max_err_px &&
            err_proj <= max_err_px)
        {
            out.push_back(p);
        }
        else
        {
            removed++;
        }
        sum_cam += err_cam;
        sum_proj += err_proj;

        max_cam = max(max_cam, err_cam);
        max_proj = max(max_proj, err_proj);
    }

    cout << "[REPROJ] removed "
         << removed
         << " / "
         << input.size()
         << endl;

    cout << "avg cam err  : "
        << sum_cam / input.size()
        << endl;

    cout << "avg proj err : "
        << sum_proj / input.size()
        << endl;

    cout << "max cam err  : "
        << max_cam
        << endl;

    cout << "max proj err : "
        << max_proj
        << endl;
    return out;
}


vector<Point3D>
GraycodeTriangulator::filterSpatialConsistency(
    const vector<Point3D>& input,
    int width,
    int height,
    float max_jump)
{
    vector<int> idx_map(width * height, -1);

    for (int i = 0; i < input.size(); i++)
    {
        int u = input[i].cam_u;
        int v = input[i].cam_v;

        idx_map[v * width + u] = i;
    }

    vector<Point3D> out;

    int dirs[4][2] = {
        {-1,0},
        {1,0},
        {0,-1},
        {0,1}
    };

    for (int i = 0; i < input.size(); i++)
    {
        auto& p = input[i];

        bool ok = true;

        for (auto& d : dirs)
        {
            int nu = p.cam_u + d[0];
            int nv = p.cam_v + d[1];

            if (nu < 0 || nv < 0 ||
                nu >= width || nv >= height)
                continue;

            int ni = idx_map[nv * width + nu];

            if (ni < 0)
                continue;

            auto& q = input[ni];

            float jump = norm(
                Point2f(p.proj_u, p.proj_v) -
                Point2f(q.proj_u, q.proj_v)
            );

            if (jump > max_jump)
            {
                ok = false;
                break;
            }
        }

        if (ok)
            out.push_back(p);
    }

    cout << "[SPATIAL] "
         << out.size()
         << " kept / "
         << input.size()
         << endl;

    return out;
}


vector<Point3D>
GraycodeTriangulator::filterIQR(
    const vector<Point3D>& input,
    float scale)
{
    if (input.empty())
        return input;

    vector<float> xs, ys, zs;

    xs.reserve(input.size());
    ys.reserve(input.size());
    zs.reserve(input.size());

    for (auto& p : input)
    {
        xs.push_back(p.x);
        ys.push_back(p.y);
        zs.push_back(p.z);
    }

    auto percentile = [](vector<float> v, float q)
    {
        sort(v.begin(), v.end());

        float idx = q * (v.size() - 1);

        int i0 = (int)floor(idx);
        int i1 = (int)ceil(idx);

        if (i0 == i1)
            return v[i0];

        float t = idx - i0;

        return v[i0] * (1.0f - t) + v[i1] * t;
    };

    float q1x = percentile(xs, 0.25f);
    float q3x = percentile(xs, 0.75f);

    float q1y = percentile(ys, 0.25f);
    float q3y = percentile(ys, 0.75f);

    float q1z = percentile(zs, 0.25f);
    float q3z = percentile(zs, 0.75f);

    float iqrx = q3x - q1x;
    float iqry = q3y - q1y;
    float iqrz = q3z - q1z;

    float minx = q1x - scale * iqrx;
    float maxx = q3x + scale * iqrx;

    float miny = q1y - scale * iqry;
    float maxy = q3y + scale * iqry;

    float minz = q1z - scale * iqrz;
    float maxz = q3z + scale * iqrz;

    vector<Point3D> out;

    int removed = 0;

    for (auto& p : input)
    {
        bool ok = true;

        if (p.x < minx || p.x > maxx)
            ok = false;

        if (p.y < miny || p.y > maxy)
            ok = false;

        if (p.z < minz || p.z > maxz)
            ok = false;

        if (ok)
            out.push_back(p);
        else
            removed++;
    }

    cout << "[IQR] removed "
         << removed
         << " / "
         << input.size()
         << endl;

    return out;
}


void GraycodeTriangulator::savePLY(const vector<Point3D>& cloud, const string& filename)
{
    ofstream file(filename);

    if (!file.is_open())
    {
        cout << "[ERROR] Cannot save file\n";
        return;
    }

    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << cloud.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";

    for (auto& p : cloud)
    {
        file << p.x << " " << p.y << " " << p.z << "\n";
    }

    file.close();

    cout << "[SAVE] " << filename << endl;
}