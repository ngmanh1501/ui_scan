#include "projector_calib.h"
#include "decode.h"
 
using namespace cv;
using namespace std;
namespace fs = std::filesystem;
 
 
// ============================================================
//  Internal helper: load toàn bộ ảnh 1 pose từ disk
// ============================================================
static bool loadPatternSet(const string& pose_path,
                           int           pattern_count,
                           PatternSet&   out)
{
    out.images.clear();
 
    for (int i = 0; i < pattern_count; i++)
    {
        string file = pose_path + "/pattern_" + to_string(i) + ".png";
        Mat img = imread(file, IMREAD_GRAYSCALE);
 
        if (img.empty())
        {
            cout << "[ERROR] Missing: " << file << "\n";
            return false;
        }
 
        out.images.push_back(img);
    }
 
    // White
    Mat white = imread(pose_path + "/white.png", IMREAD_GRAYSCALE);
    if (white.empty()) { cout << "[ERROR] Missing white.png in " << pose_path << "\n"; return false; }
    out.images.push_back(white);
 
    // Black
    Mat black = imread(pose_path + "/black.png", IMREAD_GRAYSCALE);
    if (black.empty()) { cout << "[ERROR] Missing black.png in " << pose_path << "\n"; return false; }
    out.images.push_back(black);
 
    return true;
}
 
 
// ============================================================
//  Internal helper: chụp 1 frame ổn định từ camera
// ============================================================
static Mat grabFrame(VideoCapture& cap, int warmup)
{
    Mat frame;
    for (int k = 0; k < warmup; k++)
        cap >> frame;
    return frame;
}
 
 
// ============================================================
//  Internal helper: remap 1 vector proj points từ hệ Gray-code
//  (proj_w x proj_h) về hệ preview window (previewW x previewH)
// ============================================================
static vector<Point2f> remapProjToPreview(
    const vector<Point2f>&      proj_pts,
    const ProjectorCalibConfig& cfg)
{
    vector<Point2f> out;
    out.reserve(proj_pts.size());
 
    const double sx = cfg.scaleX();
    const double sy = cfg.scaleY();
 
    for (const auto& p : proj_pts)
        out.emplace_back((float)(p.x * sx), (float)(p.y * sy));
 
    return out;
}
 
 
// ============================================================
//  decodePose
//  Trả ra proj_valid trong hệ Gray-code gốc [0..proj_w, 0..proj_h].
//  KHÔNG remap ở đây — để recalibrateFromFolder cũng dùng được.
// ============================================================
bool decodePose(
    const PatternSet&           ps,
    const ProjectorCalibConfig& cfg,
    vector<Point3f>&            obj_valid,
    vector<Point2f>&            cam_valid,
    vector<Point2f>&            proj_valid)
{
    const int BITS = (int)ceil(log2((double)max(cfg.proj_w, cfg.proj_h)));
 
    sl::DecodeParams params;
    params.bits      = BITS;
    params.threshold = cfg.threshold;
 
    // ---- Lấy ảnh trắng (áp chót) để detect checkerboard ----
    const Mat& white = ps.images[ps.images.size() - 2];
 
    // ---- Detect checkerboard ----
    vector<Point2f> corners;
    bool found = findChessboardCornersSB(white, cfg.boardSize, corners);
 
    if(found)
    {
        cornerSubPix(
            white,
            corners,
            Size(11,11),
            Size(-1,-1),
            TermCriteria(
                TermCriteria::EPS + TermCriteria::COUNT,
                30,
                0.001));
    }

    cout << "[DECODE] Chessboard found=" << found
         << "  corners=" << corners.size() << "\n";
 
    if (!found || (int)corners.size() != cfg.boardSize.area())
    {
        cout << "[ERROR] Chessboard detection failed!\n";
        return false;
    }
 
    // Debug draw
    Mat dbg;
    cvtColor(white, dbg, COLOR_GRAY2BGR);
    drawChessboardCorners(dbg, cfg.boardSize, corners, found);
    imshow("corners", dbg);
    waitKey(1);
 
    // ---- Tạo object points ----
    vector<Point3f> obj;
    obj.reserve(cfg.boardSize.area());
    for (int y = 0; y < cfg.boardSize.height; y++)
        for (int x = 0; x < cfg.boardSize.width; x++)
            obj.emplace_back(x * cfg.squareSize, y * cfg.squareSize, 0.f);
 
    // ---- Decode từng corner ----
    int success = 0, fail = 0;
    const int R = cfg.patchRadius;
 
    for (int i = 0; i < (int)corners.size(); i++)
    {
        float cx = corners[i].x;
        float cy = corners[i].y;
 
        vector<float> us, vs;
 
        for (int dy = -R; dy <= R; dy++)
        {
            for (int dx = -R; dx <= R; dx++)
            {
                int xx = (int)(cx + dx);
                int yy = (int)(cy + dy);
 
                if (xx < 0 || yy < 0 || xx >= white.cols || yy >= white.rows)
                    continue;
 
                float pu, pv;
                bool ok = sl::decodeGrayPixel(
                    ps.images, xx, yy,
                    cfg.proj_w, cfg.proj_h,
                    params, pu, pv);
 
                if (ok) { us.push_back(pu); vs.push_back(pv); }
            }
        }
 
        int patchArea = (2 * R + 1) * (2 * R + 1);
        int minPts    = patchArea / 5;
 
        if ((int)us.size() < minPts)
        {
            fail++;
            continue;
        }

        // Dùng median để loại nhiễu
        nth_element(us.begin(), us.begin() + us.size() / 2, us.end());
        nth_element(vs.begin(), vs.begin() + vs.size() / 2, vs.end());
 
        float proj_u = us[us.size() / 2];
        float proj_v = vs[vs.size() / 2];
 
        // Lưu tọa độ Gray-code gốc — chưa remap
        proj_valid.push_back({proj_u, proj_v});
        cam_valid.push_back(corners[i]);
        obj_valid.push_back(obj[i]);
 
        success++;
        cout << "  [" << i << "] cam=(" << corners[i].x << "," << corners[i].y << ")"
             << "  proj_raw=(" << proj_u << "," << proj_v << ")\n";
    }
 
    cout << "[DECODE] success=" << success << "  fail=" << fail << "\n";
 
    sl::printDiffStatistics();
    
    if ((int)proj_valid.size() < cfg.minDecodePoints)
    {
        cout << "[ERROR] Too few valid points ("
             << proj_valid.size() << " < " << cfg.minDecodePoints << ")\n";
        return false;
    }
 
    return true;
}
 
 
// ============================================================
//  captureCalibPose
// ============================================================
bool captureCalibPose(
    VideoCapture&            cap,
    const vector<Mat>&       patterns,
    const ProjectorCalibConfig& cfg,
    const string&            root_path,
    int                      pose_id,
    vector<Point3f>&         obj_valid,
    vector<Point2f>&         cam_valid,
    vector<Point2f>&         proj_valid)
{
    string pose_path = root_path + "/pose_" + to_string(pose_id);
    fs::create_directories(pose_path);
 
    cout << "\n=== CAPTURE CALIB POSE " << pose_id << " ===\n";
 
    PatternSet ps;
 
    // ---- Gray-code patterns ----
    for (int i = 0; i < (int)patterns.size(); i++)
    {
        imshow("projector", patterns[i]);
        waitKey(cfg.patternDelayMs);
 
        Mat frame = grabFrame(cap, cfg.warmupFrames);
        if (frame.empty()) { cout << "[ERROR] Empty frame at pattern " << i << "\n"; return false; }
 
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        ps.images.push_back(gray);
 
        imwrite(pose_path + "/pattern_" + to_string(i) + ".png", gray);
        cout << "  Captured pattern " << i << "\n";
    }
 
    // ---- White ----
    {
        Mat white_pat(cfg.proj_h, cfg.proj_w, CV_8U, Scalar(255));
        imshow("projector", white_pat);
        waitKey(cfg.patternDelayMs);
 
        Mat frame = grabFrame(cap, cfg.warmupFrames);
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        ps.images.push_back(gray);
 
        imwrite(pose_path + "/white.png", gray);
        cout << "  Captured white\n";
    }
 
    // ---- Black ----
    {
        Mat black_pat(cfg.proj_h, cfg.proj_w, CV_8U, Scalar(30));
        imshow("projector", black_pat);
        waitKey(cfg.patternDelayMs);
 
        Mat frame = grabFrame(cap, cfg.warmupFrames);
        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        ps.images.push_back(gray);
 
        imwrite(pose_path + "/black.png", gray);
        cout << "  Captured black\n";
    }
 
    // ---- Decode (trả ra tọa độ Gray-code gốc) ----
    bool ok = decodePose(ps, cfg, obj_valid, cam_valid, proj_valid);
    if (!ok) { cout << "[ERROR] Decode failed for pose " << pose_id << "\n"; return false; }
 
    cout << "[DONE] Pose " << pose_id << " captured\n";
    return true;
}
 
 
// ============================================================
//  computeProjectorCalib
//  Remap proj points từ hệ Gray-code → hệ preview window
//  rồi mới calib.
// ============================================================
bool computeProjectorCalib(
    const vector<vector<Point3f>>& all_obj,
    const vector<vector<Point2f>>& all_cam,
    const vector<vector<Point2f>>& all_proj,   // hệ Gray-code gốc
    const Mat&                     Kc,
    const Mat&                     dist_c,
    const ProjectorCalibConfig&    cfg)
{
    const int MIN_POSES = cfg.minPoses;
    if ((int)all_obj.size() < MIN_POSES)
    {
        cout << "[ERROR] Need at least " << MIN_POSES << " poses (have "
             << all_obj.size() << ")\n";
        return false;
    }
 
    // ---- Remap toàn bộ proj points về hệ preview (previewW x previewH) ----
    const double sx = cfg.scaleX();
    const double sy = cfg.scaleY();
 
    cout << "\n[INFO] Remap proj coords: scale_x=" << sx
         << "  scale_y=" << sy
         << "  (" << cfg.proj_w << "x" << cfg.proj_h
         << " -> " << cfg.previewW << "x" << cfg.previewH << ")\n";
 
    vector<vector<Point2f>> all_proj_remap;
    all_proj_remap.reserve(all_proj.size());
 
    for (const auto& pose_pts : all_proj)
        all_proj_remap.push_back(remapProjToPreview(pose_pts, cfg));
 
    // ---- In thống kê spread sau remap để kiểm tra ----
    {
        float min_u = 1e9f, max_u = -1e9f, min_v = 1e9f, max_v = -1e9f;
        for (const auto& pose : all_proj_remap)
            for (const auto& p : pose)
            {
                min_u = min(min_u, p.x); max_u = max(max_u, p.x);
                min_v = min(min_v, p.y); max_v = max(max_v, p.y);
            }
        cout << "[INFO] Proj points spread (preview space):"
             << "  u=[" << min_u << ".." << max_u << "]"
             << "  v=[" << min_v << ".." << max_v << "]\n";
    }
 
    // ---- Init Kp trong không gian preview ----
    Mat Kp = Mat::eye(3, 3, CV_64F);
    Kp.at<double>(0, 0) = cfg.focalGuess();
    Kp.at<double>(1, 1) = cfg.focalGuess();
    Kp.at<double>(0, 2) = cfg.previewW * 0.5;
    Kp.at<double>(1, 2) = cfg.previewH * 0.5;
    Mat dist_p = Mat::zeros(5, 1, CV_64F);
 
    vector<Mat> rvecs, tvecs;
 
    // ---- Projector intrinsic (calib trong không gian previewW x previewH) ----
    double proj_err = calibrateCamera(
        all_obj, all_proj_remap, Size(cfg.previewW, cfg.previewH),
        Kp, dist_p, rvecs, tvecs,
        //CALIB_USE_INTRINSIC_GUESS | // dùng Kp khởi tạo ở trên
        //CALIB_FIX_ASPECT_RATIO    | // khóa fx/fy ratio vì đã đo bằng thước
        CALIB_ZERO_TANGENT_DIST   | // khóa p1, p2 vì đã đo bằng thước
        //CALIB_FIX_PRINCIPAL_POINT | // khóa cx, cy vì đã đo bằng thước
        //CALIB_FIX_K2 |
        CALIB_FIX_K3 | CALIB_FIX_K4 | CALIB_FIX_K5);
    
    cout << "\nProjector reprojection error (preview space) = " << proj_err << "\n";
 
    // ---- Stereo (camera <-> projector, dùng proj remapped) ----
    Mat R, T, E, F;
    double stereo_err = stereoCalibrate(
        all_obj, all_cam, all_proj_remap,
        Kc, dist_c, Kp, dist_p,
        Size(cfg.previewW, cfg.previewH),
        R, T, E, F
        ,CALIB_FIX_INTRINSIC // chỉ tính R, T, không update Kc/Kp/dist_c/dist_p
    );
 
    cout << "Stereo reprojection error = " << stereo_err << "\n";
 
    // ---- Chuyển Kp về hệ full projector (proj_w x proj_h) ----
    // fx, fy scale ngược lại; cx, cy scale ngược lại
    Mat Kp_full = Kp.clone();
    Kp_full.at<double>(0, 0) = Kp.at<double>(0, 0) / sx;   // fx
    Kp_full.at<double>(1, 1) = Kp.at<double>(1, 1) / sy;   // fy
    Kp_full.at<double>(0, 2) = Kp.at<double>(0, 2) / sx;   // cx
    Kp_full.at<double>(1, 2) = Kp.at<double>(1, 2) / sy;   // cy
 
    cout << "\n--- Kp (preview " << cfg.previewW << "x" << cfg.previewH << ") ---\n"
         << Kp << "\n";
    cout << "--- Kp_full (projector " << cfg.proj_w << "x" << cfg.proj_h << ") ---\n"
         << Kp_full << "\n";
    cout << "dist_p =\n"  << dist_p << "\n";
    cout << "R =\n"       << R      << "\n";
    cout << "T =\n"       << T      << "\n";
 
    // ---- Save ----
    // Chỉ lưu các thông số THỰC SỰ dùng cho triangulate/reconstruction.
    // Kp, dist_p ở đây là hệ preview (proj THỰC SỰ chiếu sáng) — đây
    // cũng là hệ mà triangulator phải remap proj_u/proj_v về trước khi
    // dùng Kp, nên scale_x/scale_y được lưu kèm để bên triangulator
    // đọc động, không hardcode tỉ lệ trong code.
    FileStorage fsSave(cfg.outputJson,
                       FileStorage::WRITE | FileStorage::FORMAT_JSON);

    fsSave << "cameraMatrix"   << Kp;           // hệ preview (previewW x previewH)
    fsSave << "distCoeffs"     << dist_p;
    fsSave << "R"              << R;
    fsSave << "T"              << T;

    // Gray-code gốc (không gian decode) + scale để remap sang hệ preview ở trên
    fsSave << "proj_w"         << cfg.proj_w;
    fsSave << "proj_h"         << cfg.proj_h;
    fsSave << "previewW"       << cfg.previewW;
    fsSave << "previewH"       << cfg.previewH;
    fsSave << "scale_x"        << sx;
    fsSave << "scale_y"        << sy;
    fsSave.release();
 
    cout << "[DONE] Saved to " << cfg.outputJson << "\n";
    return true;
}
 
 
// ============================================================
//  recalibrateFromFolder
// ============================================================
bool recalibrateFromFolder(
    const string&               root_path,
    const Mat&                  Kc,
    const Mat&                  dist_c,
    const ProjectorCalibConfig& cfg)
{
    cout << "\n=== RECALIBRATE FROM FOLDER: " << root_path << " ===\n";
 
    const int BITS          = (int)ceil(log2((double)max(cfg.proj_w, cfg.proj_h)));
    const int pattern_count = BITS * 4;   // 2 axes × 2 (normal + inverse)
 
    vector<vector<Point3f>> all_obj;
    vector<vector<Point2f>> all_cam;
    vector<vector<Point2f>> all_proj;   // hệ Gray-code gốc
 
    // Sort folders để thứ tự nhất quán
    vector<fs::path> dirs;
    for (const auto& entry : fs::directory_iterator(root_path))
        if (entry.is_directory())
            dirs.push_back(entry.path());
    sort(dirs.begin(), dirs.end());
 
    for (const auto& dir : dirs)
    {
        string pose_path = dir.string();
        cout << "\n[LOAD] " << pose_path << "\n";
 
        PatternSet ps;
        if (!loadPatternSet(pose_path, pattern_count, ps))
        {
            cout << "[SKIP] Incomplete pose\n";
            continue;
        }
 
        vector<Point3f> obj_valid;
        vector<Point2f> cam_valid;
        vector<Point2f> proj_valid;
 
        if (!decodePose(ps, cfg, obj_valid, cam_valid, proj_valid))
        {
            cout << "[SKIP] Decode failed\n";
            continue;
        }
 
        all_obj.push_back(obj_valid);
        all_cam.push_back(cam_valid);
        all_proj.push_back(proj_valid);
 
        cout << "[OK] Pose added (total=" << all_obj.size() << ")\n";
    }
 
    // computeProjectorCalib sẽ tự remap
    return computeProjectorCalib(all_obj, all_cam, all_proj, Kc, dist_c, cfg);
}
 
 
// ============================================================
//  runProjectorCalibration  –  vòng lặp tương tác
// ============================================================
void runProjectorCalibration(const ProjectorCalibConfig& cfg, int camera_url)
{
    cout << "\n====== PROJECTOR CALIBRATION ======\n"
         << "  Projector (Gray-code) : " << cfg.proj_w << "x" << cfg.proj_h << "\n"
         << "  Preview window        : " << cfg.previewW << "x" << cfg.previewH << "\n"
         << "  Scale factor          : " << cfg.scaleX() << " (x) / " << cfg.scaleY() << " (y)\n"
         << "  Focal guess           : " << cfg.focalGuess() << " px\n\n"
         << "  a : chụp 1 pose mới\n"
         << "  p : tính calibration từ pose hiện tại\n"
         << "  r : tính lại từ folder đã lưu\n"
         << "  q : thoát\n\n";
 
    // ---- Load camera calibration ----
    Mat Kc, dist_c;
    sl::loadCalibration("camera_calibration.json", Kc, dist_c);
 
    const string save_path = "Projector_data";
    fs::create_directories(save_path);
 
    // ---- Tạo Gray-code patterns (vẫn generate cho proj_w x proj_h) ----
    auto graygen = structured_light::GrayCodePattern::create(cfg.proj_w, cfg.proj_h);
    vector<Mat> patterns;
    graygen->generate(patterns);
    cout << "[INFO] Pattern count = " << patterns.size() << "\n";
 
    // ---- Mở camera ----
    VideoCapture cap(camera_url);
    if (!cap.isOpened()) { cout << "[ERROR] Cannot open camera\n"; return; }
 
    // ---- Windows ----
    namedWindow("projector", WINDOW_NORMAL);
    setWindowProperty("projector", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    //resizeWindow("projector", cfg.previewW, cfg.previewH);
    //moveWindow("projector", cfg.previewX, cfg.previewY);
    namedWindow("camera", WINDOW_NORMAL);
    resizeWindow("camera", 320*2, 240*2);
    moveWindow("camera", 200, 200);
 
    // ---- Storage ----
    vector<vector<Point3f>> all_obj;
    vector<vector<Point2f>> all_cam;
    vector<vector<Point2f>> all_proj;   // hệ Gray-code gốc
    int pose_id = 0;
 
    // ---- Main loop ----
    while (true)
    {
        char key = (char)waitKey(1);
 
        // Preview camera liên tục
        Mat frame;
        cap >> frame;
        if (!frame.empty())
            imshow("camera", frame);
 
        if (key == 'a')
        {
            vector<Point3f> obj_valid;
            vector<Point2f> cam_valid;
            vector<Point2f> proj_valid;
 
            bool ok = captureCalibPose(cap, patterns, cfg,
                                       save_path, pose_id,
                                       obj_valid, cam_valid, proj_valid);
            if (ok)
            {
                all_obj.push_back(obj_valid);
                all_cam.push_back(cam_valid);
                all_proj.push_back(proj_valid);
                cout << "[INFO] Total poses = " << all_obj.size() << "\n";
                pose_id++;
            }
        }
        else if (key == 'p')
        {
            computeProjectorCalib(all_obj, all_cam, all_proj, Kc, dist_c, cfg);
        }
        else if (key == 'r')
        {
            recalibrateFromFolder(save_path, Kc, dist_c, cfg);
        }
        else if (key == 'q')
        {
            break;
        }
    }
 
    cap.release();
    destroyAllWindows();
}