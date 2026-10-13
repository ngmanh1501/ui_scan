#include "camera_calib.h"
#include "projector_calib.h"
#include "object_capture.h"
#include "triangulator.h"
#include "multipose_triangulator.h"
#include "pose_estimator.h"   // Hướng B: ReferenceModel/ObjectPose, build_model/locate
#include "app_config.h"   // AppConfig::load() — TOÀN BỘ tham số đọc từ config.json

using namespace cv;
using namespace std;


// ============================================================
//  Helpers
// ============================================================

static bool loadCameraCalib(const string& path, Mat& Kc, Mat& dist_c)
{
    sl::loadCalibration(path, Kc, dist_c);
    if (Kc.empty()) { cerr << "[ERROR] " << path << " invalid\n"; return false; }
    return true;
}


static bool loadProjectorCalib(const string& path,
                               Mat& Kp, Mat& dist_p, Mat& R, Mat& T,
                               float& scale_x, float& scale_y)
{
    FileStorage fs(path, FileStorage::READ);
    if (!fs.isOpened()) { cerr << "[ERROR] Cannot open " << path << "\n"; return false; }
    fs["cameraMatrix"] >> Kp;
    fs["distCoeffs"]   >> dist_p;
    fs["R"]            >> R;
    fs["T"]            >> T;

    // scale_x/scale_y: tỉ lệ remap từ hệ Gray-code gốc [0..proj_w/h]
    // sang hệ preview [0..previewW/H] — phải được lưu bởi computeProjectorCalib()
    // và đọc lại ở đây để truyền vào setProjectorScale(). Nếu field không tồn tại
    // trong file JSON cũ (trước khi có feature này), fallback về 1.0 và cảnh báo.
    FileNode nx = fs["scale_x"];
    FileNode ny = fs["scale_y"];
    if (nx.empty() || ny.empty())
    {
        cerr << "[WARN] scale_x/scale_y khong co trong " << path
             << " — dung gia tri mac dinh 1.0. Ket qua triangulation co the sai.\n"
             << "       Hay chay lai 'projector' de calib lai va luu scale moi.\n";
        scale_x = 1.0f;
        scale_y = 1.0f;
    }
    else
    {
        scale_x = (float)(double)nx;
        scale_y = (float)(double)ny;
    }
    cout << "[INFO] Projector scale: scale_x=" << scale_x
         << "  scale_y=" << scale_y << "\n";
    return true;
}


static bool loadPatterns(const string& folder, int count, vector<Mat>& patterns)
{
    patterns.clear();
    for (int i = 0; i < count; i++)
    {
        Mat img = imread(folder + "/pattern_" + to_string(i) + ".png", IMREAD_GRAYSCALE);
        if (img.empty()) { cerr << "[ERROR] Missing pattern_" << i << "\n"; return false; }
        patterns.push_back(img);
    }
    Mat white = imread(folder + "/white.png", IMREAD_GRAYSCALE);
    Mat black = imread(folder + "/black.png", IMREAD_GRAYSCALE);
    if (white.empty() || black.empty()) { cerr << "[ERROR] Missing white/black\n"; return false; }
    patterns.push_back(white);
    patterns.push_back(black);
    cout << "[INFO] Loaded " << patterns.size() << " images\n";
    return true;
}


static int extractNumber(const string& s)
{
    size_t pos = s.find_last_of('_');
    if (pos == string::npos)
        return -1;

    return stoi(s.substr(pos + 1));
}


static vector<string> listDirectories(const string& path)
{
    vector<string> dirs;
    if (!filesystem::exists(path)) return dirs;
    for (const auto& entry : filesystem::directory_iterator(path))
        if (entry.is_directory())
            dirs.push_back(entry.path().filename().string());
    sort(dirs.begin(), dirs.end(),
    [](const string& a, const string& b)
    {
        return extractNumber(a) < extractNumber(b);
    });
    return dirs;
}

// Hiển thị danh sách và cho người dùng chọn 1 mục.
// Trả về index đã chọn, hoặc -1 nếu không hợp lệ.
static int pickFromList(const string& label, const vector<string>& items)
{
    if (items.empty())
    {
        cerr << "[ERROR] Danh sach " << label << " trong.\n";
        return -1;
    }
    cout << "\n" << label << ":\n";
    for (int i = 0; i < (int)items.size(); i++)
        cout << "  " << i << " : " << items[i] << "\n";

    int id;
    cout << "Chon " << label << ": ";
    if (!(cin >> id) || id < 0 || id >= (int)items.size())
    {
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cerr << "[ERROR] Lua chon khong hop le.\n";
        return -1;
    }
    return id;
}

// Giống listDirectories() nhưng liệt kê FILE (lọc theo đuôi mở rộng),
// dùng để liệt kê các model .yml đã build trong 1 thư mục nhóm — xem
// cmdLocate(). Sort theo tên (alphabet) — không dùng extractNumber() vì
// tên file model là tên object/session (chuỗi tuỳ ý), không phải "x_N".
static vector<string> listFiles(const string& path, const string& ext)
{
    vector<string> files;
    if (!filesystem::exists(path)) return files;
    for (const auto& entry : filesystem::directory_iterator(path))
        if (entry.is_regular_file() && entry.path().extension() == ext)
            files.push_back(entry.path().filename().string());
    sort(files.begin(), files.end());
    return files;
}

// Load calib camera + projector vào các tham số truyền vào.
// Đường dẫn file calib lấy từ AppPaths (config.json) thay vì hard-code.
// Trả về false nếu bất kỳ bước nào thất bại.
static bool loadAllCalib(const AppPaths& paths,
                         Mat& Kc, Mat& dist_c,
                         Mat& Kp, Mat& dist_p,
                         Mat& R,  Mat& T,
                         float& scale_x, float& scale_y)
{
    if (!loadCameraCalib(paths.cameraCalibFile, Kc, dist_c))                             return false;
    if (!loadProjectorCalib(paths.projectorCalibFile, Kp, dist_p, R, T, scale_x, scale_y)) return false;
    cout << "[DEBUG] Kc:\n" << Kc << "\n[DEBUG] Kp:\n" << Kp << "\n"
         << "[DEBUG] R:\n"  << R  << "\n[DEBUG] T:\n"  << T  << "\n";
    return true;
}


// ============================================================
//  Command handlers
// ============================================================

static void cmdTri(const ObjectCaptureConfig& objCfg, const AppConfig& cfg)
{
    cout << "\n====== TRIANGULATION ======\n";

    Mat Kc, dist_c, Kp, dist_p, R, T;
    float scale_x, scale_y;
    if (!loadAllCalib(cfg.app, Kc, dist_c, Kp, dist_p, R, T, scale_x, scale_y)) return;

    // Chọn object
    vector<string> objects = listDirectories(objCfg.rootPath);
    int objID = pickFromList("Objects", objects);
    if (objID < 0) return;

    // Chọn pose
    string objectFolder = objCfg.rootPath + "/" + objects[objID];
    vector<string> poses = listDirectories(objectFolder);

    if (poses.empty())
    {
        cerr << "[ERROR] No pose directories.\n";
        return;
    }

    int poseID = pickFromList("Poses", poses);
    if (poseID < 0) return;

    // Load patterns
    string folder = objectFolder + "/" + poses[poseID];
    const int BITS = (int)ceil(log2((double)max(cfg.app.proj_w, cfg.app.proj_h)));
    vector<Mat> patterns;
    if (!loadPatterns(folder, BITS * 4, patterns)) return;

    // Reconstruct — z_min/z_max/x_min/x_max/y_min/y_max/decode/filter: sửa trong
    // config.json, không cần build lại. Toàn bộ giới hạn bounding-box đọc thẳng
    // từ cfg.tri trong constructor (giống cách MultiPoseTriangulator nhận cfg.tri).
    GraycodeTriangulator tri(Kc, dist_c, Kp, dist_p, R, T, cfg.tri);
    // QUAN TRỌNG: scale proj coords từ hệ Gray-code [0..proj_w/h] sang hệ preview
    // [0..previewW/H] — đây là hệ mà Kp/dist_p được calib. Phải gọi trước reconstruct().
    tri.setProjectorScale(scale_x, scale_y);
    vector<Point3D> cloud = tri.reconstruct(patterns, cfg.app.proj_w, cfg.app.proj_h,
                                             cfg.decode, cfg.filter);

    if (cloud.empty()) { cerr << "[ERROR] Point cloud empty!\n"; return; }
    cout << "[INFO] Cloud size: " << cloud.size() << "\n";

    string outFile = "pointcloud_" + objects[objID] + "_" + poses[poseID] + ".ply";
    tri.savePLY(cloud, outFile);
    cout << "[DONE] Saved " << outFile << "\n";
}

static void cmdTriMulti(const ObjectCaptureConfig& objCfg, const AppConfig& cfg)
{
    cout << "\n====== MULTI-POSE TRIANGULATION ======\n";

    Mat Kc, dist_c, Kp, dist_p, R, T;
    float scale_x, scale_y;
    if (!loadAllCalib(cfg.app, Kc, dist_c, Kp, dist_p, R, T, scale_x, scale_y)) return;

    // Chọn object
    vector<string> objects = listDirectories(objCfg.rootPath);
    int objID = pickFromList("Objects", objects);
    if (objID < 0) return;

    string objectFolder = objCfg.rootPath + "/" + objects[objID];
    vector<string> poses = listDirectories(objectFolder);

    if (poses.empty())
    {
        cerr << "[ERROR] No pose directories.\n";
        return;
    }

    // Reconstruct tất cả pose — toàn bộ tham số (z_min/z_max/ICP/turntable/
    // decode/filter) lấy từ config.json, sửa file đó không cần build lại.
    MultiPoseTriangulator tri(Kc, dist_c, Kp, dist_p, R, T,
                              cfg.tri, cfg.decode, cfg.filter);
    // QUAN TRỌNG: scale proj coords từ hệ Gray-code [0..proj_w/h] sang hệ preview
    // [0..previewW/H] — phải set trước reconstructFolder() để mỗi pose dùng đúng scale.
    tri.setProjectorScale(scale_x, scale_y);
    vector<Point3D> cloud = tri.reconstructFolder(objectFolder,
                                                    poses,
                                                    cfg.app.proj_w,
                                                    cfg.app.proj_h);

    if (cloud.empty()) { cerr << "[ERROR] Cloud empty!\n"; return; }
    cout << "[INFO] Total points: " << cloud.size() << "\n";

    string outFile = "pointcloud_multi_" + objects[objID] + ".ply";
    tri.savePLY(cloud, outFile);
    cout << "[DONE] Saved " << outFile << "\n";
}

// ============================================================
//  cmdBuildModel  —  Hướng B: build + lưu reference model (1 MẶT) cho
//  object pose registration (locate). Tái dùng NGUYÊN VẸN reconstructFolder()
//  (giống cmdTriMulti) — chỉ khác là lấy thêm normals (out_normals) và
//  lưu lại thành file .yml thay vì chỉ xuất PLY để xem.
//
//  "Nhóm vật": nhiều mặt (session) khác nhau của CÙNG 1 vật vật lý gom
//  vào 1 thư mục Object_models/<nhom>/ — locate() sẽ thử qua hết các
//  file .yml trong thư mục nhóm đó (xem cmdLocate). Enter rỗng ở bước
//  hỏi tên nhóm -> dùng luôn tên object/session (trường hợp phổ biến:
//  1 object = 1 nhóm, chỉ có 1 mặt).
// ============================================================
static void cmdBuildModel(const ObjectCaptureConfig& objCfg, const AppConfig& cfg)
{
    cout << "\n====== BUILD REFERENCE MODEL (Huong B) ======\n";

    Mat Kc, dist_c, Kp, dist_p, R, T;
    float scale_x, scale_y;
    if (!loadAllCalib(cfg.app, Kc, dist_c, Kp, dist_p, R, T, scale_x, scale_y)) return;

    vector<string> objects = listDirectories(objCfg.rootPath);
    int objID = pickFromList("Objects", objects);
    if (objID < 0) return;

    string objectFolder = objCfg.rootPath + "/" + objects[objID];
    vector<string> poses = listDirectories(objectFolder);
    if (poses.empty()) { cerr << "[ERROR] No pose directories.\n"; return; }

    MultiPoseTriangulator tri(Kc, dist_c, Kp, dist_p, R, T,
                              cfg.tri, cfg.decode, cfg.filter);
    tri.setProjectorScale(scale_x, scale_y);

    vector<Vec3f> modelNormals;
    vector<Point3D> modelPoints = tri.reconstructFolder(objectFolder, poses,
                                                          cfg.app.proj_w, cfg.app.proj_h,
                                                          &modelNormals);

    if (modelPoints.empty()) { cerr << "[ERROR] Cloud empty!\n"; return; }
    cout << "[INFO] Model points: " << modelPoints.size() << "\n";

    // Tên nhóm vật — Enter rỗng thì dùng luôn tên object/session đã chọn.
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "Ten nhom vat (Enter = dung '" << objects[objID] << "'): ";
    string groupName;
    getline(cin, groupName);
    auto trim = [](string& s) {
        s.erase(0, s.find_first_not_of(" \t\r\n"));
        s.erase(s.find_last_not_of(" \t\r\n") + 1);
    };
    trim(groupName);
    if (groupName.empty()) groupName = objects[objID];

    string groupFolder = cfg.poseEst.modelRoot + "/" + groupName;
    filesystem::create_directories(groupFolder);

    ReferenceModel model;
    model.points  = modelPoints;
    model.normals = modelNormals;

    string modelFile = groupFolder + "/" + objects[objID] + ".yml";
    if (saveReferenceModel(modelFile, model))
        cout << "[DONE] Model saved: " << modelFile
             << "  (nhom '" << groupName << "', mat '" << objects[objID] << "')\n";
}


static void cmdAutoCalib(const ObjectCaptureConfig& objCfg, const AppConfig& cfg)
{
    cout << "\n====== AUTO-CALIBRATE TURNTABLE (center x,z + step + chieu quay) ======\n";
    cout << "Khong dung ICP, khong tu ghi de config.json — chi in ket qua de ban tu\n"
         << "kiem tra roi dan vao config.json neu dong y.\n";

    Mat Kc, dist_c, Kp, dist_p, R, T;
    float scale_x, scale_y;
    if (!loadAllCalib(cfg.app, Kc, dist_c, Kp, dist_p, R, T, scale_x, scale_y)) return;

    vector<string> objects = listDirectories(objCfg.rootPath);
    int objID = pickFromList("Objects", objects);
    if (objID < 0) return;

    string objectFolder = objCfg.rootPath + "/" + objects[objID];
    vector<string> poses = listDirectories(objectFolder);
    if (poses.empty()) { cerr << "[ERROR] No pose directories.\n"; return; }

    MultiPoseTriangulator tri(Kc, dist_c, Kp, dist_p, R, T,
                              cfg.tri, cfg.decode, cfg.filter);
    tri.setProjectorScale(scale_x, scale_y);

    TurntableCalibResult r = tri.autoCalibrateTurntable(objectFolder, poses,
                                                          cfg.app.proj_w, cfg.app.proj_h);

    cout << "\n====== DAN VAO config.json, muc \"triangulator\" ======\n";
    cout << "    \"turntable_step_deg\": " << r.step_deg << ",\n";
    cout << "    \"turntable_sign\": " << r.sign << ",\n";
    cout << "    \"turntable_center_x\": " << r.cx << ",\n";
    cout << "    \"turntable_center_y\": " << r.cy << ",  // KHONG doi — nam doc truc quay, khong anh huong ket qua\n";
    cout << "    \"turntable_center_z\": " << r.cz << ",\n";

    if (r.sign != cfg.tri.turntable_sign)
        cout << "\n[CHU Y] Chieu quay tim duoc (sign=" << r.sign << ") KHAC config hien tai (sign="
             << cfg.tri.turntable_sign << ") — nhieu kha nang chieu quay dang gia dinh bi nguoc.\n";

    cout << "\nDa luu debug_before_autocalib.ply (theo config hien tai) va\n"
         << "debug_after_autocalib.ply (theo gia tri tim duoc) — mo bang PLY viewer\n"
         << "de so sanh truc quan truoc khi quyet dinh cap nhat config.json.\n";
}


// ============================================================
//  cmdLocate  —  Hướng B: định vị vật đã biết (đã build_model) trong 1
//  cloud LIVE chụp nhanh (1 pose, KHÔNG quay bàn xoay) — thử qua HẾT các
//  mặt (.yml) đã build trong nhóm được chọn, giữ mặt khớp nhất (fitness
//  thấp nhất trong số các model valid=true).
//
//  Input "live" hiện lấy từ 1 pose ĐÃ CHỤP SẴN (giống cách cmdTri chọn
//  pose) — đang ở giai đoạn demo nên tận dụng data có sẵn để test nhanh,
//  không tự động trigger chụp mới. Sau này ghép robot thật chỉ cần đổi
//  bước "chọn pose" thành gọi thẳng captureObjectSession() cho 1 pose
//  mới, phần còn lại giữ nguyên.
// ============================================================
static void cmdLocate(const ObjectCaptureConfig& objCfg, const AppConfig& cfg)
{
    cout << "\n====== LOCATE (Huong B) ======\n";

    Mat Kc, dist_c, Kp, dist_p, R, T;
    float scale_x, scale_y;
    if (!loadAllCalib(cfg.app, Kc, dist_c, Kp, dist_p, R, T, scale_x, scale_y)) return;

    // Chọn nhóm vật (thư mục con của Object_models — mỗi file .yml trong
    // đó là 1 mặt đã build_model())
    vector<string> groups = listDirectories(cfg.poseEst.modelRoot);
    int groupID = pickFromList("Nhom vat (model)", groups);
    if (groupID < 0) return;

    string groupFolder = cfg.poseEst.modelRoot + "/" + groups[groupID];
    vector<string> modelFiles = listFiles(groupFolder, ".yml");
    if (modelFiles.empty())
    {
        cerr << "[ERROR] Nhom '" << groups[groupID]
             << "' chua co model nao (.yml). Chay build_model truoc.\n";
        return;
    }
    cout << "[INFO] Nhom '" << groups[groupID] << "' co " << modelFiles.size()
         << " mat da build.\n";

    // Chọn object + pose để chụp làm "live" — CÙNG cơ chế chọn như cmdTri.
    vector<string> objects = listDirectories(objCfg.rootPath);
    int objID = pickFromList("Objects (live)", objects);
    if (objID < 0) return;

    string objectFolder = objCfg.rootPath + "/" + objects[objID];
    vector<string> poses = listDirectories(objectFolder);
    if (poses.empty()) { cerr << "[ERROR] No pose directories.\n"; return; }

    int poseID = pickFromList("Poses (live)", poses);
    if (poseID < 0) return;

    string liveFolder = objectFolder + "/" + poses[poseID];
    const int BITS = (int)ceil(log2((double)max(cfg.app.proj_w, cfg.app.proj_h)));
    vector<Mat> patterns;
    if (!loadPatterns(liveFolder, BITS * 4, patterns)) return;

    // Reconstruct live cloud — 1 pose, giống hệt cmdTri (KHÔNG qua
    // reconstructFolder/ICP merge nào — cloud này đã ở đúng hệ camera
    // thật tại thời điểm chụp, không phụ thuộc góc bàn xoay).
    GraycodeTriangulator liveTri(Kc, dist_c, Kp, dist_p, R, T, cfg.tri);
    liveTri.setProjectorScale(scale_x, scale_y);
    vector<Point3D> liveCloud = liveTri.reconstruct(patterns, cfg.app.proj_w, cfg.app.proj_h,
                                                      cfg.decode, cfg.filter);
    if (liveCloud.empty()) { cerr << "[ERROR] Live cloud empty!\n"; return; }
    cout << "[INFO] Live cloud: " << liveCloud.size() << " diem\n";

    // MultiPoseTriangulator ở đây CHỈ để gọi computeNormals/icpAlign/
    // applyTransform/makeRotAxis (đã public hoá cho Hướng B) — KHÔNG gọi
    // reconstructFolder() trong hàm này.
    MultiPoseTriangulator tri(Kc, dist_c, Kp, dist_p, R, T,
                              cfg.tri, cfg.decode, cfg.filter);
    tri.setProjectorScale(scale_x, scale_y);

    int camW = patterns[0].cols, camH = patterns[0].rows;
    vector<Vec3f> liveNormals = tri.computeNormals(liveCloud, camW, camH);

    // Thử qua HẾT các mặt đã build, giữ mặt khớp nhất (fitness thấp nhất
    // trong số valid=true). Vẫn track "đỡ tệ nhất" dù chưa mặt nào valid,
    // để báo fitness cụ thể cho người dùng thay vì chỉ nói chung chung.
    ObjectPose best;
    ReferenceModel bestModel;
    string bestFace;
    bool foundValid = false;

    for (auto& mf : modelFiles)
    {
        ReferenceModel model;
        if (!loadReferenceModel(groupFolder + "/" + mf, model)) continue;

        cout << "\n--- Thu mat '" << mf << "' ---\n";
        ObjectPose pose = estimatePoseByRegistration(liveCloud, liveNormals, model,
                                                       tri, cfg.tri, cfg.poseEst);

        bool better = bestFace.empty() || pose.fitness < best.fitness;
        if (pose.valid && (!foundValid || pose.fitness < best.fitness))
        {
            best = pose; bestModel = model; bestFace = mf; foundValid = true;
        }
        else if (!foundValid && better)
        {
            best = pose; bestModel = model; bestFace = mf;
        }
    }

    cout << "\n====== KET QUA ======\n";
    if (!foundValid)
    {
        cerr << "[LOCATE] KHONG mat nao khop du tin cay (fitness tot nhat = "
             << best.fitness << " mm, mat '" << bestFace << "', nguong="
             << cfg.poseEst.pose_max_accept_fitness_mm << " mm).\n";
        return;
    }

    Mat rvec;
    Rodrigues(best.R, rvec);
    double angleDeg = norm(rvec) * 180.0 / CV_PI;

    cout << "Mat khop: " << bestFace << "  (fitness=" << best.fitness << " mm)\n";
    cout << "Vi tri (mm, he camera): (" << best.position.x << ", "
         << best.position.y << ", " << best.position.z << ")\n";
    cout << "Rotation matrix R (he camera):\n" << best.R << "\n";
    cout << "Xoay (Rodrigues, radian): " << rvec.t()
         << "  (tuong duong " << angleDeg << " do)\n";

    // Debug PLY: live cloud + model (mặt khớp nhất) đã transform theo
    // pose tìm được, đặt cạnh nhau để so trực quan — cùng tinh thần
    // debug_before/after_autocalib.ply của autoCalibrateTurntable().
    string liveFile = "locate_live_" + objects[objID] + "_" + poses[poseID] + ".ply";
    liveTri.savePLY(liveCloud, liveFile);

    vector<Point3D> modelAligned = tri.applyTransform(bestModel.points, best.T4x4);
    string alignedFile = "locate_model_aligned_" + groups[groupID] + ".ply";
    tri.savePLY(modelAligned, alignedFile);

    cout << "\nDa luu " << liveFile << " va " << alignedFile
         << " — mo bang PLY viewer, model_aligned phai chong khit len live\n"
         << "neu ket qua dung.\n";
}


// ============================================================
//  main
// ============================================================
int main(int argc, char** argv)
{
    // Optional single-command mode for the local web application.
    const bool singleCommand = argc >= 3 && std::string(argv[1]) == "--command";
    const std::string requestedCommand = singleCommand ? argv[2] : "";
    // ---- Load TOÀN BỘ tham số từ config.json ----
    // File config.json nằm cạnh executable. Lần chạy đầu tiên nếu
    // chưa có file, chương trình tự tạo với giá trị mặc định.
    // MUỐN ĐỔI THAM SỐ (z_min/z_max, ICP, turntable, decode threshold,
    // filter, proj_w/h, đường dẫn...): mở config.json bằng Notepad,
    // sửa số, lưu lại, chạy lại chương trình — KHÔNG cần sửa code.
    AppConfig cfg;
    AppConfig::load("config.json", cfg);
    cfg.print();

    ProjectorCalibConfig projCfg;
    projCfg.proj_w = cfg.app.proj_w;
    projCfg.proj_h = cfg.app.proj_h;
    // Toàn bộ tham số calib projector còn lại lấy từ config.json qua
    // cfg.projCalib — sửa config.json là đủ, không cần build lại.
    projCfg.previewW        = cfg.projCalib.previewW;
    projCfg.previewH        = cfg.projCalib.previewH;
    projCfg.boardSize       = Size(cfg.projCalib.board_cols, cfg.projCalib.board_rows);
    projCfg.squareSize      = cfg.projCalib.squareSize_mm;
    projCfg.threshold        = cfg.projCalib.threshold;
    projCfg.patchRadius      = cfg.projCalib.patchRadius;
    projCfg.minDecodePoints  = cfg.projCalib.minDecodePoints;
    projCfg.warmupFrames     = cfg.projCalib.warmupFrames;
    projCfg.patternDelayMs   = cfg.projCalib.patternDelayMs;
    projCfg.minPoses         = cfg.projCalib.minPoses;
    // Dùng chung 1 path với cfg.app.projectorCalibFile — tránh 2 biến
    // độc lập cùng trỏ 1 file JSON rồi lệch nhau khi đổi config.json.
    projCfg.outputJson       = cfg.app.projectorCalibFile;

    ObjectCaptureConfig objCfg;
    objCfg.proj_w   = cfg.app.proj_w;
    objCfg.proj_h   = cfg.app.proj_h;
    objCfg.rootPath = cfg.app.objectDataRoot;

    cout << "\n====== MENU ======\n"
         << "  camera     : calib camera\n"
         << "  projector  : calib projector (chup checkerboard)\n"
         << "  capture    : capture object to scan 3D\n"
         << "  tri        : reconstruct 3D (single pose)\n"
         << "  tri_multi  : reconstruct 3D (multiple pose)\n"
         << "  autocalib  : tu do turntable_center/step/sign tu du lieu da chup\n"
         << "  build_model: (Huong B) build + luu reference model 1 mat vat\n"
         << "  locate     : (Huong B) dinh vi vat (position+orientation) tu 1 pose live\n"
         << "  reload     : doc lai config.json (sau khi sua tham so)\n"
         << "  exit       : thoat\n\n";

    string command;
    while (true)
    {
        cout << "\nCommand: ";
        if (singleCommand) command = requestedCommand;
        else if (!(cin >> command)) break;   // EOF / pipe closed

        if      (command == "camera")    runcalibration(cfg.app.camera_url, "Camera_calib");
        else if (command == "projector") runProjectorCalibration(projCfg, cfg.app.camera_url);
        else if (command == "capture")   runObjectCapture(objCfg, cfg.app.camera_url);
        else if (command == "tri")       cmdTri(objCfg, cfg);
        else if (command == "tri_multi") cmdTriMulti(objCfg, cfg);
        else if (command == "autocalib") cmdAutoCalib(objCfg, cfg);
        else if (command == "build_model") cmdBuildModel(objCfg, cfg);
        else if (command == "locate")      cmdLocate(objCfg, cfg);
        else if (command == "reload")
        {
            // Đọc lại config.json mà KHÔNG cần restart chương trình —
            // tiện khi đang tuning tham số và muốn thử nhiều giá trị
            // liên tiếp trong cùng một phiên chạy.
            AppConfig::load("config.json", cfg);
            projCfg.proj_w  = cfg.app.proj_w;
            projCfg.proj_h  = cfg.app.proj_h;
            projCfg.previewW        = cfg.projCalib.previewW;
            projCfg.previewH        = cfg.projCalib.previewH;
            projCfg.boardSize       = Size(cfg.projCalib.board_cols, cfg.projCalib.board_rows);
            projCfg.squareSize      = cfg.projCalib.squareSize_mm;
            projCfg.threshold        = cfg.projCalib.threshold;
            projCfg.patchRadius      = cfg.projCalib.patchRadius;
            projCfg.minDecodePoints  = cfg.projCalib.minDecodePoints;
            projCfg.warmupFrames     = cfg.projCalib.warmupFrames;
            projCfg.patternDelayMs   = cfg.projCalib.patternDelayMs;
            projCfg.minPoses         = cfg.projCalib.minPoses;
            projCfg.outputJson       = cfg.app.projectorCalibFile;
            objCfg.proj_w   = cfg.app.proj_w;
            objCfg.proj_h   = cfg.app.proj_h;
            objCfg.rootPath = cfg.app.objectDataRoot;
            cfg.print();
        }
        else if (command == "exit")      { cout << "Bye!\n"; break; }
        else                             cout << "Unknown command.\n";
        if (singleCommand) break;
    }

    return 0;
}