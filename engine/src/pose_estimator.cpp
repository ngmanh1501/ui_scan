#include "pose_estimator.h"
#include <iostream>
#include <algorithm>
#include <cmath>

// ================================================================
//  saveReferenceModel / loadReferenceModel
//  2 Mat Nx3 CV_32F ("points","normals") qua cv::FileStorage — cùng cơ
//  chế với camera_calibration.json/projector_calibration.json đang dùng
//  (xem loadCalibration() trong decode.h).
// ================================================================
bool saveReferenceModel(const string& path, const ReferenceModel& model)
{
    if (model.points.size() != model.normals.size())
    {
        cout << "[MODEL] points/normals lech kich thuoc (" << model.points.size()
             << " / " << model.normals.size() << "), khong luu.\n";
        return false;
    }
    if (model.points.empty())
    {
        cout << "[MODEL] cloud rong, khong luu.\n";
        return false;
    }

    Mat pts((int)model.points.size(), 3, CV_32F);
    Mat nrm((int)model.normals.size(), 3, CV_32F);
    for (int i = 0; i < (int)model.points.size(); i++)
    {
        pts.at<float>(i,0) = model.points[i].x;
        pts.at<float>(i,1) = model.points[i].y;
        pts.at<float>(i,2) = model.points[i].z;

        nrm.at<float>(i,0) = model.normals[i][0];
        nrm.at<float>(i,1) = model.normals[i][1];
        nrm.at<float>(i,2) = model.normals[i][2];
    }

    FileStorage fs(path, FileStorage::WRITE);
    if (!fs.isOpened())
    {
        cout << "[MODEL] Khong the tao file " << path << "\n";
        return false;
    }
    fs << "points"  << pts;
    fs << "normals" << nrm;
    fs.release();

    cout << "[MODEL] Da luu " << path << " (" << model.points.size() << " diem)\n";
    return true;
}

bool loadReferenceModel(const string& path, ReferenceModel& model)
{
    FileStorage fs(path, FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "[MODEL] Khong mo duoc " << path << "\n";
        return false;
    }

    Mat pts, nrm;
    fs["points"]  >> pts;
    fs["normals"] >> nrm;
    fs.release();

    if (pts.empty() || nrm.empty() || pts.rows != nrm.rows || pts.cols != 3 || nrm.cols != 3)
    {
        cout << "[MODEL] " << path << " khong hop le (thieu points/normals hoac lech kich thuoc)\n";
        return false;
    }

    model.points.resize(pts.rows);
    model.normals.resize(pts.rows);
    for (int i = 0; i < pts.rows; i++)
    {
        model.points[i].x = pts.at<float>(i,0);
        model.points[i].y = pts.at<float>(i,1);
        model.points[i].z = pts.at<float>(i,2);

        // cam_u/cam_v/proj_u/proj_v: hết ý nghĩa sau khi model được lưu/đọc
        // lại riêng (mỗi pose gốc có grid riêng, đã mất khi merge) — đặt 0.
        // KHÔNG dùng lại computeNormals() trên model đọc từ đây — normals
        // đã tính sẵn TRƯỚC khi merge trong reconstructFolder(), dùng nguyên.
        model.points[i].cam_u  = 0;
        model.points[i].cam_v  = 0;
        model.points[i].proj_u = 0.0f;
        model.points[i].proj_v = 0.0f;

        model.normals[i] = Vec3f(nrm.at<float>(i,0), nrm.at<float>(i,1), nrm.at<float>(i,2));
    }

    cout << "[MODEL] Da doc " << path << " (" << model.points.size() << " diem)\n";
    return true;
}

// ================================================================
//  fitnessFromTo — percentile-30 nearest-neighbor distance (mm) TỪ mỗi
//  điểm trong "from" ĐẾN "to". Cùng công thức với
//  MultiPoseTriangulator::scoreCandidatePairwise() (KD-tree + percentile-
//  30) nhưng viết riêng ở đây vì hàm đó là private, gắn chặt với luồng
//  autoCalibrateTurntable(), không tái dùng trực tiếp được.
// ================================================================
static double fitnessFromTo(const vector<Point3D>& from, const vector<Point3D>& to)
{
    if (from.empty() || (int)to.size() < 4)
        return 1e18;

    Mat toMat((int)to.size(), 3, CV_32F);
    for (int i = 0; i < (int)to.size(); i++)
    {
        toMat.at<float>(i,0) = to[i].x;
        toMat.at<float>(i,1) = to[i].y;
        toMat.at<float>(i,2) = to[i].z;
    }
    cv::flann::Index kd(toMat, cv::flann::KDTreeIndexParams(4));

    vector<float> dists;
    dists.reserve(from.size());
    for (auto& p : from)
    {
        Mat q = (Mat_<float>(1,3) << p.x, p.y, p.z);
        vector<int> idx(1);
        vector<float> dist(1);
        kd.knnSearch(q, idx, dist, 1, cv::flann::SearchParams(32));
        dists.push_back(dist[0]);   // squared L2
    }

    sort(dists.begin(), dists.end());
    int p30 = max(0, (int)(0.3 * dists.size()) - 1);
    return sqrt((double)dists[p30]);
}

// ================================================================
//  estimatePoseByRegistration
// ================================================================
ObjectPose estimatePoseByRegistration(
    const vector<Point3D>& liveCloud,
    const vector<Vec3f>&   liveNormals,
    const ReferenceModel&  model,
    const MultiPoseTriangulator& tri,
    const TriangulatorConfig&    triCfg,
    const PoseEstimationConfig&  poseCfg)
{
    ObjectPose result;
    result.T4x4 = Mat::eye(4, 4, CV_64F);
    result.R    = Mat::eye(3, 3, CV_64F);

    if ((int)liveCloud.size() < poseCfg.pose_min_live_points)
    {
        cout << "[POSE] Live cloud qua it diem (" << liveCloud.size()
             << " < " << poseCfg.pose_min_live_points << "), tu choi.\n";
        return result;
    }
    if (model.points.empty())
    {
        cout << "[POSE] Reference model rong, tu choi.\n";
        return result;
    }

    // ---- Coarse: search 1 chiều quanh turntable_axis, tâm turntable_center
    //      (dùng ĐÚNG công thức T_back*R_guess*T_to như reconstructFolder()/
    //      scoreCandidatePairwise() trong multipose_triangulator.cpp) ----
    double cx = triCfg.turntable_center[0];
    double cy = triCfg.turntable_center[1];
    double cz = triCfg.turntable_center[2];

    Mat T_to = Mat::eye(4, 4, CV_64F);
    T_to.at<double>(0,3) = -cx; T_to.at<double>(1,3) = -cy; T_to.at<double>(2,3) = -cz;
    Mat T_back = Mat::eye(4, 4, CV_64F);
    T_back.at<double>(0,3) = cx; T_back.at<double>(1,3) = cy; T_back.at<double>(2,3) = cz;

    float stepDeg = (poseCfg.pose_coarse_angle_step_deg > 0.1f)
                        ? poseCfg.pose_coarse_angle_step_deg : 15.0f;
    int nSteps = max(1, (int)std::round(360.0f / stepDeg));

    double bestScore = 1e18;
    Mat    bestGuess = Mat::eye(4, 4, CV_64F);

    cout << "[POSE] Coarse search: " << nSteps << " gia thuyet goc (buoc "
         << stepDeg << " do)\n";

    for (int k = 0; k < nSteps; k++)
    {
        double angleRad = (double)k * stepDeg * CV_PI / 180.0;
        Mat R_guess = tri.makeRotAxis(triCfg.turntable_axis[0],
                                       triCfg.turntable_axis[1],
                                       triCfg.turntable_axis[2], angleRad);
        Mat T_guess = T_back * R_guess * T_to;

        vector<Point3D> modelGuess = tri.applyTransform(model.points, T_guess);
        double score = fitnessFromTo(modelGuess, liveCloud);

        if (score < bestScore)
        {
            bestScore = score;
            bestGuess = T_guess;
        }
    }

    cout << "[POSE] Coarse best score = " << bestScore << " mm\n";

    // ---- Fine: icpAlign() đầy đủ (KHÔNG đổi logic ICP) từ guess tốt nhất.
    //      src=model, ref=live: T4x4 * model ≈ live — đúng nghĩa "model
    //      đang ở đâu trong hệ camera hiện tại" = pose cần tìm. ----
    Mat T4x4 = tri.icpAlign(model.points, model.normals, liveCloud, liveNormals, bestGuess);

    vector<Point3D> modelFinal = tri.applyTransform(model.points, T4x4);
    double finalFitness = fitnessFromTo(modelFinal, liveCloud);

    result.T4x4     = T4x4.clone();
    result.R        = T4x4(Rect(0,0,3,3)).clone();
    result.position = Point3f((float)T4x4.at<double>(0,3),
                               (float)T4x4.at<double>(1,3),
                               (float)T4x4.at<double>(2,3));
    result.fitness  = finalFitness;
    result.valid    = finalFitness <= poseCfg.pose_max_accept_fitness_mm;

    cout << "[POSE] Fine fitness = " << finalFitness << " mm ("
         << (result.valid ? "DAT" : "KHONG DAT")
         << ", nguong=" << poseCfg.pose_max_accept_fitness_mm << " mm)\n";

    return result;
}