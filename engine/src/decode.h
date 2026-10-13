#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <array>
#include <numeric>

 

namespace sl
{

static std::array<uint64_t, 256> g_diffHist = {};

struct DecodeParams
{
    int bits = 10;

    int threshold = 10;

    int horizontalStart = 0;
    int verticalStart   = 20;

    int whiteIndex = 40;
    int blackIndex = 41;

    // ---- Tuỳ chọn nâng cao (mặc định TẮT để không đổi hành vi cũ) ----
    // Khi bật: threshold cho từng pixel được tính lại theo độ tương phản
    // trực tiếp (Ld = white - black) thay vì dùng `threshold` cố định —
    // pixel có ánh sáng mạnh sẽ chấp nhận ngưỡng cao hơn, giúp loại
    // nhiễu tốt hơn ở vùng tối/yếu sáng. Công thức: max(threshold, Ld/10).
    bool useAdaptiveThreshold = false;

    // Khi > 0: loại pixel có vùng tối (blackVal) quá sáng — dấu hiệu
    // saturation/phản xạ nhiễu (vùng quá sáng khiến white/black gần
    // nhau theo giá trị tuyệt đối dù Ld vẫn đủ lớn).
    int maxLg = 0;   // 0 = không áp dụng check này
};


inline int getRobustBit(int p, int ip, int threshold)
{
    if (std::abs(p - ip) < threshold)
        return -1;

    return (p > ip) ? 1 : 0;
}


inline uint16_t grayToBinary(uint16_t gray)
{
    uint16_t bin = gray;

    while (gray >>= 1)
        bin ^= gray;

    return bin;
}


// ======================================================
// Decode one camera pixel
// ======================================================

inline bool decodeGrayPixel(
    const std::vector<cv::Mat>& patterns,
    int u,
    int v,
    int projWidth,
    int projHeight,
    const DecodeParams& params,
    float& proj_u,
    float& proj_v)
{
    const int bits = params.bits;

    // ==================================================
    // White / Black validation
    // ==================================================

    int whiteVal = patterns[params.whiteIndex].at<uchar>(v, u);
    int blackVal = patterns[params.blackIndex].at<uchar>(v, u);

    int directLight = whiteVal - blackVal;

    if (directLight < params.threshold)
        return false;

    // Lọc vùng saturation / phản xạ nhiễu: nếu cả white lẫn black đều
    // sáng cao (Lg lớn) thì pixel này không đáng tin dù Ld đủ lớn.
    if (params.maxLg > 0)
    {
        int Lg = std::min(whiteVal, blackVal);
        if (Lg > params.maxLg)
            return false;
    }

    // Adaptive threshold: pixel tương phản mạnh dùng ngưỡng cao hơn để
    // loại nhiễu tốt hơn, pixel tương phản yếu vẫn giữ ngưỡng tối thiểu.
    const int effectiveThreshold = params.useAdaptiveThreshold
        ? std::max(params.threshold, directLight / 10)
        : params.threshold;

    // ==================================================
    // Decode Gray codes
    // ==================================================

    uint16_t grayH = 0;
    uint16_t grayV = 0;

    // ---------------- Horizontal ----------------

    for (int i = 0; i < bits; i++)
    {
        int idx = params.horizontalStart + i * 2;

        int p  = patterns[idx].at<uchar>(v, u);
        int ip = patterns[idx + 1].at<uchar>(v, u);

        g_diffHist[std::abs(p - ip)]++;

        int bit = getRobustBit(p, ip, effectiveThreshold);

        if (bit == -1)
            return false;

        if (bit)
            grayH |= (1 << (bits - 1 - i));
    }

    // ---------------- Vertical ----------------

    for (int i = 0; i < bits; i++)
    {
        int idx = params.verticalStart + i * 2;

        int p  = patterns[idx].at<uchar>(v, u);
        int ip = patterns[idx + 1].at<uchar>(v, u);

        g_diffHist[std::abs(p - ip)]++;

        int bit = getRobustBit(p, ip, effectiveThreshold);

        if (bit == -1)
            return false;

        if (bit)
            grayV |= (1 << (bits - 1 - i));
    }

    // ==================================================
    // Gray -> Binary
    // ==================================================
    int decodedU = grayToBinary(grayH);
    int decodedV = grayToBinary(grayV);

    // ==================================================
    // Offset correction
    // ==================================================
    // Tính toán offset dựa trên lũy thừa của 2
    int maxW = 1 << params.bits; // 1024
    int maxH = 1 << params.bits; // 1024
    
    int offsetU = (maxW - projWidth) / 2;   // (1024 - 1024)/2 = 0
    int offsetV = (maxH - projHeight) / 2;  // (1024 - 768)/2 = 128

    // Trừ đi offset để đưa về đúng hệ tọa độ pixel của projector
    int finalU = decodedU - offsetU;
    int finalV = decodedV - offsetV;
    // ==================================================
    // Bounds check
    // ==================================================

    if (finalU < 0 || finalU >= projWidth)
        return false;

    if (finalV < 0 || finalV >= projHeight)
        return false;

    // ==================================================
    // Pixel center
    // ==================================================

    proj_u = finalU + 0.5f;
    proj_v = finalV + 0.5f;

    return true;
}


inline void printDiffStatistics()
{
    uint64_t total = 0;

    for (auto c : g_diffHist)
        total += c;

    if (total == 0)
    {
        std::cout << "[DEBUG] No statistics.\n";
        return;
    }

    auto percentile = [&](double p)
    {
        uint64_t target = static_cast<uint64_t>(p * total);

        uint64_t sum = 0;

        for (int i = 0; i < 256; i++)
        {
            sum += g_diffHist[i];

            if (sum >= target)
                return i;
        }

        return 255;
    };

    int minVal = 0;
    while (minVal < 256 && g_diffHist[minVal] == 0)
        minVal++;

    int maxVal = 255;
    while (maxVal >= 0 && g_diffHist[maxVal] == 0)
        maxVal--;

    double mean = 0;

    for (int i = 0; i < 256; i++)
        mean += i * static_cast<double>(g_diffHist[i]);

    mean /= total;

    std::cout << "\n";
    std::cout << "========== |Pattern-Inverse| Statistics ==========\n";
    std::cout << "Samples : " << total << '\n';
    std::cout << "Min     : " << minVal << '\n';
    std::cout << "P10     : " << percentile(0.10) << '\n';
    std::cout << "P20     : " << percentile(0.20) << '\n';
    std::cout << "P30     : " << percentile(0.30) << '\n';
    std::cout << "P90     : " << percentile(0.90) << '\n';
    std::cout << "Max     : " << maxVal << '\n';
    std::cout << "Mean    : " << mean << '\n';
    std::cout << "===============================================\n";

    g_diffHist.fill(0);
}


inline void loadCalibration(const std::string& file, cv::Mat& cameraMatrix, cv::Mat& distCoeffs)
{
    cv::FileStorage fs(file, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cout << "\nFailed to open calibration file!\n";
        return;
    }

    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;

    fs.release();

    std::cout << "\nLoaded Camera Matrix:\n" << cameraMatrix << std::endl;
    std::cout << "\nLoaded Distortion:\n" << distCoeffs << std::endl;
}

}

