#ifndef CaMERA_CALIB_H
#define CaMERA_CALIB_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <filesystem>
#include <fstream>


using namespace cv;
using namespace std;
namespace fs = std::filesystem;



void createCheckerboard();
void captureImages(int url, string folder);
void calibrateAndSave(string folder);
void runcalibration(int url, string folder);


#endif