#include"camera_calib.h"


void createCheckerboard(){
    int rows = 7;       // số ô theo chiều dọc
    int cols = 9;       // số ô theo chiều ngang
    int squareSize = 100; // kích thước mỗi ô (pixel)

    int widthBoard = cols * squareSize;
    int heightBoard = rows * squareSize;

    Mat board(heightBoard, widthBoard, CV_8UC1, Scalar(255)); // tạo ảnh trắng

    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            if((i + j) % 2 == 1) // tô màu đen cho ô có tổng chỉ số lẻ
            {
                rectangle(board,
                    Point(j * squareSize, i * squareSize),
                    Point((j + 1) * squareSize, (i + 1) * squareSize),
                    Scalar(0),
                    FILLED);
            }
        }
    }
    
    imshow("Checkerboard", board);   
    imwrite("checkerboard.png", board);
}


void captureImages(int url, string folder)
{

    cout << "\n====== CONTROL ======\n";
    cout << "c : capture\n";
    cout << "ESC : quit\n";

    int width = 320;
    int height = 240;   
    
    VideoCapture cap(url);
    cap.set(CAP_PROP_BUFFERSIZE, 1); // giảm độ trễ

    if(!cap.isOpened())
    {
        cout << "\nFailed to open camera\n";
        return;
    }

    namedWindow("Phone Camera", WINDOW_NORMAL);
    resizeWindow("Phone Camera", width, height);

    Mat frame;
    int imgCount = 1;

    while(true)
    {

        cap >> frame;
        if(frame.empty()) break;

        imshow("Phone Camera", frame);
        
        char key = waitKey(30);

        // nhấn ESC để thoát
        if(key == 27)  break;

        if(key == 'c'){ 
            string filename = folder + "/image_" + to_string(imgCount) + ".png";
            imwrite(filename, frame);
            cout << "Image saved: " << filename << endl;
            imgCount++;
        }
    }

    cap.release();
    destroyAllWindows();
}


void calibrateAndSave(string folder)
{
    int width = 640;
    int height = 480;

    Size boardSize(9,7);      // số inner corners
    float squareSize = 30.0f; // mm

    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;

    vector<String> images;

    glob(folder + "/*.png", images); // lấy tất cả ảnh file.png trong folder và trả vể vector 
    if(images.empty())
    {
        cout << "No images found!\n";
        return;
    }
    cout << "Images found: " << images.size() << endl;

    // tạo tọa độ bàn cờ 3D
    vector<Point3f> objp;
    for(int i=0;i<boardSize.height;i++)
    {
        for(int j=0;j<boardSize.width;j++)
        {
            objp.push_back(Point3f(j*squareSize, i*squareSize, 0));
        }
    }

    Mat image, gray;
    Size imageSize;
    for(size_t i=0;i<images.size();i++)
    {
        image = imread(images[i]);
        if(image.empty())   continue;

        imageSize = image.size();
        cvtColor(image, gray, COLOR_BGR2GRAY);
        vector<Point2f> corners;

        bool found = findChessboardCornersSB(
            gray,
            boardSize,
            corners,
            CALIB_CB_NORMALIZE_IMAGE |
            CALIB_CB_EXHAUSTIVE |
            CALIB_CB_ACCURACY
        );
    

        if(found)
        {
            cornerSubPix(
                gray,
                corners,
                Size(11,11),
                Size(-1,-1),
                TermCriteria(
                    TermCriteria::EPS + TermCriteria::COUNT,
                    30,
                    0.001
                )
            );

            imagePoints.push_back(corners);
            objectPoints.push_back(objp);

            drawChessboardCorners(image, boardSize, corners, found);
        }

        namedWindow("Calibration", WINDOW_NORMAL);
        resizeWindow("Calibration", width, height);
        imshow("Calibration", image);
        waitKey(200);
        cout << images[i] << " -> " << found << endl;
    }

    cout << "Images detected: " << imagePoints.size() << endl;

    if(imagePoints.size() < 5)
    {
        cout << "Not enough images detected!\n";
        return;
    }

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;

    double rms = calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs
    );

    cout << "\nRMS error: " << rms << endl;

    cout << "\nCamera Matrix:\n" << cameraMatrix << endl;
    cout << "\nDistortion:\n" << distCoeffs << endl;

    // lưu file JSON
    FileStorage fs("camera_calibration.json", FileStorage::WRITE | FileStorage::FORMAT_JSON);

    if (!fs.isOpened())
    {
        cout << "Failed to open file for writing!\n";
        return;
    }

    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;

    fs.release();

    cout << "[DONE] Saved calibration to " << "camera_calibration.json" << endl;
}


void runcalibration(int url, string folder)
{

    cout << "\n====== CONTROL ======\n";
    cout << "c : capture\n";
    cout << "p : process\n";
    cout << "q : quit\n";

    string command;

    while (true) {
        
        cout << "\nCamera Command: ";
        cin >> command;

        if (command == "c")
        {
            captureImages(url, folder);
        }
        else if (command == "p")
        {
            calibrateAndSave(folder);
        }
        else if (command == "q")
        {
            cout << "Exiting calibration.\n";
            break;
        }
        else
        {
            cout << "Invalid command!" << endl;
        }
    }
}