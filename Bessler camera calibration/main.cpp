  #include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char **argv) {

  (void)argc;
  (void)argv;

  std::vector<cv::String> fileNames;
  cv::glob("/home/albert/Desktop/Semesterprojekt3/kalibreringsbilleder_lego/Image*.bmp", fileNames, false);
  cv::Size patternSize(9, 6);
  std::vector<std::vector<cv::Point2f>> q(fileNames.size());

  // Detect feature points
  std::size_t i = 0;
  for (auto const &f : fileNames) {
    std::cout << std::string(f)  << std::endl;

    // 1. Read in the image an call cv::findChessboardCorners()
    cv::Mat img = cv::imread(f, cv::IMREAD_GRAYSCALE)  ;
    //cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    bool success = cv::findChessboardCorners(img, patternSize, q[i]);
    std::cout << "Chessboard detection success: " << success << std::endl;

    // 2. Use cv::cornerSubPix() to refine the found corner detections
    if (success) {
        cv::cornerSubPix(img, q[i], cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }

    // Display
    cv::drawChessboardCorners(img, patternSize, q[i], success);
    cv::imshow("chessboard detection", img);
    cv::waitKey(0);

    i++;
  }

  std::vector<std::vector<cv::Point3f>> Q;
  // 3. Generate checkerboard (world) coordinates Q. The board has 10 x 7
  // fields with a size of 52x52mm
  for (int i = 0; i < fileNames.size(); ++i) {
      std::vector<cv::Point3f> points;
      for (int j = 0; j < patternSize.height * patternSize.width; ++j) {
          points.push_back(cv::Point3f(j % patternSize.width * 52, j / patternSize.width * 52, 0));
      }
      Q.push_back(points);
  }

  cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
  cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
  cv::Size frameSize(1080  , 1080);


  std::cout << "Number of images: " << fileNames.size() << std::endl;
  std::cout << "Calibrating..." << std::endl;
  // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
  // and output parameters as declared above...
  float  error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, stdIntrinsics, stdExtrinsics, perViewErrors);


  std::cout << "Reprojection error = " << error << "\nK =\n"
            << K << "\nk=\n"
            << k << std::endl;

  // Precompute lens correction interpolation
  cv::Mat mapX, mapY;
  cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);

  // Show lens corrected images
  for (auto const &f : fileNames) {
    std::cout << std::string(f) << std::endl;

    cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

    cv::Mat imgUndistorted;
    // 5. Remap the image using the precomputed interpolation maps.
    cv::undistort(img, imgUndistorted, K, k);

    // Display
    cv::imshow("undistorted image", imgUndistorted);
    cv::waitKey(0);
  }

  return 0;
}




































