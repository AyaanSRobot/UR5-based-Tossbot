#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pylon/PylonIncludes.h>
#include<unistd.h>
#include"server.h"
#include"wsg.h"
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <tuple>
#include <chrono>
#include <thread>
#include <math.h>


std::vector<cv::Point> circleCenters; // Vector to store center coordinates

using namespace std;
using namespace ur_rtde;
using namespace std::chrono;

tuple<vector<double>, vector<double>, double, double, pair<double, double>, vector<double>> returnTuple(Server s, int x_b, int y_b, int x_c, int y_c)
{
    s.sendMessage(to_string(x_b)+","+to_string(y_b)+","+to_string(x_c)+","+to_string(y_c));
    istringstream iss(s.waitForMessage());
    string token;

    vector<double> s_0;
    vector<double> q_dot;
    vector<double> qq_vector;
    double leadingAxis;
    double t_const;
    pair<double, double> ball_xy;

    // Counter to keep track of which component we are parsing
    int componentCounter = 0;

    // Loop through the tokens separated by commas
    while (std::getline(iss, token, ',')) {
        // Convert each token to a double
        double value = std::stod(token);

        // Determine which component we are currently parsing and store the value accordingly
        switch (componentCounter)
        {
        // Parsing s_0 components
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
            s_0.push_back(value);
            break;
            // Parsing q_dot components
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
            q_dot.push_back(value);
            break;
            // Parsing leadingAxis
        case 12:
            leadingAxis = value;
            break;
            // Parsing t_const
        case 13:
            t_const = value;
            break;
            // Parsing start ballXY
        case 14:
            ball_xy.first = value;
            break;
        case 15:
            ball_xy.second = value;
            break;
            // Parsing start components
        case 16:
        case 17:
        case 18:
        case 19:
        case 20:
        case 21:
            qq_vector.push_back(value);
            break;
        default:
            break;
        }

        // Increment the counter
        componentCounter++;
    }

    // Return a tuple containing all parsed components
    return std::make_tuple(s_0, q_dot, leadingAxis, t_const, ball_xy, qq_vector);
}

int main(int argc, char* argv[])
{
    // addr and port for different connections
    //----------------------------------------------------------------

    string notLanIP = "10.126.16.209"; // ip of the computer
    int portServer = 54000;

    string robotIP = "192.168.1.10"; // The ip of the robot

    string griberIP = "192.168.1.20"; // The ip of the griber
    int portGriber = 1000;
    //----------------------------------------------------------------
    RTDEControlInterface rtde_control(robotIP);
    RTDEReceiveInterface rtde_receive(robotIP);
    WSG gripper(portGriber, griberIP);

    cv::Point cupCenter;
    cv::Point ballCenter;
    double ball_z = 196;//These are default z cord for cup and ball
    double cup_z = 298;//--

    std::vector<double> Home;
    Home = {26.6*0.001, -223 *0.001, 262.8*0.001, 2.641, -1.771, 0};

    cout<<"Run the matlab program to convert coordinates"<<endl;
    Server s(portServer, notLanIP);
    s.waitForMessage();

    int myExposure = 30000;

    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.
        Pylon::CInstantCamera camera( Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        // Get a camera nodemap in order to access camera parameters.
        GenApi::INodeMap& nodemap= camera.GetNodeMap();

        // Open the camera before accessing any parameters.
        camera.Open();
        // Create pointers to access the camera Width and Height parameters.
        GenApi::CIntegerPtr width= nodemap.GetNode("Width");
        GenApi::CIntegerPtr height= nodemap.GetNode("Height");

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        //camera.MaxNumBuffer = 5;

        // Create a pylon ImageFormatConverter object.
        Pylon::CImageFormatConverter formatConverter;
        // Specify the output pixel format.
        formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
        // Create a PylonImage that will be used to create OpenCV images later.
        Pylon::CPylonImage pylonImage;

        // Create an OpenCV image.
        cv::Mat openCvImage;

        // Set exposure to manual
        GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        if ( GenApi::IsWritable( exposureAuto)){
            exposureAuto->FromString("Off");
            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
        if(exposureTime.IsValid()) {
            if(myExposure >= exposureTime->GetMin() && myExposure <= exposureTime->GetMax()) {
                exposureTime->SetValue(myExposure);
            }else {
                exposureTime->SetValue(exposureTime->GetMin());
                std::cout << ">> Exposure has been set with the minimum available value." << std::endl;
                std::cout << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
            }
        }else {

            std::cout << ">> Failed to set exposure value." << std::endl;
            return false;
        }
        std::cout << "New exposure: " << exposureTime->GetValue() << std::endl;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        // image grabbing loop
        int frame = 1;
        while ( camera.IsGrabbing())
        {
            cout<<"Taking picture"<<endl;

            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {

                // Convert the grabbed buffer to a pylon image.
                formatConverter.Convert(pylonImage, ptrGrabResult);

                // Create an OpenCV image from a pylon image.
                openCvImage= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

                // Check if the image is loaded successfully
                if (openCvImage.empty()) {
                    std::cerr << "Error: Couldn't read the image." << std::endl;
                    return -1;
                }

                // Define the intrinsic camera matrix for table calibration (3x3)
                cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
                                         1186.613, 0, 759.47504,
                                         0, 1186.24, 575.23102,
                                         0, 0, 1
                                         );

                // Define the distortion coefficients (1x5)
                cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) << -0.25831, 0.313, -0.000590848, 0.00091597, -0.721834);

                // Define the intrinsic camera matrix for lego calibration (3x3)
                cv::Mat camera_matrix_lego = (cv::Mat_<double>(3, 3) <<
                                              1186.613, 0, 759.47504,
                                              0, 1186.24, 575.23102,
                                              0, 0, 1
                                              );

                // Define the distortion coefficients (1x5)
                cv::Mat distortion_coefficients_lego = (cv::Mat_<double>(1, 5) << -0.25831, 0.313, -0.000590848, 0.00091597, -0.721834);

                // Undistort the image and name new pic "image"
                cv::Mat undistortedImage;
                cv::undistort(openCvImage, undistortedImage, camera_matrix_lego, distortion_coefficients_lego);

                // Define the source and destination points for the perspective transformation
                cv::Point2f src[4];
                cv::Point2f dst[4];

                // Define your source and destination points here
                src[0] = cv::Point2f(588 - 1, 610 - 1);
                src[1] = cv::Point2f(900 - 1, 610 - 1);
                src[2] = cv::Point2f(900 - 1, 744 - 1);
                src[3] = cv::Point2f(632 - 1, 746 - 1);

                dst[0] = cv::Point2f(50 -1, 150 -1);
                dst[1] = cv::Point2f(400 -1, 150 -1);
                dst[2] = cv::Point2f(400 -1, 300 -1);
                dst[3] = cv::Point2f(100 -1, 300 -1);

                // Calculate the perspective transformation matrix
                cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(src, dst);

                // Apply the perspective transformation
                cv::Mat transformedImage;
                cv::warpPerspective(undistortedImage, transformedImage, perspectiveMatrix, cv::Size(450, 550));

                // Convert to gray-scale
                cv::Mat gray;
                cv::cvtColor(transformedImage, gray, cv::COLOR_BGR2GRAY);

                // Blur the image to reduce noise
                cv::Mat img_blur;
                //medianBlur(gray, img_blur, 5);
                cv::GaussianBlur(gray, img_blur, cv::Size(9, 9), 2, 2);

                // Create a vector for detected circles
                std::vector<cv::Vec3f> circles;

                // Apply Hough Transform
                cv::HoughCircles(img_blur, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 8, 200, 30, 12, 50);

                // Get coords for ball and cup center
                if(cvRound(circles[0][2])>cvRound(circles[1][2]))// The bigger radius belongs to the cup
                {
                    cupCenter = cv::Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
                    ballCenter = cv::Point(cvRound(circles[1][0]), cvRound(circles[1][1]));
                    cout << "cup x = " << cupCenter.x << " cup y = " << cupCenter.y << endl;
                    cout << "ball x = " << ballCenter.x << " ball y = " << ballCenter.y << endl;
                }
                else
                {
                    cupCenter = cv::Point(cvRound(circles[1][0]), cvRound(circles[1][1]));
                    ballCenter = cv::Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
                    cout << "cup x = " << cupCenter.x << " cup y = " << cupCenter.y << endl;
                    cout << "ball x = " << ballCenter.x << " ball y = " << ballCenter.y << endl;
                }


                // Detect key press and quit if 'q' is pressed
                cout<<"Click q to quit program or t to throw ball"<<endl;
                cout<<"If another button is pressed a new picure will be taken"<<endl;
                string keyPressed;
                cin>>keyPressed;
                if(keyPressed == "q")//quit
                {
                    std::cout << "Shutting down camera..." << std::endl;
                    camera.Close();
                    std::cout << "Camera successfully closed." << std::endl;
                    s.sendMessage("stop_matlab");//Stop the matlab program
                    gripper.bye();
                    s.closeSocket();
                    rtde_control.stopScript();
                    break;
                }
                else if(keyPressed == "t")
                {
                    cout<<"Throwing!"<<endl;

                    pair <double,double> ball_coord;

                    tuple<vector<double>, vector<double>, double, double, pair<double, double>, vector<double>> values =
                            returnTuple(s,ballCenter.x,ballCenter.y,cupCenter.x,cupCenter.y);

                    ball_coord.first = get<4>(values).first;
                    ball_coord.second = get<4>(values).second;
                    vector<double> s_0 = get<0>(values);
                    vector<double> q_dot = get<1>(values);
                    vector<double> qq_vector = get<5>(values);
                    double leadingAxis = get<2>(values);
                    double t_const = get<3>(values);

                    // Output the values
                    std::cout << "ball x = " << ball_coord.first << std::endl;
                    std::cout << "ball y = " << ball_coord.second << std::endl;
                    std::cout << "s_0: ";
                    for (const auto &val : s_0) {
                        std::cout << val << " ";
                    }
                    std::cout << std::endl;

                    std::cout << "q_dot: ";
                    for (const auto &val : q_dot) {
                        std::cout << val << " ";
                    }
                    std::cout << std::endl;

                    std::cout << "start: ";
                    for (const auto &val : qq_vector) {
                        std::cout << val << " ";
                    }
                    std::cout << std::endl;

                    std::cout << "leadingAxis: " << leadingAxis << std::endl;
                    std::cout << "t_const: " << t_const << std::endl;


                    int time = -0.0004*pow(cupCenter.y,2)+0.4413*cupCenter.y + 645.92;

                    rtde_control.moveL(Home, 1, 0.5);
                    rtde_control.moveL({ball_coord.first*0.001, ball_coord.second*0.001, 0.2, 2.641, -1.771, 0}, 1, 0.5);
                    gripper.grip();
                    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

                    rtde_control.moveL({ball_coord.first*0.001, ball_coord.second*0.001, 0.3, 2.641, -1.771, 0}, 1, 0.5);

                    rtde_control.moveL(Home, 1, 0.5);

                    rtde_control.moveJ(qq_vector, 1, 0.5);

                    rtde_control.moveJ(s_0);
                    rtde_control.speedJ(q_dot, leadingAxis);

                    std::this_thread::sleep_for(std::chrono::milliseconds(time));
                    gripper.release();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    rtde_control.speedStop(10);

                    rtde_control.moveL(Home, 1, 0.5);
                    gripper.home();

                }

                frame++;
            }
            else
            {
                std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            }
        }

    }

    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        exitCode = 1;
    }
    return exitCode;
}
