#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::cout << "Testing OpenCV window..." << std::endl;
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 255, 0));
    cv::putText(img, "Test Window", cv::Point(200, 240), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 3);
    
    cv::namedWindow("Test Window", cv::WINDOW_NORMAL);
    cv::imshow("Test Window", img);
    
    std::cout << "Window created and displayed. Press any key to exit..." << std::endl;
    
    int key = cv::waitKey(0);
    std::cout << "Key pressed: " << key << std::endl;
    
    cv::destroyAllWindows();
    return 0;
}
