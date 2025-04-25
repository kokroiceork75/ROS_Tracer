#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class GestureRecognitionNode {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

public:
    GestureRecognitionNode()
        : it_(nh_) {
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &GestureRecognitionNode::imageCb, this);
        cv::namedWindow("Gesture Recognition");
    }

    ~GestureRecognitionNode() {
        cv::destroyWindow("Gesture Recognition");
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat processedImage = processImage(cv_ptr->image);
        cv::imshow("Gesture Recognition", processedImage);
        cv::waitKey(3);
    }

    cv::Mat processImage(cv::Mat& inputImage) {
        cv::Mat gray, blurred, thresh;
        cv::cvtColor(inputImage, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        cv::threshold(blurred, thresh, 60, 255, cv::THRESH_BINARY);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresh.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // Find the largest contour
        int largestContourIdx = -1;
        double largestArea = 0.0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > largestArea) {
                largestArea = area;
                largestContourIdx = i;
            }
        }

        cv::Mat drawing = inputImage.clone();
        if (largestContourIdx != -1) {
            // Approximate the contour to a polygon
            std::vector<std::vector<cv::Point>> hull(1);
            cv::convexHull(cv::Mat(contours[largestContourIdx]), hull[0], false);

            // Draw the contour and hull
            cv::drawContours(drawing, contours, largestContourIdx, cv::Scalar(0, 255, 0), 2);
            cv::drawContours(drawing, hull, 0, cv::Scalar(0, 0, 255), 3);

            // Identify and display the gesture
            std::string gesture = identifyGesture(hull[0]); // You need to implement this
            cv::putText(drawing, gesture, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
        }

        return drawing;
    }

    std::string identifyGesture(std::vector<cv::Point>& hull) {
        // This function should analyze the hull to identify the gesture.
        // As an example, let's just count the number of points in the hull.
        int points = hull.size();

        if (points < 5) {
            return "Closed Hand";
        } else {
            return "Open Hand";
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gesture_recognition_node");
    GestureRecognitionNode grn;
    ros::spin();
    return 0;
}
