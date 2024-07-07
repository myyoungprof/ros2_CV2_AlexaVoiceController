#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

class CameraPublisher: public rclcpp::Node
{

    public:
        int cam_id;
        cv::VideoCapture cap;

        //constructor
        CameraPublisher():Node("camera_publisher")
        {

            //parameters
            this -> declare_parameter("cam_id", 0); //device_id for webcam
            this -> declare_parameter("camera_img_topic", "/camera/color/image_raw"); //frame publish topic name

            //open the video stream
            this -> get_parameter("cam_id", cam_id);
            cap = open_stream(cam_id);

            //create the image publisher and timer
            std::string camera_publish_topic_name;
            this -> get_parameter("camera_img_topic", camera_publish_topic_name);
            _image_publisher_ = this -> create_publisher<sensor_msgs::msg::Image>(camera_publish_topic_name, 1);
            _image_timer_ = this -> create_wall_timer(0.03s, std::bind(&CameraPublisher::publish_frame, this));

        }

        //Opens the camera stream and sets to high resolution.
        cv::VideoCapture open_stream(int cam_id)
        {
            cap.open(cam_id, cv::CAP_V4L2); //CAP_V4L2 is linux only.
            if (!cap.isOpened())
            {   
                RCLCPP_ERROR(this->get_logger(), "Camera could not be opened on device id: '%i'", cam_id);
                exit(0);
            }

            RCLCPP_INFO(this->get_logger(), "Camera opened on device id: '%i'", cam_id);

            //set resolution
            cap.set(3, 1920);
            cap.set(4, 1080);

            return cap;
        }


    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher_;
        rclcpp::TimerBase::SharedPtr _image_timer_;

        void publish_frame()
        {
            //read camera frame
            cv::Mat frame;
            cap.read(frame);
            if (frame.empty()){
                RCLCPP_WARN(this->get_logger(), "Frame data is emtpy");
                return;
            }

            //create ROS2 messages
            sensor_msgs::msg::Image _img_msg;
            std_msgs::msg::Header _header;
            cv_bridge::CvImage _cv_bridge;
            _header.stamp = this->get_clock() -> now();
            _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, frame);
            _cv_bridge.toImageMsg(_img_msg);

            //publish
            _image_publisher_ -> publish(_img_msg);
        }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}