#include "daheng_camera_driver/daheng_camera.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("daheng_camera_publisher") {
        // 参数声明[8](@ref)
        this->declare_parameter("frame_rate", 30);
        this->declare_parameter("exposure_time", 10000.0);
        this->declare_parameter("gain", 10.0);
        
        // 初始化相机
        camera_ = std::make_unique<DahengCamera>(this);
        if(!camera_->open()) {
            RCLCPP_FATAL(get_logger(), "无法打开相机");
            rclcpp::shutdown();
            return;
        }
        
        // 设置相机参数
        double exposure = this->get_parameter("exposure_time").as_double();
        double gain = this->get_parameter("gain").as_double();
        camera_->set_exposure(exposure);
        camera_->set_gain(gain);
        
        if(!camera_->start_acquisition()) {
            RCLCPP_FATAL(get_logger(), "无法开始采集");
            rclcpp::shutdown();
            return;
        }
        
        // 创建图像发布者
        image_pub_ = image_transport::create_publisher(this, "camera/image");
        
        // 创建定时器采集图像
        int frame_rate = this->get_parameter("frame_rate").as_int();
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000/frame_rate),
            std::bind(&ImagePublisher::capture_and_publish, this)
        );
        
        RCLCPP_INFO(get_logger(), "大恒相机节点已启动");
    }
    
    ~ImagePublisher() {
        camera_->stop_acquisition();
        camera_->close();
    }

private:
    void capture_and_publish() {
        cv::Mat frame;
        if(camera_->capture(frame)) {
            auto msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), 
                frame.channels() == 1 ? "mono8" : "rgb8", 
                frame
            ).toImageMsg();
            
            msg->header.stamp = this->now();
            msg->header.frame_id = "camera_optical_frame";
            image_pub_.publish(msg);
        }
    }
    
    std::unique_ptr<DahengCamera> camera_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}