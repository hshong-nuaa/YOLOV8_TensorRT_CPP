#include <ros/ros.h>
#include "yolov8.h"
#include "cmd_line_util.h"
#include <opencv2/cudaimgproc.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <mutex>
class Segment {
public:
    Segment(ros::NodeHandle n,std::string config_path);
    void ImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void process();
    bool loadParaments(const std::string &yaml_file);
    std::thread Process;
private:
    std::mutex m_buf;
    ros::Subscriber img_sub;
    ros::Publisher img_pub;
    YoloV8Config config;
    std::string onnxModelPath;
    std::queue<sensor_msgs::Image::ConstPtr> img_queue;
    std::string IMAGE_SUB_TOPIC,SEGMENT_PUB_TOPIC;
};

void Segment::ImageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    m_buf.lock();
    img_queue.push(msg);
    m_buf.unlock();
}
Segment::Segment(ros::NodeHandle n,std::string config_path)
{
    if(loadParaments(config_path))
    {
        img_pub=n.advertise<sensor_msgs::Image>(SEGMENT_PUB_TOPIC,1000);
        img_sub=n.subscribe<sensor_msgs::Image>(IMAGE_SUB_TOPIC,1000, [this](const sensor_msgs::Image ::ConstPtr &msg) { ImageCallback(msg); });
        Process=std::thread(std::bind(&Segment::process,this));
    }
}

void Segment::process()
{
    YoloV8 yoloV8(onnxModelPath,config);
    ROS_INFO("Segment Thread Start!");
    while (1)
    {
        if(img_queue.empty())
            continue;
        m_buf.lock();
        sensor_msgs::Image::ConstPtr img_msg=img_queue.front();
        img_queue.pop();
        m_buf.unlock();
        cv_bridge::CvImageConstPtr ptr;
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = ptr->image.clone();
        auto t1=std::chrono::high_resolution_clock::now();
        // Run inference
        const auto objects = yoloV8.detectObjects(img);
        cv::Mat mask=objects.data()->boxMask;
        yoloV8.drawObjectLabels(img, objects);
        auto t2=std::chrono::high_resolution_clock::now();
        double t=std::chrono::duration<double>(t2 - t1).count();
        cv::putText(img,std::to_string(1.0/t)+"FPS" , cv::Point (5,50 ), cv::FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2);
        sensor_msgs::ImagePtr msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        msg_ptr->header.stamp=img_msg->header.stamp;
        img_pub.publish(msg_ptr);
    }
}
bool Segment::loadParaments(const std::string &yaml_file)
{
    auto yaml = YAML::LoadFile(yaml_file);
    try
    {
        onnxModelPath=yaml["onnx_model_path"].as<std::string>();
        IMAGE_SUB_TOPIC=yaml["image_topic"].as<std::string>();
        SEGMENT_PUB_TOPIC=yaml["publish_topic"].as<std::string>();
    }
    catch (...) {
        ROS_ERROR("bad conversion") ;
        return false;
    }
    return true;
}