#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "smoke.hpp"
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
class SmokeDetector
{
public:
    SmokeDetector(ros::NodeHandle& nh)
        : it_(nh), start_detect(false)
    {
        // 获取参数
        nh.param("image_topic", image_topic_, std::string("/camera/image"));
        nh.param("onnx_path", onnx_path_, std::string(""));
        nh.param("engine_path", engine_path_, std::string(""));
        nh.param<std::vector<float>>("camera/camera_matrix", camera_matrix, std::vector<float>());
        nh.param<std::vector<float>>("camera/distort", distort, std::vector<float>());
        nh.param<int>("camera/camera_width", imagewidth, 1920);
        nh.param<int>("camera/camera_height", imageheight, 1080);
        
        // 订阅图像话题
        image_sub_ = it_.subscribe(image_topic_, 1, &SmokeDetector::imageCallback, this);
        image_pub_ = it_.advertise("/3d_detector/image", 1);
        publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/3d_detector/world", 1);
        
        float scalew = 1242 / imagewidth;
        float scaleh = 375 / imageheight;
        // 初始化 SMOKE 模型
        intrinsic_ = (cv::Mat_<float>(3, 3) << 
            camera_matrix[0]*scalew, 0.0, camera_matrix[2]*scalew, 
            0.0,camera_matrix[4]*scaleh, camera_matrix[5]*scaleh, 
            0.0, 0.0, 1.0);

        intri_ = cv::Mat(3, 3, CV_32F, camera_matrix.data()).clone();
        distortion_coeffs_ = cv::Mat(5,1, CV_32F, distort.data()).clone();
        
        smoke_.prepare(intrinsic_);
        std::ifstream f(engine_path_.c_str());
        bool engine_file_exist = f.good();
        if (engine_file_exist)
        {
            smoke_.LoadEngine(engine_path_);
            start_detect = true;
        }
        else
        {
            smoke_.LoadOnnx(onnx_path_);
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // 将 ROS 图像消息转换为 OpenCV 格式
        if(start_detect){
            cv::Mat kitti_img;
            try
            {
                kitti_img = cv_bridge::toCvShare(msg, "bgr8")->image;
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                return;
            }
            cv::Mat undistorted_image;
            cv::undistort(kitti_img, undistorted_image, intri_, distortion_coeffs_);
            cv::Mat resized_image;
            cv::resize(undistorted_image, resized_image, cv::Size(1242, 375));
            // 进行检测
            smoke_.Detect(resized_image);
            // 发布话题
            cv::Mat pub_img = smoke_.img_resize;
            sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(msg->header, "bgr8", pub_img).toImageMsg();
            image_pub_.publish(output_msg);
            visualization_msgs::MarkerArray markers;
            int marker_id = 0;  // 初始化marker的ID
            for(const auto& bbox : smoke_.detector_result){
                visualization_msgs::Marker marker;
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.header = msg->header;
                marker.header.frame_id = "cam";
                marker.action = visualization_msgs::Marker::ADD;
                marker.id = marker_id++;
                geometry_msgs::Point pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8;
                geometry_msgs::Quaternion orientation;
                tf2::Quaternion myQuaternion;
                marker.pose.position.x = bbox.x;
                marker.pose.position.y = bbox.y;
                marker.pose.position.z = bbox.z;

                myQuaternion.setRPY(0, 0, bbox.angle);
                orientation = tf2::toMsg(myQuaternion);
                marker.pose.orientation = orientation;

                pos1.x = bbox.length / 2;
                pos1.y = bbox.width / 2;
                pos1.z = bbox.height / 2;

                pos2.x = bbox.length / 2;
                pos2.y = bbox.width / 2;
                pos2.z = -bbox.height / 2;

                pos3.x = bbox.length / 2;
                pos3.y = -bbox.width / 2;
                pos3.z = -bbox.height / 2;

                pos4.x = bbox.length / 2;
                pos4.y = -bbox.width / 2;
                pos4.z = bbox.height / 2;

                pos5.x = -bbox.length / 2;
                pos5.y = -bbox.width / 2;
                pos5.z = bbox.height / 2;

                pos6.x = -bbox.length / 2;
                pos6.y = -bbox.width / 2;
                pos6.z = -bbox.height / 2;

                pos7.x = -bbox.length / 2;
                pos7.y = bbox.width / 2;
                pos7.z = -bbox.height / 2;

                pos8.x = -bbox.length / 2;
                pos8.y = bbox.width / 2;
                pos8.z = bbox.height / 2;
                marker.points.push_back(pos1);
                marker.points.push_back(pos2);
                marker.points.push_back(pos3);
                marker.points.push_back(pos4);
                marker.points.push_back(pos5);
                marker.points.push_back(pos6);
                marker.points.push_back(pos7);
                marker.points.push_back(pos8);
                marker.points.push_back(pos1);
                marker.points.push_back(pos4);
                marker.points.push_back(pos3);
                marker.points.push_back(pos6);
                marker.points.push_back(pos5);
                marker.points.push_back(pos8);
                marker.points.push_back(pos7);
                marker.points.push_back(pos2);
                switch (bbox.class_id)
                {
                case 0: //Vehicle
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    break;
                case 1: //Pedestrian
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    break;
                case 2: //Cyclist
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    break;
                }
                marker.scale.x = 0.1;
                marker.color.a = 1.0;
                marker.lifetime.fromSec(0.1);
                markers.markers.push_back(marker);
            }
            publisher_.publish(markers);
        }

    }

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher publisher_;
    SMOKE smoke_;
    cv::Mat intrinsic_;
    std::string image_topic_, onnx_path_, engine_path_;
    cv::Mat intri_, distortion_coeffs_;
    std::vector<float> camera_matrix;
    std::vector<float> distort;
    int imagewidth;
    int imageheight;
    bool start_detect;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smoke_detector");
    ros::NodeHandle nh;

    SmokeDetector detector(nh);

    ros::spin();
    return 0;
}
