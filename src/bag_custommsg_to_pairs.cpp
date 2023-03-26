#include "include/CustomMsg.h"
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <thread>

using namespace std;

string bag_file;
string lidar_topic;
string camera_topic;
string pcd_path;
string img_path;
bool is_custom_msg;
int pcl_number;
int pcl_counter = 0;

void iterateMessagesPCL(rosbag::Bag* pBag, std::string& topic, std::vector<livox_ros_driver::CustomMsgPtr>& vMsgs);
void iterateMessagesIMG(rosbag::Bag* pBag, std::string& topic, std::vector<sensor_msgs::ImagePtr>& vMsgs);

void findCloseMessages(std::vector<livox_ros_driver::CustomMsgPtr>& vInMsgs1,
                       std::vector<sensor_msgs::ImagePtr>& vInMsgs2,
                       std::map<livox_ros_driver::CustomMsgPtr , sensor_msgs::ImagePtr>& dict);

void outputDict (std::map<livox_ros_driver::CustomMsgPtr , sensor_msgs::ImagePtr>& dict,
                 std::string& pathImg,
                 std::string& pathPcd);

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidarCamCalib");
    ros::NodeHandle nh;

    nh.param<string>("bag_file", bag_file, "");
    nh.param<string>("pcd_path", pcd_path, "");
    nh.param<string>("img_path", img_path, "");
    nh.param<string>("lidar_topic", lidar_topic, "/livox/lidar");
    nh.param<string>("camera_topic", camera_topic, "/kinect2/hd/image_color_rect");
    nh.param<bool>("is_custom_msg", is_custom_msg, false);
    nh.param<int>("pcl_number", pcl_number, 1);

    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    std::fstream file_;

    file_.open(bag_file, ios::in);
    if (!file_) {
        std::string msg = "Loading the rosbag " + bag_file + " failure";
        ROS_ERROR_STREAM(msg.c_str());
        return -1;
    }

    ROS_INFO("Loading the rosbag %s", bag_file.c_str());
    rosbag::Bag bag;
    rosbag::Bag *pBag = &bag;
    std::vector<livox_ros_driver::CustomMsgPtr> vPCL;
    std::vector<sensor_msgs::ImagePtr> vIMG;
    std::map<livox_ros_driver::CustomMsgPtr, sensor_msgs::ImagePtr> dict;

    try {
        bag.open(bag_file, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return -1;
    }

    iterateMessagesPCL(pBag, lidar_topic, vPCL);
    iterateMessagesIMG(pBag, camera_topic, vIMG);
    findCloseMessages(vPCL, vIMG, dict);
    outputDict(dict, img_path, pcd_path);

    return 0;
}

void iterateMessagesPCL(rosbag::Bag* pBag, std::string& topic, std::vector<livox_ros_driver::CustomMsgPtr>& vMsgs) {
    std::vector<string> topic_vec;
    topic_vec.push_back(topic);

    rosbag::View view(*pBag, rosbag::TopicQuery(topic_vec));

    for (const rosbag::MessageInstance &m : view) {
        livox_ros_driver::CustomMsgPtr msg = m.instantiate<livox_ros_driver::CustomMsg>(); // message type
        vMsgs.push_back(msg);
    }
    ROS_INFO("Finished iterationg PCL messages");
}

void iterateMessagesIMG(rosbag::Bag* pBag, std::string& topic, std::vector<sensor_msgs::ImagePtr>& vMsgs) {
    std::vector<string> topic_vec;
    topic_vec.push_back(topic);

    rosbag::View view(*pBag, rosbag::TopicQuery(topic_vec));

    for (const rosbag::MessageInstance &m : view) {
        sensor_msgs::ImagePtr msg = m.instantiate<sensor_msgs::Image>(); // message type
        vMsgs.push_back(msg);
    }
    ROS_INFO("Finished iterationg IMG messages");
}

void findCloseMessages(std::vector<livox_ros_driver::CustomMsgPtr>& vInMsgs1,
                       std::vector<sensor_msgs::ImagePtr>& vInMsgs2,
                       std::map<livox_ros_driver::CustomMsgPtr , sensor_msgs::ImagePtr>& dict) {
    for (auto& inMsg1: vInMsgs1) {
        for (auto& inMsg2: vInMsgs2) { // TODO: Think of long long (or leave fabs)
            uint64_t tMsg1 = inMsg1.get()->header.stamp.toNSec();
            uint64_t tMsg2 = inMsg2.get()->header.stamp.toNSec();
            uint64_t delay = tMsg1 - tMsg2;
            if (std::fabs((unsigned long long)(delay) <= 5e6)) {
                dict.insert({inMsg1, inMsg2});
                // TODO: Given a timestamp. Find a specific number of lidar msgs after this timestamp
//                std::find(vInMsgs1.begin(), vInMsgs1.end(), tMsg1);

//                std::vector<>
//                std::vector<std::vector<livox_ros_driver::CustomMsgPtr>>
            } // TODO: Change to something faster
        }
    }
    ROS_INFO("Searching for close messages is completed");
}

void outputDict (std::map<livox_ros_driver::CustomMsgPtr, sensor_msgs::ImagePtr>& dict,
                 std::string& pathImg,
                 std::string& pathPcd) {
    // PCL
    std::map<livox_ros_driver::CustomMsgPtr , sensor_msgs::ImagePtr>::iterator it;
    for (it = dict.begin(); it != dict.end(); it++) {
        pcl::PointCloud<pcl::PointXYZI> outputPCL;
        for (uint i = 0; i < it->first.get()->point_num; ++i) {
            pcl::PointXYZI p;
            p.x = it->first.get()->points[i].x;
            p.y = it->first.get()->points[i].y;
            p.z = it->first.get()->points[i].z;
            p.intensity = it->first.get()->points[i].reflectivity;
            outputPCL.points.push_back(p);
        }
        outputPCL.is_dense = false;
        outputPCL.width = outputPCL.points.size();
        outputPCL.height = 1;
        std::string pcdFile = pathPcd + std::to_string(it->first.get()->header.stamp.toSec()) + ".pcd";
        pcl::io::savePCDFileASCII(pcdFile, outputPCL);
        string msgPCL = "Sucessfully saved point cloud into pcd file: " + pcdFile;
        ROS_INFO_STREAM(msgPCL.c_str());

    // IMG
        cv_bridge::CvImagePtr img;
        try {  // TODO: Be careful with the encodings
            img = cv_bridge::toCvCopy(it->second, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        std::string imgFile = pathImg + std::to_string(it->second.get()->header.stamp.toSec()) + ".png";
        cv::imwrite(imgFile, img->image);
        string msgIMG = "Sucessfully saved image into png file: " + imgFile;
        ROS_INFO_STREAM(msgIMG.c_str());
    }
}