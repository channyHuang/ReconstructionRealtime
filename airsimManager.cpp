#include "airsimManager.h"

#include <stdio.h>
#include <chrono>
#include <thread>
#include <future>
#include <iostream>
#include <fstream>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

std::shared_ptr<AirSimManager> AirSimManager::instance = nullptr;

AirSimManager::AirSimManager() {
}

AirSimManager::~AirSimManager() {
}

void AirSimManager::getImuData() {
    lock();
    auto imu_data = client.getImuData();
    unlock();

    sensor_msgs::Imu::ConstPtr msg = std::make_shared<sensor_msgs::Imu::StImu>();
    msg->header.stamp.sec = imu_data.time_stamp / 1000000000;
    msg->header.stamp.nsec = imu_data.time_stamp % 1000000000;
    msg->angular_velocity.x() = imu_data.angular_velocity[0];
    msg->angular_velocity.y() = imu_data.angular_velocity[1];
    msg->angular_velocity.z() = imu_data.angular_velocity[2];
    msg->linear_acceleration.x() = imu_data.linear_acceleration[0];
    msg->linear_acceleration.y() = imu_data.linear_acceleration[1];
    msg->linear_acceleration.z() = imu_data.linear_acceleration[2];

    //std::cout << msg->header.stamp.toSec() << " " << msg->angular_velocity.x() << " " << msg->angular_velocity.y() << " " << msg->angular_velocity.z() << " " << msg->linear_acceleration.x() << " " << msg->linear_acceleration.y() << " " << msg->linear_acceleration.z() << std::endl;
    notifyImu(msg);
}

void AirSimManager::getImageData() {
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    lock();
    const std::vector<ImageRequest> request{ ImageRequest("bottom_center", ImageType::Scene) };
    const std::vector<ImageResponse>& response = client.simGetImages(request, "UAV201");
    unlock();
    if (response.size()) {
        for (const ImageResponse& image_info : response) {
            if (image_info.image_data_uint8.size() <= 0) {
                std::cout << "info: " << image_info.width << " " << image_info.height << " " << image_info.compress << std::endl;
                continue;
            }

            sensor_msgs::ImageConstPtr msg = std::make_shared<sensor_msgs::Image::StImage>();
            msg->header.stamp.sec = image_info.time_stamp / 1000000000;
            msg->header.stamp.nsec = image_info.time_stamp % 1000000000;
            msg->imageView = cv::imdecode(image_info.image_data_uint8, cv::IMREAD_COLOR);
            if (msg->imageView.empty()) {
                std::cout << "info: image is empty" << std::endl;
                continue;
            }
            //cv::imshow("image", msg->imageView);
            //cv::waitKey(1);
            notifyImage(msg);
        }
    }

    //auto info = client.simGetCameraInfo("front_center", "UAV201");
    //std::cout << "----------proj------------" << std::endl;
    //for (int i = 0; i < 4; ++i) {
    //    for (int j = 0; j < 4; ++j) {
    //        std::cout << " " << info.proj_mat.matrix[i][j];
    //    }
    //    std::cout << std::endl;
    //}
    //std::cout << "----------proj end------------" << std::endl;
}

void AirSimManager::getLidarData() {
    lock();
    msr::airlib::LidarData lidarData = client.getLidarData("MyLidar1", "UAV201");
    unlock();

    sensor_msgs::PointCloud2::Ptr msg = std::make_shared<sensor_msgs::PointCloud2::StPointCloud>();
    msg->header.stamp.sec = lidarData.time_stamp / 1000000000;
    msg->header.stamp.nsec = lidarData.time_stamp % 1000000000;
    int len = lidarData.point_cloud.size();

    for (int i = 0; i < len; i += 3) { 
        //pcl::PointXYZINormal pt(lidarData.point_cloud[i], lidarData.point_cloud[i + 1], lidarData.point_cloud[i + 2]);;
        
        //double frame_dis = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        //if (frame_dis > 5)
        //{
        //    continue;
        //}
        // pt.intensity = 1.0f;
        // pt.curvature = 0;

        // msg->pcl_pc.push_back(pt);
    }
    if (msg->pcl_pc.size() <= 0) {
        std::cout << __FUNCTION__ << " " << " has no point" << std::endl;
        return;
    }
    notifyPoints(msg);
}

void getGPSData(msr::airlib::MultirotorRpcLibClient& client) {
    auto gps_data = client.getGpsData();
    std::cout << "GPS data \n"
        << "gps_data.time_stamp \t" << gps_data.time_stamp << std::endl
        << "gps_data.gnss.time_utc \t" << gps_data.gnss.time_utc << std::endl
        << "gps_data.gnss.geo_point \t" << gps_data.gnss.geo_point << std::endl
        << "gps_data.gnss.eph \t" << gps_data.gnss.eph << std::endl
        << "gps_data.gnss.epv \t" << gps_data.gnss.epv << std::endl
        << "gps_data.gnss.velocity \t" << gps_data.gnss.velocity << std::endl
        << "gps_data.gnss.fix_type \t" << gps_data.gnss.fix_type << std::endl;
}

void getPosition(msr::airlib::MultirotorRpcLibClient &client) {
    auto position = client.getMultirotorState().getPosition();
    std::cout << "get position " << position.x() << " " << position.y() << " " << position.z() << std::endl;
}


struct GetDataThread {
    explicit GetDataThread(AirSimManager* manager, int type = 1) {
        pManager = manager;
        mType = type;
    }

    void run() {
        mThread = std::thread([this]() {
            this->threadLoop(this->mExitSignal.get_future());
            });
    }

    bool join() {
        if (mThread.joinable()) {
            mThread.join();
            return true;
        }
        return false;
    }

    bool detach() {
        if (mThread.joinable()) {
            mThread.detach();
            return true;
        }
        return false;
    }

    bool isStoped() {
        return mStoped;
    }

    void stop() {
        mStoped = true;
        mExitSignal.set_value();
    }

private:
    int mType = 0;
    bool mStoped = false;
    std::thread mThread;
    std::promise<void> mExitSignal;
    AirSimManager* pManager = nullptr;

    void threadLoop(std::future<void> exitListener) {
        do {
            switch (mType) {
            case 1:
                pManager->getImageData();
                //std::this_thread::sleep_for(std::chrono::microseconds(8));
                break;
            case 2:
                pManager->getLidarData();
                //std::this_thread::sleep_for(std::chrono::microseconds(15));
                break;
            default:
                pManager->getImuData();
                break;
            }
        } while (exitListener.wait_for(std::chrono::microseconds(1)) == std::future_status::timeout);
    }
};

void AirSimManager::run() {
    client.confirmConnection();
    client.enableApiControl(true);
    bool ret = client.armDisarm(true);
    client.takeoffAsync()->waitOnLastTask();
    std::vector<GetDataThread> getDataThreads;
    for (int i = 0; i < 3; ++i) {
        getDataThreads.push_back(GetDataThread(this, i));
    }

    std::cout << "run start " << std::endl;
    for (auto& t : getDataThreads) {
        t.run();
    }
    float velocity = .5f;
    auto rotate = [&](float yaw = 0) {
        client.rotateToYawAsync(yaw)->waitOnLastTask();
        std::this_thread::sleep_for(std::chrono::seconds(5));
        };

    std::this_thread::sleep_for(std::chrono::seconds(30));
    try {
        Vector3r origin = client.getMultirotorState().getPosition();
        float height = origin.z();
        vector<Vector3r> path = {
            Vector3r(0, 0, height)
        };

        int curx = 0, cury = 0;

        for (int y = 5; y >= -20; y -= 3) {
            rotate(cury > y ? -90 : 90);
            client.moveToPositionAsync(curx, y, height, velocity, Utils::max<float>(), DrivetrainType::ForwardOnly, YawMode(false, 0))->waitOnLastTask();
            std::this_thread::sleep_for(std::chrono::seconds(5));
            cury = y;
            rotate(curx > 0 ? 180 : 0);
            curx = 15 - curx;
            client.moveToPositionAsync(curx, y, height, velocity, Utils::max<float>(), DrivetrainType::ForwardOnly, YawMode(false, 0))->waitOnLastTask();
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

        for (auto& t : getDataThreads) {
            t.stop();
        }
    }
    catch (rpc::rpc_error& e) {
        std::cout << "Exception " << e.get_error().as<std::string>() << std::endl;
    }

    for (auto& t : getDataThreads) {
        t.join();
    }

    client.landAsync()->waitOnLastTask();
    client.armDisarm(false);
    client.enableApiControl(false);

    std::cout << "control end " << std::endl;
}