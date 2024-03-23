#include <iostream>
#include <memory>

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/StrictMode.hpp"
#include "common/common_utils/FileSystem.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

using namespace msr::airlib;

#include "signalSlots.h"
#include "messageStruct.h"

#include <mutex>

class AirSimManager {
public:
	static std::shared_ptr<AirSimManager> getInstance() {
		if (instance == nullptr) {
			instance = std::make_shared<AirSimManager>();
		}
		return instance;
	}

	SignalSlot::Signal<void(const sensor_msgs::ImageConstPtr& msg)> notifyImage;
	SignalSlot::Signal<void(const sensor_msgs::Imu::ConstPtr& msg_in)> notifyImu;
	SignalSlot::Signal<void(const sensor_msgs::PointCloud2::ConstPtr& msg_in)> notifyPoints;

	AirSimManager();
	~AirSimManager();

	void run();

	void getImuData();
	void getImageData();
	void getLidarData();

	void lock() {
		mutex.lock();
	}
	void unlock() { mutex.unlock(); }

private:
    
    
	static std::shared_ptr<AirSimManager> instance;
	std::mutex mutex;
	msr::airlib::MultirotorRpcLibClient client;
};
