#ifndef PARSEBAG_H
#define PARSBAG_H

#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

//#define MAX_LEN 65536
#define MAX_LEN 200000

#include "signalSlots.h"
#include "messageStruct.h"

enum Options {
	None_0,
	None_1,
	Message_2,
	BAGHEAD_3,
	INDEXDATA_4,
	CHUNK_5,
	CHUNKINFO_6,
	CONNECTION_7
};

class ParseBag {
public:
	virtual ~ParseBag();

	static ParseBag* getInstance() {
		if (instance == nullptr) {
			instance = new ParseBag();
		}
		return instance;
	}

	SignalSlot::Signal<void(const sensor_msgs::ImageConstPtr& msg)> notifyImage;
	SignalSlot::Signal<void(const sensor_msgs::Imu::ConstPtr& msg_in)> notifyImu;
	SignalSlot::Signal<void(const sensor_msgs::PointCloud2::ConstPtr& msg_in)> notifyPoints;

	void parseBag(const std::string& sFileName);

private:
	int readIMU(std::ifstream& ifs);
	int readImage(std::ifstream& ifs);
	int readPoint(std::ifstream& ifs);

	int readHeader(std::ifstream& ifs, bool bBagHeader = false);
	int readData(std::ifstream& ifs);

	int toInt(unsigned char* buf, int st, int len);
	int toString(unsigned char* buf, int st, int len, std::string& name, std::string& value);

	int readHeader(std::FILE* file);
private:
	ParseBag();
	static ParseBag* instance;

	unsigned char clen[4] = { 0 };
	unsigned char buffer[MAX_LEN] = { 0 };
	std::unordered_map<std::string, std::string> mapHeaderAttribute;
	std::string sHeaderName, sHeaderValue; // header attribute
	double m_imuData[37] = { 0 };
	std::vector<uchar> imgdata;
	char cintensity;
	int nintensity;

	std::vector<sensor_msgs::ImageConstPtr> vImageMsg;
	std::vector<sensor_msgs::Imu::ConstPtr> vImuMsg;
	std::vector<sensor_msgs::PointCloud2::ConstPtr> vPointsMsg;
	//std::ofstream ofs;
};

#endif