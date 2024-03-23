#include "parseBag.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ffmpeg-filter.h"

ParseBag* ParseBag::instance = nullptr;

ParseBag::ParseBag() {
    memset(clen, 0, 4);
    memset(buffer, 0, MAX_LEN);
    mapHeaderAttribute.clear();

    //ofs.open("imu.txt");
}

ParseBag::~ParseBag() {
    //ofs.close();
}

void ParseBag::parseBag(const std::string& sFileName) {
    std::ifstream ifs(sFileName, std::ios::binary | std::ios::in);
    std::string line;
    std::getline(ifs, line);
    std::cout << line << std::endl;

    // bag header record 
    readHeader(ifs, true);

    // for each record
    while (!ifs.eof()) {
        // header
        int res = readHeader(ifs, false);
        if (res < 0) {
            std::cout << "read header len <= 0" << std::endl;
            break;
        }
        // data
        switch (std::stoi(mapHeaderAttribute["op"])) {
        case 2:
        {
            switch (std::stoi(mapHeaderAttribute["conn"])) {
            case 0: // IMU
                readIMU(ifs);
                break;
            case 1: // Image
                readImage(ifs);
                break;
            case 2: // PointCloud
                readPoint(ifs);
                break;
            default:
                readData(ifs);
                break;
            }
        }
        break;
        case 5:
        {
            memset(clen, 0, 4);
            ifs.read((char*)&clen, sizeof(char) * 4);
            int datalen = toInt(clen, 0, 4);

            int curlen = 0;
            while (curlen < datalen) {
                curlen += readHeader(ifs);

                if (std::stoi(mapHeaderAttribute["op"]) == 7) {
                    curlen += readHeader(ifs);
                }
                else if (std::stoi(mapHeaderAttribute["op"]) == 2) {
                    switch (std::stoi(mapHeaderAttribute["conn"])) {
                    case 0: // IMU
                        curlen += readIMU(ifs);
                        break;
                    case 1: // Image
                        curlen += readImage(ifs);
                        break;
                    case 2: // PointCloud
                        curlen += readPoint(ifs);
                        break;
                    default:
                        curlen += readData(ifs);
                        break;
                    }
                }
                else {
                    curlen += readData(ifs);
                }
            }
        }
        break;
        case 7:
        {
            readHeader(ifs);
        }
        break;
        default:
            readData(ifs);
            break;
        }
    }
    ifs.close();
}

int ParseBag::toInt(unsigned char* buf, int st, int len) {
    int res = 0;
    int base = 1;
    for (int i = 0; i < len; ++i) {
        res = res + buf[i + st] * base;
        base *= 256;
    }
    return res;
}

int ParseBag::toString(unsigned char* buf, int st, int len, std::string& name, std::string& value) {
    int curLen = 0;
    char cname[5] = { 0 };
    while (curLen < len) {
        if (buf[st + curLen] == '=') {
            curLen++;
            break;
        }
        if (curLen >= 5) break;
        cname[curLen] = buf[st + curLen];
        curLen++;
    }
    name = std::string(cname);
    if ((std::strcmp(name.c_str(), "op") != 0) && (std::strcmp(name.c_str(), "conn") != 0)) return 0;
    int val = 0;
    int base = 1;
    while (curLen < len) {
        if (buf[st + curLen] == 0) break;
        val = val + buf[st + curLen] * base;
        base *= 256;
        curLen++;
    }

    return val;
}

//int ParseBag::toString(unsigned char* buf, int st, int len, std::string& name, std::string& value) {
//    char cname[4096] = { 0 }, cval[4096] = { 0 };
//    bool isval = false, isnum = true;
//    int base = 1;
//    int val = 0;
//    int i = 0;
//    int idx = 0;
//    for (i = 0; i < len; ++i) {
//        if (buf[st + i] == '=') {
//            break;
//        }
//        cname[i] = buf[st + i];
//        if (!((buf[st + i] >= 'a' && buf[st + i] <= 'z') || (buf[st + i] >= 'A' && buf[st + i] <= 'Z')
//            || (buf[st + i] == '_')
//            || (buf[st + i] >= '0' && buf[st + i] <= '9'))) {
//            std::cout << "error " << std::endl;
//        }
//    }
//    name = std::string(cname);
//    std::unordered_set<std::string> setHeaderString = { "compression", "topic", "callerid", "md5sum", "message_definition", "type" };
//    if (setHeaderString.find(name) != setHeaderString.end()) {
//        isnum = false;
//        val = -1;
//    }
//
//    for (++i; i < len; ++i) {
//        if (isnum) {
//            if (buf[st + i] == 0 || buf[st + i] == '0x20') break;
//            val = val + buf[st + i] * base;
//            base *= 256;
//        }
//        else {
//            cval[idx++] = buf[st + i];
//        }
//    }
//
//    value = std::string(cval);
//    return val;
//}

int ParseBag::readIMU(std::ifstream& ifs) {
    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4);
    int datalen = toInt(clen, 0, 4);

    sensor_msgs::Imu::ConstPtr msg = std::make_shared<sensor_msgs::Imu::StImu>();

    // header
    ifs.read((char*)&buffer, sizeof(char) * 4); // seq
    //ifs.read((char*)&buffer, sizeof(char) * 8); // time
    //ifs.read((char*)&msg->header.seq, sizeof(char) * 4);
    ifs.read((char*)&msg->header.stamp.sec, sizeof(char) * 4);
    ifs.read((char*)&msg->header.stamp.nsec, sizeof(char) * 4);

    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4); // frame_id len
    int frameIdLen = toInt(clen, 0, 4);
    memset(buffer, 0, MAX_LEN);
    ifs.read((char*)&buffer, sizeof(char) * frameIdLen); // frame_id

    // data: orientation (quat), orientationCov (Matrix3x3), angularVel (Vec3), angularCov (Matrix3x3), linearAcc (Vec3), linearCov (Matrix3x3)
    for (int i = 0; i < 37; ++i) {
        ifs.read((char*)&(m_imuData[i]), sizeof(char) * 8);
    }

    msg->angular_velocity.x() = m_imuData[13];
    msg->angular_velocity.y() = m_imuData[14];
    msg->angular_velocity.z() = m_imuData[15];
    msg->linear_acceleration.x() = m_imuData[25];
    msg->linear_acceleration.y() = m_imuData[26];
    msg->linear_acceleration.z() = m_imuData[27];
    //ofs << msg->header.stamp.toSec() << " " << msg->angular_velocity.x() << " " << msg->angular_velocity.y() << " " << msg->angular_velocity.z() << " " << msg->linear_acceleration.x() << " " << msg->linear_acceleration.y() << " " << msg->linear_acceleration.z() << std::endl;
    notifyImu(msg);

    return datalen + 4;
}

int ParseBag::readImage(std::ifstream& ifs) {
    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4);
    int datalen = toInt(clen, 0, 4);

    sensor_msgs::ImageConstPtr msg = std::make_shared<sensor_msgs::Image::StImage>();

    // header
    ifs.read((char*)&buffer, sizeof(char) * 4); // seq
    //ifs.read((char*)&buffer, sizeof(char) * 8); // time
    //ifs.read((char*)&msg->header.seq, sizeof(char) * 4);
    ifs.read((char*)&msg->header.stamp.sec, sizeof(char) * 4);
    ifs.read((char*)&msg->header.stamp.nsec, sizeof(char) * 4);

    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4); // frame_id len
    int frameIdLen = toInt(clen, 0, 4);
    if (frameIdLen > 0) {
        memset(buffer, 0, MAX_LEN);
        ifs.read((char*)&buffer, sizeof(char) * frameIdLen); // frame_id
        //msg->header.frame_id = std::string((char*)buffer);
    }

    // format
    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4); // format len
    int formatLen = toInt(clen, 0, 4);
    memset(buffer, 0, MAX_LEN);
    ifs.read((char*)&buffer, sizeof(char) * formatLen); // format
    //std::string format = std::string((char*)buffer);

    // data
    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4); // format len
    int rawDataLen = toInt(clen, 0, 4);

    imgdata.resize(rawDataLen);
    //ifs.read((char*)&imgdata, sizeof(char) * rawDataLen);

    int curlen = 0;
    while (curlen <= rawDataLen - MAX_LEN) {
        ifs.read((char*)&buffer, sizeof(char) * MAX_LEN);
        for (int i = 0; i < MAX_LEN; ++i) {
            imgdata[curlen + i] = buffer[i];
        }
        curlen += MAX_LEN;
    }
    if (curlen < rawDataLen) {
        ifs.read((char*)&buffer, sizeof(char) * (rawDataLen - curlen));
        for (int i = 0; i < rawDataLen - curlen; ++i) {
            imgdata[curlen + i] = buffer[i];
        }
    }

    msg->imageView = cv::imdecode(imgdata, 1);
    //imageFilter(msg->imageView);
    notifyImage(msg);

    return datalen + 4;
}

int ParseBag::readPoint(std::ifstream& ifs) {
    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4);
    int datalen = toInt(clen, 0, 4);

    sensor_msgs::PointCloud2::Ptr msg = std::make_shared<sensor_msgs::PointCloud2::StPointCloud>();
    // header
    ifs.read((char*)&buffer, sizeof(char) * 4); // seq
    //ifs.read((char*)&buffer, sizeof(char) * 8); // time
    //ifs.read((char*)&msg->header.seq, sizeof(char) * 4);
    ifs.read((char*)&msg->header.stamp.sec, sizeof(char) * 4);
    ifs.read((char*)&msg->header.stamp.nsec, sizeof(char) * 4);

    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4); // frame_id len
    int frameIdLen = toInt(clen, 0, 4);
    memset(buffer, 0, MAX_LEN);
    ifs.read((char*)&buffer, sizeof(char) * frameIdLen); // frame_id

    // height and width 
    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4);
    int height = toInt(clen, 0, 4);
    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4);
    int width = toInt(clen, 0, 4);

    // point field
    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4); // point field num
    int fieldNum = toInt(clen, 0, 4);

    for (int i = 0; i < fieldNum; ++i) {
        memset(clen, 0, 4);
        ifs.read((char*)&clen, sizeof(char) * 4); // point field len
        int fieldlen = toInt(clen, 0, 4);

        memset(buffer, 0, MAX_LEN);
        ifs.read((char*)&buffer, sizeof(char) * fieldlen); // field name
        std::string name = std::string((char*)buffer);

        memset(clen, 0, 4);
        ifs.read((char*)&clen, sizeof(char) * 4); // offset
        int offset = toInt(clen, 0, 4);

        memset(clen, 0, 4);
        ifs.read((char*)&clen, sizeof(char) * 1); // datatype
        int datatype = toInt(clen, 0, 4);

        memset(clen, 0, 4);
        ifs.read((char*)&clen, sizeof(char) * 4); // count
        int count = toInt(clen, 0, 4);

        //sensor_msgs::PointCloud2::Field field;
        //field.name = name;
        //field.datatype = datatype;
        //field.count = count;
        //field.offset = offset;
        //msg->fields.push_back(field);
    }

    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 1); // is_bigendian

    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4); // point_step
    int pointStep = toInt(clen, 0, 4);

    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4); // row_step
    int rowStep = toInt(clen, 0, 4);

    // data
    {
        double timestemp = 0.0;
        short ring;

        memset(clen, 0, 4);
        ifs.read((char*)&clen, sizeof(char) * 4); // datalen = height * row_step

        double maximum_range = 10;
        for (int i = 0; i < height; ++i) {
            memset(buffer, 0, MAX_LEN);

            pcl::PointXYZINormal pt;

            ifs.read((char*)&pt.x, sizeof(float));
            ifs.read((char*)&pt.y, sizeof(float));
            ifs.read((char*)&pt.z, sizeof(float));
            ifs.read((char*)&cintensity, sizeof(char));
            //ifs.read((char*)&pt.intensity, sizeof(char));
            ifs.read((char*)&ring, sizeof(char) * 2);
            ifs.read((char*)&timestemp, sizeof(char) * 8);

            double frame_dis = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (frame_dis > maximum_range)
            {
                continue;
            }
            if ((int)(cintensity) < 0) {
                continue;
            }
            pt.intensity = ((int)(cintensity)+127) / 256.0;
            pt.curvature = 0;

            msg->pcl_pc.push_back(pt);
        }
    }

    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 1); // is_dense

    notifyPoints(msg);

    return datalen + 4;
}

int ParseBag::readHeader(std::ifstream& ifs, bool bBagHeader) {
    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4);
    int headerlen = toInt(clen, 0, 4);
    if (headerlen <= 0) return -1;
    memset(buffer, 0, MAX_LEN);
    ifs.read((char*)&buffer, sizeof(char) * (bBagHeader ? (4096 + 4) : headerlen));
    int curlen = 0;
    mapHeaderAttribute.clear();

    while (curlen < headerlen) {
        int fieldlen = toInt(buffer, curlen, 4);
        curlen += 4;
        int val = toString(buffer, curlen, fieldlen, sHeaderName, sHeaderValue);
        curlen += fieldlen;
        if ((strcmp(sHeaderName.c_str(), "op") != 0) && (strcmp(sHeaderName.c_str(), "conn") != 0)) continue;
        auto itr = mapHeaderAttribute.find(sHeaderName);
        if (itr != mapHeaderAttribute.end()) {
            itr->second = (val < 0 ? sHeaderValue : std::to_string(val));
        }
        else {
            mapHeaderAttribute[sHeaderName] = (val < 0 ? sHeaderValue : std::to_string(val));
        }
    }
    return headerlen + 4;
}

int ParseBag::readData(std::ifstream& ifs) {
    memset(clen, 0, 4);
    ifs.read((char*)&clen, sizeof(char) * 4);
    int datalen = toInt(clen, 0, 4);
    memset(buffer, 0, MAX_LEN);
    int curlen = 0;
    while (curlen <= datalen - MAX_LEN) {
        ifs.read((char*)&buffer, sizeof(char) * MAX_LEN);
        curlen += MAX_LEN;
    }
    if (curlen < datalen) {
        ifs.read((char*)&buffer, sizeof(char) * (datalen - curlen));
    }
    return datalen + 4;
}

int ParseBag::readHeader(std::FILE* file) {
    memset(clen, 0, 4);
    fread(clen, sizeof(char), 4, file);
    int headerlen = toInt(clen, 0, 4);
    memset(buffer, 0, MAX_LEN);
    fread(buffer, sizeof(char), headerlen, file);
    int curlen = 0;
    while (curlen < headerlen) {
        int fieldlen = toInt(buffer, curlen, 4);
        curlen += 4;
        std::string name, value;
        int val = toString(buffer, curlen, fieldlen, name, value);
        curlen += fieldlen;
        if (val < 0) {
            mapHeaderAttribute[name] = value;
        }
        else {
            mapHeaderAttribute[name] = std::to_string(val);
        }
    }
    return headerlen + 4;
}