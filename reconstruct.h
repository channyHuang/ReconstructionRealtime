#pragma once

#include "rgb_map/image_frame.hpp"
#include "rgb_map/pointcloud_rgbd.hpp"
#include "rgb_map/offline_map_recorder.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>

//#include <boost/program_options.hpp>
#include "meshing/MVS/Common.h"
#include "meshing/MVS/Image.h"
#include "meshing/MVS/PointCloud.h"
#include "meshing/MVS/Mesh.h"

#include "tools_mem_used.h"
#include "signalSlots.h"

class Reconstruction {
public:
    static Reconstruction* getInstance() {
        if (instance == nullptr) {
            instance = new Reconstruction();
        }
        return instance;
    }

    ~Reconstruction() {}

    int reconFromRecorder(Offline_map_recorder& r3live_map_recorder);
    int reconFromFile(const std::string& file);

    MVS::Mesh reconstruct_mesh(Offline_map_recorder& r3live_map_recorder, std::string output_dir);
    void r3live_map_to_mvs_scene(Offline_map_recorder& r3live_map_recorder, MVS::ImageArr& m_images, MVS::PointCloud& m_pointcloud);
    bool ReconstructMesh(float distInsert, bool bUseFreeSpaceSupport, unsigned nItersFixNonManifold, float kSigma, float kQual, MVS::Mesh& m_mesh, MVS::ImageArr& m_images, MVS::PointCloud& m_pointcloud);

public:
    SignalSlot::Signal<void(float x, float y, float z, float r, float g, float b)> notifyPoint;
    SignalSlot::Signal<void(std::vector<std::vector<double>> p)> notifyPoints;
    SignalSlot::Signal<void(float x, float y, float z, float w, float tx, float ty, float tz)> notifyPose;
    SignalSlot::Signal<void(int idx1, int idx2)> notifyEdge;
    SignalSlot::Signal<void(std::string str, bool b)> notifyFinish;

private:
    Reconstruction();

    static Reconstruction* instance;
};
