#include "reconstruct.h"

#include "parseBag.h"
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
//#include <unistd.h>
#include <so3_math.h>
//#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <common_lib.h>
#include <kd_tree/ikd_Tree.h>
//#include <nav_msgs/Odometry.h>
//#include <nav_msgs/Path.h>
#include <opencv2/core/eigen.hpp>
//#include <visualization_msgs/Marker.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <tf/transform_datatypes.h>
//#include <tf/transform_broadcaster.h>
//#include <geometry_msgs/Vector3.h>
#include <FOV_Checker/FOV_Checker.h>

#include "loam/IMU_Processing.hpp"
//#include "tools_logger.hpp"
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_data_io.hpp"
#include "tools_timer.hpp"
#include "tools_openCV_3_to_4.hpp"

#include "r3live.hpp"

#include "meshing/build_mesh.hpp"

Camera_Lidar_queue g_camera_lidar_queue;
MeasureGroup Measures;
StatesGroup g_lio_state;
std::string data_dump_dir = std::string("/mnt/0B3B134F0B3B134F/color_temp_r3live/");



Common_tools::Cost_time_logger              g_cost_time_logger;
//std::shared_ptr< Common_tools::ThreadPool > m_thread_pool_ptr;
std::string                                 data_path = std::string("/home/ziv/color_temp_r3live/");

std::string g_working_dir = "E:/projects/AllOsgProj/osgReconRealtime/build"; //"E:/projects/r3live-lab-res";
std::string g_offline_map_name;
double      g_insert_pt_dis = 1.0;
//bool        bUseConstantWeight;
bool        g_if_use_free_space_support = false;
double      g_thickness_factor = 1.0;
double      g_quality_factor = 0.0;
double      g_decimate_mesh = 1.0;
double      g_if_remove_spurious = 40.0;
bool        g_if_remove_spikes = true;
int         g_close_holes_dist = -1; // 20;
int         g_smooth_mesh_factor = 5;
double      g_add_keyframe_t = 0.15;
double      g_add_keyframe_R = 10;
int         g_texturing_smooth_factor = 10;
std::string g_str_export_type;
std::string strConfigFileName;

int         g_min_N_rgb = 5;

#define MeshPointType pcl::PointXYZRGBL
//using PointType = pcl::PointXYZRGBL; // pcl::PointXYZRGBA;
pcl::PointCloud<MeshPointType>::Ptr pcl_pc_rgb = nullptr;
pcl::KdTreeFLANN<MeshPointType> kdtree;

Reconstruction* Reconstruction::instance = nullptr;

void build_pcl_kdtree(Offline_map_recorder& r3live_map_recorder)
{
    if (pcl_pc_rgb == nullptr)
    {
        //pcl_pc_rgb = boost::make_shared< pcl::PointCloud< PointType > >();
        pcl_pc_rgb = std::make_shared< pcl::PointCloud< MeshPointType > >();
    }
    if (0) // if reload all pts.
    {
        pcl_pc_rgb->clear();
        pcl_pc_rgb->points.resize(r3live_map_recorder.m_global_map->m_rgb_pts_vec.size());
        for (int i = 0; i < r3live_map_recorder.m_global_map->m_rgb_pts_vec.size(); i++)
        {
            MeshPointType  pcl_pt;

            RGB_pt_ptr rgb_pt = r3live_map_recorder.m_global_map->m_rgb_pts_vec[i];
            if (rgb_pt->m_N_rgb < g_min_N_rgb /*5*/)
            {
                continue;
            }
            pcl_pt.x = rgb_pt->m_pos[0];
            pcl_pt.y = rgb_pt->m_pos[1];
            pcl_pt.z = rgb_pt->m_pos[2];
            pcl_pt.r = rgb_pt->m_rgb[2];
            pcl_pt.g = rgb_pt->m_rgb[1];
            pcl_pt.b = rgb_pt->m_rgb[0];
            pcl_pt.label = rgb_pt->m_pt_index;
            pcl_pc_rgb->points[i] = pcl_pt;
        }
    }
    kdtree.setInputCloud(pcl_pc_rgb);
}

void texture_mesh(Offline_map_recorder& r3live_map_recorder, std::string input_mesh_name, std::string output_mesh_name, int smooth_factor)
{
    cout << "Performaning the mesh texturing..." << endl;
    cout << "Build Kd tree, please wait..." << endl;
    build_pcl_kdtree(r3live_map_recorder);
    cout << "Build Kd tree finish !" << endl;
    pcl::PolygonMesh        mesh_obj, mesh_textured;
    std::vector< int >      pointIdxNKNSearch(smooth_factor);
    std::vector< float >    pointNKNSquaredDistance(smooth_factor);
    pcl::PointCloud< MeshPointType > rgb_pointcloud;

    cout << "Loading mesh to PCL polygon, please wait...." << endl;
    cout << "Load mesh from file: " << input_mesh_name << endl;
    pcl::io::loadOBJFile(input_mesh_name, mesh_obj);
    cout << "Loading mesh finish" << endl;
    pcl::fromPCLPointCloud2(mesh_obj.cloud, rgb_pointcloud);
    for (int i = 0; i < rgb_pointcloud.points.size(); ++i)
    {
        uint8_t  r = 0;
        uint8_t  g = 0;
        uint8_t  b = 0;
        float    dist = 0.0;
        int      red = 0;
        int      green = 0;
        int      blue = 0;
        uint32_t rgb;
        if (kdtree.nearestKSearch(rgb_pointcloud.points[i], smooth_factor, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            for (int j = 0; j < pointIdxNKNSearch.size(); ++j)
            {
                r = pcl_pc_rgb->points[pointIdxNKNSearch[j]].r;
                g = pcl_pc_rgb->points[pointIdxNKNSearch[j]].g;
                b = pcl_pc_rgb->points[pointIdxNKNSearch[j]].b;
                red += int(r);
                green += int(g);
                blue += int(b);
                dist += 1.0 / pointNKNSquaredDistance[j];
            }
        }
        if (pointNKNSquaredDistance.size() < smooth_factor)
        {
            cout << "\r\n" << "Knn search fail!" << endl << endl;
        }
        rgb_pointcloud.points[i].r = int(red / pointIdxNKNSearch.size());
        rgb_pointcloud.points[i].g = int(green / pointIdxNKNSearch.size());
        rgb_pointcloud.points[i].b = int(blue / pointIdxNKNSearch.size());
        rgb_pointcloud.points[i].a = 255;
        if (i % 10000 == 0)
        {
            printf("\33[2K\rTexturing mesh [%u%%] ...", i * 100 / (rgb_pointcloud.points.size() - 1));
            ANSI_SCREEN_FLUSH;
        }
    }
    printf("\33[2K\rTexturing mesh [100%%] \r\n");
    pcl::toPCLPointCloud2(rgb_pointcloud, mesh_obj.cloud);
    cout << "Saved textured mesh to: " << output_mesh_name << endl;
    pcl::io::savePLYFileBinary(output_mesh_name, mesh_obj);
    cout << "Finish!!!" << endl;
    cout << "=== Mesh texturing finish ! ===" << endl;
}

void load_parameter()
{
    std::string _offline_map_name = "test.r3live"; 
    g_offline_map_name = std::string(g_working_dir).append("/").append(_offline_map_name);
}

int Reconstruction::reconFromFile(const std::string& file) {
    load_parameter();
    Global_map       global_map(0);
    Offline_map_recorder r3live_map_recorder;
    cout << "Open file from: " << g_offline_map_name << endl;
    global_map.m_if_reload_init_voxel_and_hashed_pts = 0;
    r3live_map_recorder.m_global_map = &global_map;
    Common_tools::load_obj_from_file(&r3live_map_recorder, g_offline_map_name);
    cout << "Number of rgb points: " << global_map.m_rgb_pts_vec.size() << endl;

    return reconFromRecorder(r3live_map_recorder);
}

int Reconstruction::reconFromRecorder(Offline_map_recorder& r3live_map_recorder)
{
    cout << "Size of frames: " << r3live_map_recorder.m_image_pose_vec.size() << endl;
    MVS::Mesh meshWithoutTex = reconstruct_mesh(r3live_map_recorder, g_working_dir);
    cout << "=== Reconstruct mesh finish ! ===" << endl;

    std::string input_mesh_name = std::string(g_working_dir).append("/reconstructed_mesh.obj");
    std::string output_mesh_name = std::string(g_working_dir).append("/textured_mesh.ply");
    texture_mesh(r3live_map_recorder, input_mesh_name, output_mesh_name, g_texturing_smooth_factor);

    notifyFinish((g_working_dir)+("/reconstructed_mesh") + ".obj", true);
    return 0;
}

MVS::Mesh Reconstruction::reconstruct_mesh(Offline_map_recorder& r3live_map_recorder, std::string output_dir)
{
    cout << "==== Work directory: " << output_dir << endl;

    MVS::ImageArr   m_images;
    MVS::PointCloud m_pointcloud;
    MVS::Mesh       reconstructed_mesh;

    r3live_map_to_mvs_scene(r3live_map_recorder, m_images, m_pointcloud);

    ReconstructMesh(g_insert_pt_dis, g_if_use_free_space_support, 4, g_thickness_factor, g_quality_factor, reconstructed_mesh, m_images, m_pointcloud);
    printf("Mesh reconstruction completed: %u vertices, %u faces\n", reconstructed_mesh.vertices.GetSize(), reconstructed_mesh.faces.GetSize());
    cout << "Clean mesh [1/3]: ";
    reconstructed_mesh.Clean(g_decimate_mesh, g_if_remove_spurious, g_if_remove_spikes, g_close_holes_dist, g_smooth_mesh_factor, false);

    cout << "Clean mesh [2/3]: ";
    reconstructed_mesh.Clean(1.f, 0.f, g_if_remove_spikes, g_close_holes_dist, 0, false); // extra cleaning trying to close more holes

    cout << "Clean mesh [3/3]: ";
    reconstructed_mesh.Clean(1.f, 0.f, false, 0, 0, true); // extra cleaning to
                                                             // remove non-manifold
                                                             // problems created by
                                                             // closing holes
    //reconstructed_mesh.Save(MAKE_PATH_SAFE(Util::getFileFullName(output_dir)) + _T("/reconstructed_mesh") + ".ply");
    //reconstructed_mesh.Save(MAKE_PATH_SAFE(Util::getFileFullName(output_dir)) + _T("/reconstructed_mesh") + ".obj");
    reconstructed_mesh.Save((output_dir)+("/reconstructed_mesh") + ".obj");
    return reconstructed_mesh;
}

void outputCamera(Offline_map_recorder& r3live_map_recorder) {
    std::cout << "start output camera " << std::endl;
    std::ofstream ofs("./sparse/cameras.txt");
    ofs << "# Camera list with one line of data per camera:\n#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n# Number of cameras : 1" << std::endl;
    ofs << "1 PINHOLE " << r3live_map_recorder.m_image_pose_vec[0]->m_img_cols << " " << r3live_map_recorder.m_image_pose_vec[0]->m_img_rows << " ";
    ofs << r3live_map_recorder.m_image_pose_vec[0]->fx << " " << r3live_map_recorder.m_image_pose_vec[0]->fy << " " << r3live_map_recorder.m_image_pose_vec[0]->cx << " " << r3live_map_recorder.m_image_pose_vec[0]->cy << std::endl;
    ofs.close();

    std::unordered_map<int, std::unordered_map<int, int>> point2dIndex;
    ofs.open("./sparse/images.txt");
    char name[10] = { 0 };
    ofs << "# Image list with two lines of data per image:\n#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n#   POINTS2D[] as(X, Y, POINT3D_ID)" << std::endl;
    ofs << "# Number of images: " << r3live_map_recorder.m_image_pose_vec.size() << std::endl;
    for (int frame_idx = 0; frame_idx < r3live_map_recorder.m_image_pose_vec.size(); frame_idx++) {
        memset(name, 0, 10);
        sprintf(name, "%04d.jpg", frame_idx);
        r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_w2c_q = r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_c2w_q.inverse();
        r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_w2c_t = -(r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_c2w_q.inverse() * r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_c2w_t);
        ofs << frame_idx + 1 << " "
            << r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_c2w_q.w() << " "
            << r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_c2w_q.x() << " "
            << r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_c2w_q.y() << " "
            << r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_c2w_q.z() << " "
            << r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_c2w_t.x() << " "
            << r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_c2w_t.y() << " "
            << r3live_map_recorder.m_image_pose_vec[frame_idx]->m_pose_c2w_t.z() << " "
            << "1 " << name << std::endl;

        std::shared_ptr< Image_frame > img_ptr = r3live_map_recorder.m_image_pose_vec[frame_idx];
        double u_f, v_f, angle;
        int real_point_idx = 0;
        for (int point_idx = 0; point_idx < r3live_map_recorder.m_pts_in_views_vec[frame_idx].size(); ++point_idx) {
            std::shared_ptr< RGB_pts > pt = r3live_map_recorder.m_pts_in_views_vec[frame_idx][point_idx];
            pcl::PointXYZI temp_pt;
            temp_pt.x = pt->get_pos().x();
            temp_pt.y = pt->get_pos().y();
            temp_pt.z = pt->get_pos().z();
            bool res = img_ptr->project_3d_point_in_this_img(temp_pt, u_f, v_f, angle, nullptr, 1.0);
            if (res) {
                ofs << u_f << " " << v_f << " " << pt->m_pt_index << " ";

                point2dIndex[pt->m_pt_index].insert(std::make_pair(frame_idx + 1, real_point_idx + 1));
                real_point_idx++;
            }
        }
        ofs << std::endl;
    }
    ofs.close();

    ofs.open("./sparse/points3D.txt");
    ofs << "# 3D point list with one line of data per point:\n#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as(IMAGE_ID, POINT2D_IDX)" << std::endl;
    for (auto p_pt = r3live_map_recorder.m_global_map->m_rgb_pts_vec.begin(); p_pt != r3live_map_recorder.m_global_map->m_rgb_pts_vec.end(); p_pt++) {
        std::shared_ptr< RGB_pts > pt = *p_pt;
        if (pt->m_N_rgb < g_min_N_rgb) continue;
        ofs << pt->m_pt_index << " " << pt->get_pos().x() << " " << pt->get_pos().y() << " " << pt->get_pos().z() << " "
            << pt->get_rgb().x() << " " << pt->get_rgb().y() << " " << pt->get_rgb().z() << " 1 ";

        auto pit = point2dIndex.find(pt->m_pt_index);
        if (pit != point2dIndex.end()) {
            for (int frame_idx = pt->m_frame_idx_st; frame_idx <= pt->m_frame_idx_end; ++frame_idx) {
                auto fit = pit->second.find(frame_idx + 1);
                if (fit != pit->second.end()) {
                    ofs << frame_idx + 1 << " " << fit->second << " ";
                }
            }
        }
        ofs << std::endl;
    }
    ofs.close();
    std::cout << "end output camera " << std::endl;
}

// TODO
void Reconstruction::r3live_map_to_mvs_scene(Offline_map_recorder& r3live_map_recorder, MVS::ImageArr& m_images, MVS::PointCloud& m_pointcloud)
{
    vec_3   last_pose_t = vec_3(-100, 0, 0);
    eigen_q last_pose_q = eigen_q(1, 0, 0, 1);
    int     m_image_id = 0;
    int     number_of_image_frame = r3live_map_recorder.m_pts_in_views_vec.size();
    cout << "Number of image frames: " << number_of_image_frame << endl;

    Eigen::Matrix3d camera_intrinsic;
    int             image_width = r3live_map_recorder.m_image_pose_vec[0]->m_img_cols;
    int             image_heigh = r3live_map_recorder.m_image_pose_vec[0]->m_img_rows;
    double          fx = r3live_map_recorder.m_image_pose_vec[0]->fx;
    double          fy = r3live_map_recorder.m_image_pose_vec[0]->fy;
    double          cx = r3live_map_recorder.m_image_pose_vec[0]->cx;
    double          cy = r3live_map_recorder.m_image_pose_vec[0]->cy;
    camera_intrinsic << fx / image_width, 0, cx / image_width, 0, fy / image_width, cy / image_width, 0, 0, 1;
    // camera_intrinsic << fx , 0, cx , 0, fy , cy , 0, 0, 1;
    cout << "Iamge resolution  = " << r3live_map_recorder.m_image_pose_vec[0]->m_img_cols << " X " << r3live_map_recorder.m_image_pose_vec[0]->m_img_rows << endl;
    // cout << "Camera intrinsic: \r\n" << camera_intrinsic << endl;

    MVS::Platform m_platforms = MVS::Platform();
    m_platforms.name = std::string("platfrom");
    m_platforms.cameras.push_back(MVS::Platform::Camera());
    m_platforms.cameras[0].K = camera_intrinsic;
    m_platforms.cameras[0].R = Eigen::Matrix3d::Identity();
    m_platforms.cameras[0].C = Eigen::Vector3d::Zero();

    std::unordered_map< std::shared_ptr< RGB_pts >, std::vector< int > >& m_pts_with_view = r3live_map_recorder.m_pts_with_view;
    for (int frame_idx = 0; frame_idx < number_of_image_frame; frame_idx++)
    {
        std::shared_ptr< Image_frame > img_ptr = r3live_map_recorder.m_image_pose_vec[frame_idx];
        vec_3                              pose_t = -img_ptr->m_pose_c2w_q.toRotationMatrix().transpose() * img_ptr->m_pose_c2w_t;
        if ((pose_t - last_pose_t).norm() < g_add_keyframe_t && (img_ptr->m_pose_c2w_q.angularDistance(last_pose_q) * 57.3 < g_add_keyframe_R))
        {
            //continue;
        }
        img_ptr->m_valid = true;

        MVS::Platform::Pose pose;
        MVS::Image          image;

        pose.R = img_ptr->m_pose_c2w_q.toRotationMatrix();
        pose.C = pose_t;
        last_pose_t = pose_t;
        last_pose_q = img_ptr->m_pose_c2w_q;
        m_platforms.poses.push_back(pose);
        // cout << "[ " << frame_idx << " ]: q = " << img_ptr->m_pose_c2w_q.coeffs().transpose() << " | "<< pose_t.transpose() << endl;

        image.ID = m_image_id;
        m_image_id++;
        image.poseID = image.ID;
        image.platformID = 0;
        image.cameraID = 0;
        image.width = img_ptr->m_img_cols;
        image.height = img_ptr->m_img_rows;
        image.camera = MVS::Camera(m_platforms.GetCamera(image.cameraID, image.poseID));

        // compute the unnormalized camera
        image.camera.K = image.camera.GetK< REAL >(image_width, image_heigh);
        image.camera.ComposeP();
        m_images.push_back(image);

        for (int pt_idx = 0; pt_idx < r3live_map_recorder.m_pts_in_views_vec[frame_idx].size(); pt_idx++)
        {
            m_pts_with_view[r3live_map_recorder.m_pts_in_views_vec[frame_idx][pt_idx]].push_back(image.ID);
        }
        cout << ANSI_DELETE_CURRENT_LINE;
        printf("\33[2K\rAdd frames: %u%%, total_points = %u ...", frame_idx * 100 / (number_of_image_frame - 1), m_pts_with_view.size());
        ANSI_SCREEN_FLUSH;
    }
    cout << endl;
    cout << "Number of image frames: " << m_image_id << endl;
    cout << "Number of points " << m_pts_with_view.size() << endl;

    int acc_count = 0;
    m_pointcloud.points.resize(m_pts_with_view.size());
    m_pointcloud.pointViews.resize(m_pts_with_view.size());
    m_pointcloud.colors.resize(m_pts_with_view.size());
    long point_index = 0;
    long temp_int = 0;
    if (pcl_pc_rgb == nullptr)
    {
        //pcl_pc_rgb = boost::make_shared<pcl::PointCloud<PointType>>();
        pcl_pc_rgb = std::make_shared<pcl::PointCloud<MeshPointType>>();
    }
    pcl_pc_rgb->clear();
    pcl_pc_rgb->reserve(1e8);
    for (std::unordered_map< std::shared_ptr< RGB_pts >, std::vector< int > >::iterator it = m_pts_with_view.begin(); it != m_pts_with_view.end(); it++)
    {
        if ((it->second.size() >= 0) && ((it->first)->m_N_rgb >= g_min_N_rgb/*5*/))
        {
            acc_count++;
            MVS::PointCloud::Point    pt3d;
            MVS::PointCloud::ViewArr  pt_view_arr;
            MVS::PointCloud::ColorArr color_arr;
            MVS::PointCloud::Color    color;
            vec_3                     pt_pos = ((it->first))->get_pos();
            pt3d.x = pt_pos(0);
            pt3d.y = pt_pos(1);
            pt3d.z = pt_pos(2);
            color.r = ((it->first))->m_rgb[2];
            color.g = ((it->first))->m_rgb[1];
            color.b = ((it->first))->m_rgb[0];
            MeshPointType pcl_pt;
            pcl_pt.x = pt3d.x;
            pcl_pt.y = pt3d.y;
            pcl_pt.z = pt3d.z;
            pcl_pt.r = color.r;
            pcl_pt.g = color.g;
            pcl_pt.b = color.b;
            pcl_pt.label = (it->first)->m_pt_index;
            pcl_pc_rgb->points.push_back(pcl_pt);
            for (auto _idx : it->second)
            {
                pt_view_arr.push_back(_idx);
            }
            m_pointcloud.points[point_index] = pt3d;
            m_pointcloud.pointViews[point_index] = pt_view_arr;
            m_pointcloud.colors[point_index] = color;
            point_index++;
            if ((temp_int + 1) % (m_pts_with_view.size() / 10) == 0)
            {
                printf("\33[2K\rRetring points: %u%% ...", temp_int * 10 / (m_pts_with_view.size() / 10));
                ANSI_SCREEN_FLUSH;
            }
        }
        temp_int++;
    }
    printf("\33[2K\rRetriving points: %u%% ...", 100);
    m_pointcloud.points.resize(point_index);
    m_pointcloud.pointViews.resize(point_index);
    m_pointcloud.colors.resize(point_index);
    cout << endl;
    cout << "Total available points number " << point_index << endl;

    outputCamera(r3live_map_recorder);
}


// First, iteratively create a Delaunay triangulation of the existing point-cloud by inserting point by point,
// iif the point to be inserted is not closer than distInsert pixels in at least one of its views to
// the projection of any of already inserted points.
// Next, the score is computed for all the edges of the directed graph composed of points as vertices.
// Finally, graph-cut algorithm is used to split the tetrahedrons in inside and outside,
// and the surface is such extracted.

// Input data: mesh, pointcloud, camera positions, pts_in camera_ray
bool Reconstruction::ReconstructMesh(float distInsert, bool bUseFreeSpaceSupport, unsigned nItersFixNonManifold, float kSigma, float kQual, Mesh& m_mesh,
    ImageArr& m_images, PointCloud& m_pointcloud)
{
    float kb = 4.f;
    float kf = 3.f;
    float kRel = 0.1f /*max 0.3*/;
    float kAbs = 1000.f /*min 500*/;
    float kOutl = 400.f /*max 700.f*/;
    float kInf = (float)(INT_MAX / 8);
    // cout << distInsert << " | " << bUseFreeSpaceSupport << " | " << nItersFixNonManifold << " | " << kSigma << " | " << kQual << " | "  << endl;
    using namespace DELAUNAY;

    ASSERT(!m_pointcloud.IsEmpty());
    m_mesh.Release();

    // create the Delaunay triangulation
    delaunay_t                   delaunay;
    std::vector< cell_info_t >   infoCells;
    std::vector< camera_cell_t > camCells;
    std::vector< facet_t >       hullFacets;

    {
        TD_TIMER_STARTD();

        std::vector< point_t >        vertices(m_pointcloud.points.GetSize());
        std::vector< std::ptrdiff_t > indices(m_pointcloud.points.GetSize());
        //std::ofstream ofs("delaunay.obj");
        // ===============================================
        // fetch points
        FOREACH(i, m_pointcloud.points)
        {
            const PointCloud::Point& X(m_pointcloud.points[i]);
            vertices[i] = point_t(X.x, X.y, X.z);
            indices[i] = i;

            //ofs << "v " << X.x << " " << X.y << " " << X.z << std::endl;
        }
        //ofs.close();
        // ===============================================
        // sort vertices
        typedef CGAL::Spatial_sort_traits_adapter_3< delaunay_t::Geom_traits, point_t* > Search_traits;
        CGAL::spatial_sort(indices.begin(), indices.end(), Search_traits(&vertices[0], delaunay.geom_traits()));

        // ===============================================
        // insert vertices
        Util::Progress          progress(("Points inserted"), indices.size());
        const float             distInsertSq(SQUARE(distInsert));
        vertex_handle_t         hint;
        delaunay_t::Locate_type lt;
        int                     li, lj;

        std::for_each(indices.cbegin(), indices.cend(), [&](size_t idx) {
            const point_t& p = vertices[idx];
            const PointCloud::Point& point = m_pointcloud.points[idx];
            const PointCloud::ViewArr& views = m_pointcloud.pointViews[idx];
            ASSERT(!views.IsEmpty());
            if (hint == vertex_handle_t())
            {
                // this is the first point,
                // insert it
                hint = delaunay.insert(p);
                ASSERT(hint != vertex_handle_t());
            }
            else if (distInsert <= 0)
            {
                // insert all points
                hint = delaunay.insert(p, hint);
                ASSERT(hint != vertex_handle_t());
            }
            else
            {
                // locate cell containing this point
                const cell_handle_t c(delaunay.locate(p, lt, li, lj, hint->cell()));
                if (lt == delaunay_t::VERTEX)
                {
                    // duplicate point, nothing to insert,
                    // just update its visibility info
                    hint = c->vertex(li);
                    ASSERT(hint != delaunay.infinite_vertex());
                }
                else
                {
                    // locate the nearest vertex
                    vertex_handle_t nearest;
                    if (delaunay.dimension() < 3)
                    {
                        // use a brute-force algorithm if dimension < 3
                        delaunay_t::Finite_vertices_iterator vit = delaunay.finite_vertices_begin();
                        nearest = vit;
                        ++vit;
                        adjacent_vertex_back_inserter_t inserter(delaunay, p, nearest);
                        for (delaunay_t::Finite_vertices_iterator end = delaunay.finite_vertices_end(); vit != end; ++vit)
                            inserter = vit;
                    }
                    else
                    {
                        // - start with the closest vertex from the located cell
                        // - repeatedly take the nearest of its incident vertices if any
                        // - if not, we're done
                        ASSERT(c != cell_handle_t());
                        nearest = delaunay.nearest_vertex_in_cell(p, c);
                        while (true)
                        {
                            const vertex_handle_t v(nearest);
                            delaunay.adjacent_vertices(nearest, adjacent_vertex_back_inserter_t(delaunay, p, nearest));
                            if (v == nearest)
                                break;
                        }
                    }
                    ASSERT(nearest == delaunay.nearest_vertex(p, hint->cell()));
                    hint = nearest;
                    // check if point is far enough to all existing points
                    FOREACHPTR(pViewID, views)
                    {
                        const Image& imageData = m_images[*pViewID];
                        const Point3f pn(imageData.camera.ProjectPointP3(point));
                        const Point3f pe(imageData.camera.ProjectPointP3(CGAL2MVS< float >(nearest->point())));
                        if (!IsDepthSimilar(pn.z, pe.z) || normSq(Point2f(pn) - Point2f(pe)) > distInsertSq)
                        {
                            // point far enough to an existing point,
                            // insert as a new point
                            static int print_idx = 0;
                            hint = delaunay.insert(p, lt, c, li, lj);
                            ASSERT(hint != vertex_handle_t());
                            break;
                        }
                    }
                }
            }
            // update point visibility info
            hint->info().InsertViews(m_pointcloud, idx);
            ++progress;
            });

        progress.close();
        m_pointcloud.Release();

        // ===============================================
        // init cells weights and
        // loop over all cells and store the finite facet of the infinite cells
        const size_t numNodes(delaunay.number_of_cells());
        infoCells.resize(numNodes);
        memset(&infoCells[0], 0, sizeof(cell_info_t) * numNodes);
        cell_size_t ciID(0);

        for (delaunay_t::All_cells_iterator ci = delaunay.all_cells_begin(), eci = delaunay.all_cells_end(); ci != eci; ++ci, ++ciID)
        {
            ci->info() = ciID;
            // skip the finite cells
            if (!delaunay.is_infinite(ci)) {
                continue;
            }
            // find the finite face
            for (int f = 0; f < 4; ++f)
            {
                const facet_t facet(ci, f);
                if (!delaunay.is_infinite(facet))
                {
                    // store face
                    hullFacets.push_back(facet);
                    break;
                }
            }
        }

        // ===============================================
        // find all cells containing a camera
        camCells.resize(m_images.GetSize());
        FOREACH(i, m_images)
        {
            const Image& imageData = m_images[i];
            if (!imageData.IsValid())
                continue;
            const Camera& camera = imageData.camera;
            camera_cell_t& camCell = camCells[i];
            camCell.cell = delaunay.locate(MVS2CGAL(camera.C)); // Locate camera in cells.
            ASSERT(camCell.cell != cell_handle_t());
            fetchCellFacets< CGAL::POSITIVE >(delaunay, hullFacets, camCell.cell, imageData, camCell.facets);
            // link all cells contained by the camera to the source
            for (const facet_t& f : camCell.facets)
                infoCells[f.first->info()].weight_s = kInf;
        }

        // DEBUG_EXTRA( "Delaunay tetrahedralization completed: %u points -> %u vertices, %u (+%u) cells, %u (+%u) faces (%s)", indices.size(),
        // delaunay.number_of_vertices(),
        //              delaunay.number_of_finite_cells(), delaunay.number_of_cells() - delaunay.number_of_finite_cells(),
        //              delaunay.number_of_finite_facets(), delaunay.number_of_facets() - delaunay.number_of_finite_facets(),
        //              TD_TIMER_GET_FMT().c_str() );
        printf("Delaunay tetrahedralization completed: %u points -> %u vertices, %u (+%u) cells, %u (+%u) faces (%s)\r\n", indices.size(),
            delaunay.number_of_vertices(), delaunay.number_of_finite_cells(), delaunay.number_of_cells() - delaunay.number_of_finite_cells(),
            delaunay.number_of_finite_facets(), delaunay.number_of_facets() - delaunay.number_of_finite_facets(), TD_TIMER_GET_FMT().c_str());
    }
    //output(delaunay, "mesh0.obj");
    // ===============================================
    // for every camera-point ray intersect it with the tetrahedrons and
    // add alpha_vis(point) to cell's directed edge in the graph
    {
        TD_TIMER_STARTD();

        // estimate the size of the smallest reconstructible object
        FloatArr distsSq(0, delaunay.number_of_edges());
        for (delaunay_t::Finite_edges_iterator ei = delaunay.finite_edges_begin(), eei = delaunay.finite_edges_end(); ei != eei; ++ei)
        {
            const cell_handle_t& c(ei->first);
            distsSq.Insert(normSq(CGAL2MVS< float >(c->vertex(ei->second)->point()) - CGAL2MVS< float >(c->vertex(ei->third)->point())));
        }
        const float sigma(SQRT(distsSq.GetMedian())* kSigma);
        const float inv2SigmaSq(0.5f / (sigma * sigma));
        distsSq.Release();

        std::vector< facet_t > facets;

        // compute the weights for each edge
        {
            TD_TIMER_STARTD();
            Util::Progress progress(("Points weighted"), delaunay.number_of_vertices());
#ifdef DELAUNAY_USE_OPENMP
            delaunay_t::Vertex_iterator vertexIter(delaunay.vertices_begin());
            const int64_t               nVerts(delaunay.number_of_vertices() + 1);
#pragma omp parallel for private( facets )
            for (int64_t i = 0; i < nVerts; ++i)
            {
                delaunay_t::Vertex_iterator vi;
#pragma omp critical
                vi = vertexIter++;
#else
            for (delaunay_t::Vertex_iterator vi = delaunay.vertices_begin(), vie = delaunay.vertices_end(); vi != vie; ++vi)
            {
#endif
                vert_info_t& vert(vi->info());
                if (vert.views.IsEmpty())
                    continue;
#ifdef DELAUNAY_WEAKSURF
                vert.AllocateInfo();
#endif
                const point_t& p(vi->point());
                const Point3   pt(CGAL2MVS< REAL >(p));
                FOREACH(v, vert.views)
                {
                    const typename vert_info_t::view_t view(vert.views[v]);
                    const uint32_t                     imageID(view.idxView);
                    const edge_cap_t                   alpha_vis(view.weight);
                    const Image& imageData = m_images[imageID];
                    ASSERT(imageData.IsValid());
                    const Camera& camera = imageData.camera;
                    const camera_cell_t& camCell = camCells[imageID];
                    // compute the ray used to find point intersection
                    const Point3   vecCamPoint(pt - camera.C);
                    const REAL     invLenCamPoint(REAL(1) / norm(vecCamPoint));
                    intersection_t inter(pt, Point3(vecCamPoint * invLenCamPoint));
                    // find faces intersected by the camera-point segment
                    const segment_t segCamPoint(MVS2CGAL(camera.C), p);
                    if (!intersect(delaunay, segCamPoint, camCell.facets, facets, inter))
                        continue;
                    do
                    {
                        // assign score, weighted by the distance from the point to the intersection
                        const edge_cap_t w(alpha_vis * (1.f - EXP(-SQUARE((float)inter.dist) * inv2SigmaSq)));
                        edge_cap_t& f(infoCells[inter.facet.first->info()].wright_face[inter.facet.second]);
#ifdef DELAUNAY_USE_OPENMP
#pragma omp atomic
#endif
                        f += w;
                    } while (intersect(delaunay, segCamPoint, facets, facets, inter));
                    ASSERT(facets.empty() && inter.type == intersection_t::VERTEX && inter.v1 == vi);
#ifdef DELAUNAY_WEAKSURF
                    ASSERT(vert.viewsInfo[v].cell2Cam == NULL);
                    vert.viewsInfo[v].cell2Cam = inter.facet.first;
#endif
                    // find faces intersected by the endpoint-point segment
                    inter.dist = FLT_MAX;
                    inter.bigger = false;
                    const Point3        endPoint(pt + vecCamPoint * (invLenCamPoint * sigma));
                    const segment_t     segEndPoint(MVS2CGAL(endPoint), p);
                    const cell_handle_t endCell(delaunay.locate(segEndPoint.source(), vi->cell()));
                    ASSERT(endCell != cell_handle_t());
                    fetchCellFacets< CGAL::NEGATIVE >(delaunay, hullFacets, endCell, imageData, facets);
                    edge_cap_t& t(infoCells[endCell->info()].weight_t);
#ifdef DELAUNAY_USE_OPENMP
#pragma omp atomic
#endif
                    t += alpha_vis;
                    while (intersect(delaunay, segEndPoint, facets, facets, inter))
                    {
                        // assign score, weighted by the distance from the point to the intersection
                        const facet_t& mf(delaunay.mirror_facet(inter.facet));
                        const edge_cap_t w(alpha_vis * (1.f - EXP(-SQUARE((float)inter.dist) * inv2SigmaSq)));
                        edge_cap_t& f(infoCells[mf.first->info()].wright_face[mf.second]);
#ifdef DELAUNAY_USE_OPENMP
#pragma omp atomic
#endif
                        f += w;
                    }
                    ASSERT(facets.empty() && inter.type == intersection_t::VERTEX && inter.v1 == vi);
#ifdef DELAUNAY_WEAKSURF
                    ASSERT(vert.viewsInfo[v].cell2End == NULL);
                    vert.viewsInfo[v].cell2End = inter.facet.first;
#endif
                }
                ++progress;
            }
            progress.close();
            // DEBUG_ULTIMATE( "\tweighting completed in %s", TD_TIMER_GET_FMT().c_str() );
            printf("Weighting completed in %s\r\n", TD_TIMER_GET_FMT().c_str());
            }
        camCells.clear();

#ifdef DELAUNAY_WEAKSURF
        // enforce t-edges for each point-camera pair with free-space support weights
        if (bUseFreeSpaceSupport)
        {
            TD_TIMER_STARTD();
#ifdef DELAUNAY_USE_OPENMP
            delaunay_t::Vertex_iterator vertexIter(delaunay.vertices_begin());
            const int64_t               nVerts(delaunay.number_of_vertices() + 1);
#pragma omp parallel for private( facets )
            for (int64_t i = 0; i < nVerts; ++i)
            {
                delaunay_t::Vertex_iterator vi;
#pragma omp critical
                vi = vertexIter++;
#else
            for (delaunay_t::Vertex_iterator vi = delaunay.vertices_begin(), vie = delaunay.vertices_end(); vi != vie; ++vi)
            {
#endif
                const vert_info_t& vert(vi->info());
                if (vert.views.IsEmpty())
                    continue;
                const point_t& p(vi->point());
                const Point3f  pt(CGAL2MVS< float >(p));
                FOREACH(v, vert.views)
                {
                    const uint32_t imageID(vert.views[(vert_info_t::view_vec_t::IDX)v]);
                    const Image& imageData = m_images[imageID];
                    ASSERT(imageData.IsValid());
                    const Camera& camera = imageData.camera;
                    // compute the ray used to find point intersection
                    const Point3f vecCamPoint(pt - Cast< float >(camera.C));
                    const float   invLenCamPoint(1.f / norm(vecCamPoint));
                    // find faces intersected by the point-camera segment and keep the max free-space support score
                    const Point3f   bgnPoint(pt - vecCamPoint * (invLenCamPoint * sigma * kf));
                    const segment_t segPointBgn(p, MVS2CGAL(bgnPoint));
                    intersection_t  inter;
                    if (!intersectFace(delaunay, segPointBgn, vi, vert.viewsInfo[v].cell2Cam, facets, inter))
                        continue;
                    edge_cap_t beta(0);
                    do
                    {
                        const edge_cap_t fs(freeSpaceSupport(delaunay, infoCells, inter.facet.first));
                        if (beta < fs)
                            beta = fs;
                    } while (intersectFace(delaunay, segPointBgn, facets, facets, inter));
                    // find faces intersected by the point-endpoint segment
                    const Point3f   endPoint(pt + vecCamPoint * (invLenCamPoint * sigma * kb));
                    const segment_t segPointEnd(p, MVS2CGAL(endPoint));
                    if (!intersectFace(delaunay, segPointEnd, vi, vert.viewsInfo[v].cell2End, facets, inter))
                        continue;
                    edge_cap_t gammaMin(FLT_MAX), gammaMax(0);
                    do
                    {
                        const edge_cap_t fs(freeSpaceSupport(delaunay, infoCells, inter.facet.first));
                        if (gammaMin > fs)
                            gammaMin = fs;
                        if (gammaMax < fs)
                            gammaMax = fs;
                    } while (intersectFace(delaunay, segPointEnd, facets, facets, inter));
                    const edge_cap_t gamma((gammaMin + gammaMax) * 0.5f);
                    // if the point can be considered an interface point,
                    // enforce the t-edge weight of the end cell
                    const edge_cap_t epsAbs(beta - gamma);
                    const edge_cap_t epsRel(gamma / beta);
                    if (epsRel < kRel && epsAbs > kAbs && gamma < kOutl)
                    {
                        edge_cap_t& t(infoCells[inter.ncell->info()].weight_t);
#ifdef DELAUNAY_USE_OPENMP
#pragma omp atomic
#endif
                        t *= epsAbs;
                    }
                }
            }
            // DEBUG_ULTIMATE( "\tt-edge reinforcement completed in %s", TD_TIMER_GET_FMT().c_str() );
            printf("\tt-edge reinforcement completed in %s\r\n", TD_TIMER_GET_FMT().c_str());
            }
#endif

        // DEBUG_EXTRA( "Delaunay tetrahedras weighting completed: %u cells, %u faces (%s)", delaunay.number_of_cells(), delaunay.number_of_facets(),
        // TD_TIMER_GET_FMT().c_str() );
        printf("Delaunay tetrahedras weighting completed: %u cells, %u faces (%s)\r\n", delaunay.number_of_cells(), delaunay.number_of_facets(),
            TD_TIMER_GET_FMT().c_str());
        }

    // run graph-cut and extract the mesh
    {
        TD_TIMER_STARTD();

        // create graph
        MaxFlow< cell_size_t, edge_cap_t > graph(delaunay.number_of_cells());
        // set weights
        for (delaunay_t::All_cells_iterator ci = delaunay.all_cells_begin(), ce = delaunay.all_cells_end(); ci != ce; ++ci)
        {
            const cell_size_t  ciID(ci->info());
            const cell_info_t& ciInfo(infoCells[ciID]);
            graph.AddNode(ciID, ciInfo.weight_s, ciInfo.weight_t);
            for (int i = 0; i < 4; ++i)
            {
                const cell_handle_t cj(ci->neighbor(i));
                const cell_size_t   cjID(cj->info());
                if (cjID < ciID)
                    continue;
                const cell_info_t& cjInfo(infoCells[cjID]);
                const int          j(cj->index(ci));
                const edge_cap_t   q(
                    (1.f - MINF(computePlaneSphereAngle(delaunay, facet_t(ci, i)), computePlaneSphereAngle(delaunay, facet_t(cj, j)))) *
                    kQual);
                graph.AddEdge(ciID, cjID, ciInfo.wright_face[i] + q, cjInfo.wright_face[j] + q);
            }
        }
        infoCells.clear();
        // find graph-cut solution
        const float maxflow(graph.ComputeMaxFlow());
        // extract surface formed by the facets between inside/outside cells
        const size_t                               nEstimatedNumVerts(delaunay.number_of_vertices());
        std::unordered_map< void*, Mesh::VIndex > mapVertices;
#if defined( _MSC_VER ) && ( _MSC_VER > 1600 )
        //printf_line;
        mapVertices.reserve(nEstimatedNumVerts);
#endif
        m_mesh.vertices.Reserve((Mesh::VIndex)nEstimatedNumVerts);
        m_mesh.faces.Reserve((Mesh::FIndex)nEstimatedNumVerts * 2);

        for (delaunay_t::All_cells_iterator ci = delaunay.all_cells_begin(), ce = delaunay.all_cells_end(); ci != ce; ++ci)
        {
            const cell_size_t ciID(ci->info());
            for (int i = 0; i < 4; ++i)
            {
                if (delaunay.is_infinite(ci, i))
                    continue;
                const cell_handle_t cj(ci->neighbor(i));
                const cell_size_t   cjID(cj->info());
                if (ciID < cjID)
                    continue;
                const bool ciType(graph.IsNodeOnSrcSide(ciID));
                if (ciType == graph.IsNodeOnSrcSide(cjID))
                    continue;
                Mesh::Face& face = m_mesh.faces.AddEmpty();
                const triangle_vhandles_t tri(getTriangle(ci, i));
                for (int v = 0; v < 3; ++v)
                {
                    const vertex_handle_t vh(tri.verts[v]);
                    ASSERT(vh->point() == delaunay.triangle(ci, i)[v]);
                    const auto pairItID(
                        mapVertices.insert(std::make_pair(vh.for_compact_container(), (Mesh::VIndex)m_mesh.vertices.GetSize())));
                    if (pairItID.second) {
                        m_mesh.vertices.Insert(CGAL2MVS< Mesh::Vertex::Type >(vh->point()));
                    }
                    ASSERT(pairItID.first->second < m_mesh.vertices.GetSize());
                    face[v] = pairItID.first->second;
                }
                // correct face orientation
                if (!ciType)
                    std::swap(face[0], face[2]);
            }
        }

        delaunay.clear();

        // DEBUG_EXTRA( "Delaunay tetrahedras graph-cut completed (%g flow): %u vertices, %u faces (%s)", maxflow, m_mesh.vertices.GetSize(),
        // m_mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str() );
        printf("Delaunay tetrahedras graph-cut completed (%g flow): %u vertices, %u faces (%s)\r\n", maxflow, m_mesh.vertices.GetSize(),
            m_mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str());
    }

    // cout << "nItersFixNonManifold = " << nItersFixNonManifold << endl;
    // fix non-manifold vertices and edges
    for (unsigned i = 0; i < nItersFixNonManifold; ++i)
    {
        cout << "FixNonManifold, current iteration " << i << ", ";
        if (!m_mesh.FixNonManifold())
        {
            break;
        }
        printf("Mesh Info: have %u vertices and %u faces\r\n", m_mesh.vertices.GetSize(), m_mesh.faces.GetSize());
    }

    return true;
        }

