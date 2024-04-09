#ifndef FAST_LIO_MATCHING_HPP_
#define FAST_LIO_MATCHING_HPP_
#include <ros/ros.h>

#include <deque>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

//ndt匹配
class NDTRegistration{
  public:
    // NDTRegistration();
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_target) ;
    bool ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_source, 
                   const Eigen::Matrix4d& predict_pose, 
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud_ptr,
                   Eigen::Matrix4d& result_pose) ;
    float GetFitnessScore() ;
  
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    pcl::NormalDistributionsTransform<pcl::PointXYZI,pcl::PointXYZI>::Ptr ndt_ptr_;
};

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>()) {
    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}
bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);

    std::cout << "NDT params:" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}
bool NDTRegistration::SetInputTarget(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_target) {
    ndt_ptr_->setInputTarget(input_target);
    return true;
}

bool NDTRegistration::ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_source, 
                                const Eigen::Matrix4d& predict_pose, 
                                pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud_ptr,
                                Eigen::Matrix4d& result_pose) {
    ndt_ptr_->setInputSource(input_source);
    ndt_ptr_->align(*result_cloud_ptr, predict_pose.cast<float>());
    result_pose = ndt_ptr_->getFinalTransformation().cast<double>();
    return true;
}

float NDTRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
}



//点云滤波的接口，通过多态选择滤波器
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) = 0;
};

//不需要点云滤波
class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) override;

};
NoFilter::NoFilter() {}

bool NoFilter::Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>(*input_cloud_ptr));
    return true;
}

//体素滤波
class VoxelFilter: public CloudFilterInterface {
  public:
    //VoxelFilter();
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
};
VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    std::cout << "Voxel Filter params:" << std::endl
            << leaf_size_x << ", "
            << leaf_size_y << ", "
            << leaf_size_z 
            << std::endl << std::endl;

    return true;
}
bool VoxelFilter::Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) {
    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}

//长方体滤波，用于生成（匹配的局部地图）
class BoxFilter: public CloudFilterInterface {
  public:
    BoxFilter(std::vector<float> input_par);
    BoxFilter() = default;

    bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) override;

    void SetSize(std::vector<float> size);
    void SetOrigin(std::vector<float> origin);
    std::vector<float> GetEdge();

  private:
    void CalculateEdge();

  private:
    pcl::CropBox<pcl::PointXYZI> pcl_box_filter_;

    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;
};

BoxFilter::BoxFilter(std::vector<float> input_par) {
    size_.resize(6);
    edge_.resize(6);
    origin_.resize(3);

    for (size_t i = 0; i < size_.size(); i++) {
        size_.at(i) = input_par.at(i);
    }
    SetSize(size_);
}

bool BoxFilter::Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud_ptr) {
    output_cloud_ptr->clear();
    pcl_box_filter_.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
    pcl_box_filter_.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
    pcl_box_filter_.setInputCloud(input_cloud_ptr);
    pcl_box_filter_.filter(*output_cloud_ptr);

    return true;
}

void BoxFilter::SetSize(std::vector<float> size) {
    size_ = size;
    std::cout << "Box Filter params:" << std::endl
              << "min_x: " << size.at(0) << ", "
              << "max_x: " << size.at(1) << ", "
              << "min_y: " << size.at(2) << ", "
              << "max_y: " << size.at(3) << ", "
              << "min_z: " << size.at(4) << ", "
              << "max_z: " << size.at(5)
              << std::endl << std::endl;
    
    CalculateEdge();
}

void BoxFilter::SetOrigin(std::vector<float> origin) {
    origin_ = origin;
    CalculateEdge();
}

void BoxFilter::CalculateEdge() {
    for (size_t i = 0; i < origin_.size(); ++i) {
        edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);
        edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i);
    }
}

std::vector<float> BoxFilter::GetEdge() {
    return edge_;
}


class Matching {
  public:
    Matching(ros::NodeHandle& nh);

    bool Registe_2_globalmap(const PointCloudXYZI::Ptr& cloud_data,Eigen::Matrix4d& predict_from_imu,Eigen::Matrix4d& cloud_pose);

    bool InitFromGuess(const Eigen::Matrix4d& guess_pose,const PointCloudXYZI::Ptr &init_pointcloud,Eigen::Matrix4d& result);

    bool SetInitPose(const Eigen::Matrix4d& init_pose);
    bool SetInited(void);

    Eigen::Matrix4d GetInitPose(void);
    void GetGlobalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& global_map);
    pcl::PointCloud<pcl::PointXYZI>::Ptr& GetLocalMap();
    pcl::PointCloud<pcl::PointXYZI>::Ptr& GetCurrentScan();
    bool HasInited();
    bool HasNewGlobalMap();
    bool HasNewLocalMap();

  private:
    bool InitWithConfig();
    bool InitDataPath();
    bool InitScanContextManager();
    bool InitRegistration(std::shared_ptr<NDTRegistration>& registration_ptr, ros::NodeHandle& nh);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr,ros::NodeHandle& nh);
    bool InitBoxFilter(ros::NodeHandle& nh);

    bool InitGlobalMap();
    bool ResetLocalMap(float x, float y, float z);

  private:
    std::string map_path_ = "";

    std::shared_ptr<NDTRegistration> registration_ptr_; 

    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    std::shared_ptr<BoxFilter> box_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_ptr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_ptr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_ptr_;

    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();

    bool has_inited_ = false;
    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;

    float init_threshold;

};

Matching::Matching(ros::NodeHandle& nh)
    : global_map_ptr_(new pcl::PointCloud<pcl::PointXYZI>),
      local_map_ptr_(new pcl::PointCloud<pcl::PointXYZI>),
      current_scan_ptr_(new pcl::PointCloud<pcl::PointXYZI>) 
{
    std::cout << std::endl
              << "-----------------Init Localization-------------------" 
              << std::endl;
    nh.param<std::string>("globalmap_dir", map_path_, "/Downloads/LOAM/");//全局地图路径
    nh.param<float>("init_threshold",init_threshold,0.5);
    std::cout << "全局地图路径" << map_path_ << std::endl;

    InitRegistration(registration_ptr_, nh);

    // a. global map filter -- downsample point cloud map for visualization:
    InitFilter("global_map", global_map_filter_ptr_, nh);
    // b. local map filter -- downsample & ROI filtering for scan-map matching:
    InitBoxFilter(nh);
    InitFilter("local_map", local_map_filter_ptr_, nh);
    // c. scan filter -- 
    InitFilter("frame", frame_filter_ptr_, nh);

    InitGlobalMap();

    ResetLocalMap(0.0, 0.0, 0.0);
}

bool Matching::InitRegistration(std::shared_ptr<NDTRegistration>& registration_ptr, ros::NodeHandle& nh) {
    float res,step_size,trans_eps;
    int max_iter ;
    nh.param<float>("NDT/res",res,1.0);
    nh.param<float>("NDT/step_size",step_size,0.1);
    nh.param<float>("NDT/trans_eps",trans_eps,0.01);
    nh.param<int>("NDT/max_iter",max_iter,10);
    registration_ptr = std::make_shared<NDTRegistration>(res,step_size,trans_eps,max_iter);
    return true;
}

bool Matching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr,ros::NodeHandle& nh) {
    std::string filter_method;
    nh.param<string>(filter_user+"_filter",filter_method,"empty");
    std::cout << "\tFilter Method for " << filter_user << ": " << filter_method << std::endl;

    if (filter_method == "voxel_filter") {
        std::vector<float> filter_vec;
        float leaf_size_x,leaf_size_y,leaf_size_z;
        nh.param<vector<float>>(filter_method + "/" + filter_user,filter_vec,vector<float>());
        leaf_size_x = filter_vec[0];
        leaf_size_y = filter_vec[1];
        leaf_size_z = filter_vec[2];
        filter_ptr = std::make_shared<VoxelFilter>(leaf_size_x,leaf_size_y,leaf_size_z);
    } else if (filter_method == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        std::cout << "Filter method " << filter_method << " for " << filter_user << " NOT FOUND!";
        return false;
    }

    return true;
}

bool Matching::InitBoxFilter(ros::NodeHandle& nh) {
    std::vector<float> box_vec;
    nh.param<vector<float>>("box_filter_size",box_vec,vector<float>());
    box_filter_ptr_ = std::make_shared<BoxFilter>(box_vec);
    return true;
}

bool Matching::InitGlobalMap() {
    pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
    std::cout << "Load global map, size:" << global_map_ptr_->points.size();

    // since scan-map matching is used, here apply the same filter to local map & scan:
    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
    std::cout << "Filtered global map, size:" << global_map_ptr_->points.size() << std::endl;

    has_new_global_map_ = true;

    return true;
}

bool Matching::ResetLocalMap(float x, float y, float z) {
    std::vector<float> origin = {x, y, z};
    //根据当前位置切割出一个立方体当作局部地图，加入inputtarget
    // use ROI filtering for local map segmentation:
    box_filter_ptr_->SetOrigin(origin);
    box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

    local_map_filter_ptr_->Filter(local_map_ptr_,local_map_ptr_);

    registration_ptr_->SetInputTarget(local_map_ptr_);

    has_new_local_map_ = true;

    std::vector<float> edge = box_filter_ptr_->GetEdge();
    std::cout << "New local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;

    return true;
}

// TODO: understand this function
bool Matching::SetInitPose(const Eigen::Matrix4d& init_pose) {
    init_pose_ = init_pose;
    ResetLocalMap(init_pose(0,3), init_pose(1,3), init_pose(2,3));

    return true;
}

bool Matching::SetInited(void) {
    has_inited_ = true;

    return true;
}

Eigen::Matrix4d Matching::GetInitPose(void) {
    return init_pose_;
}

void Matching::GetGlobalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& global_map) {
    // downsample global map for visualization:
    global_map_filter_ptr_->Filter(global_map_ptr_, global_map);

    has_new_global_map_ = false;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr& Matching::GetLocalMap() {
    has_new_local_map_ = false;
    return local_map_ptr_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr& Matching::GetCurrentScan() {
    return current_scan_ptr_;
}

bool Matching::HasInited() {
    return has_inited_;
}

bool Matching::HasNewGlobalMap() {
    return has_new_global_map_;
}

bool Matching::HasNewLocalMap() {
    return has_new_local_map_;
}

/**
 * @brief 通过rviz给的初始猜测进行位姿初始化
 * 
 * @param guess_pose 猜测的位姿
 * @param init_pointcloud 当前帧点云
 * @param result 匹配好的位姿
 * @return 是否初始化成功
 */
bool Matching::InitFromGuess(const Eigen::Matrix4d& guess_pose,const PointCloudXYZI::Ptr &init_pointcloud,Eigen::Matrix4d& result){
    ResetLocalMap(guess_pose(0,3),guess_pose(1,3),guess_pose(2,3));//重建用于初始化的局部地图

    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr init_input(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::copyPointCloud(*init_pointcloud,*init_input);
    registration_ptr_->ScanMatch(init_input, guess_pose, result_cloud_ptr, result);

    if(registration_ptr_->GetFitnessScore()>init_threshold){
        std::cout << "initialization false , register score:" << registration_ptr_->GetFitnessScore() << std::endl;
        pcl::transformPointCloud(*init_input, *current_scan_ptr_, result);
        return false;
    }
    std::cout << "initialization success , register score:" << registration_ptr_->GetFitnessScore() << std::endl;
    pcl::transformPointCloud(*init_input, *current_scan_ptr_, result);
    return true;
}

/**
 * @brief 通过当前帧点云与世界系的局部地图匹配，得到匹配位姿
 * 
 * @param cloud_data IMU系下的点云
 * @param predict_from_imu imu前向传播得到的位姿（世界系）
 * @param cloud_pose 匹配好的位姿
 * @return 是否初始化成功
 */
bool Matching::Registe_2_globalmap(const PointCloudXYZI::Ptr& cloud_data,Eigen::Matrix4d& predict_from_imu ,Eigen::Matrix4d& cloud_pose){
    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr reg_input(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*cloud_data,*reg_input);
    // downsample:
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    frame_filter_ptr_->Filter(reg_input, filtered_cloud_ptr);
        
    // matching:
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_from_imu, result_cloud_ptr, cloud_pose);
    pcl::transformPointCloud(*reg_input, *current_scan_ptr_, cloud_pose);

    // 匹配之后判断是否需要更新局部地图
    std::vector<float> edge = box_filter_ptr_->GetEdge();
    for (int i = 0; i < 3; i++) {
        if (
            fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
            fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0
        ) {
            continue;
        }
            
        ResetLocalMap(cloud_pose(0,3), cloud_pose(1,3), cloud_pose(2,3));
        break;
    }

    return true;
}


#endif