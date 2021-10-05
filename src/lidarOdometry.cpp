/**
 * Created by haocaichao on 2021/8/29
 */

#include "header.h"

typedef pcl::PointXYZI PointType;

int isDone=1;
float planeMax=0.5;
std::mutex mBuf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

struct PlaneFeatureCost{
    const Eigen::Vector3d _po,_pa,_norm;

    PlaneFeatureCost(Eigen::Vector3d po,Eigen::Vector3d pa,Eigen::Vector3d norm):
            _po(po),_pa(pa),_norm(norm){}

    template <typename T>
    bool operator()(const T* q,const T* t,T* residual) const {
        Eigen::Matrix<T,3,1> po_curr{T(_po[0]),T(_po[1]),T(_po[2])};
        Eigen::Matrix<T,3,1> pa_last{T(_pa[0]),T(_pa[1]),T(_pa[2])};
        Eigen::Matrix<T,3,1> p_norm{T(_norm[0]),T(_norm[1]),T(_norm[2])};
        Eigen::Quaternion<T> q_last_curr{q[3],q[0],q[1],q[2]};
        Eigen::Matrix<T,3,1> t_last_curr{t[0],t[1],t[2]};
        Eigen::Matrix<T,3,1> po_last;
        po_last=q_last_curr*po_curr+t_last_curr;
        residual[0]=((po_last-pa_last).dot(p_norm));
        return true;
    }
};

ros::Subscriber sub_plane_frame_cloud;
ros::Publisher pub_plane_frame_cloud;
ros::Publisher pub_frame_odometry;
ros::Publisher pub_frame_odom_path;
ros::Publisher pub_sum_lidar_odom_cloud;
pcl::PointCloud<PointType>::Ptr lastPlaneCloudPtr(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr currPlaneCloudPtr(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr sumPlaneCloudPtr(new pcl::PointCloud<PointType>());
std::queue<sensor_msgs::PointCloud2> planeQueue;
nav_msgs::Path lidarPathInOdom;
std_msgs::Header currHead;
double timePlane=0;
int numFrame=0;
int flagStart=0;
double para_q[4]={0,0,0,1};
double para_t[3]={0,0,0};
Eigen::Quaterniond q_0_curr(1,0,0,0);
Eigen::Vector3d t_0_curr(0,0,0);
Eigen::Quaterniond q_0_last(1,0,0,0);
Eigen::Vector3d t_0_last(0,0,0);
Eigen::Quaterniond q_last_curr=Eigen::Map<Eigen::Quaterniond>(para_q);
Eigen::Vector3d t_last_curr=Eigen::Map<Eigen::Vector3d>(para_t);

void transformToLast(PointType const *const pi,PointType *const po){
    Eigen::Vector3d curr_point(pi->x,pi->y,pi->z);
    Eigen::Vector3d proj_point;
    proj_point=q_last_curr*curr_point+t_last_curr;
    po->x=proj_point.x();
    po->y=proj_point.y();
    po->z=proj_point.z();
    po->intensity=pi->intensity;
}

void publishResult(){
    q_0_curr = q_0_last * q_last_curr ;
    t_0_curr = t_0_last + q_0_last * t_last_curr;
    q_0_last = q_0_curr;
    t_0_last = t_0_curr;

    nav_msgs::Odometry lidarOdometry;
    lidarOdometry.header.frame_id = "map";
    lidarOdometry.child_frame_id = "map_child";
    lidarOdometry.header.stamp = currHead.stamp;
    lidarOdometry.pose.pose.position.x = t_0_curr[0];
    lidarOdometry.pose.pose.position.y = t_0_curr[1];
    lidarOdometry.pose.pose.position.z = t_0_curr[2];
    lidarOdometry.pose.pose.orientation.x = q_0_curr.x();
    lidarOdometry.pose.pose.orientation.y = q_0_curr.y();
    lidarOdometry.pose.pose.orientation.z = q_0_curr.z();
    lidarOdometry.pose.pose.orientation.w = q_0_curr.w();
    pub_frame_odometry.publish(lidarOdometry);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = lidarOdometry.header.stamp;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = lidarOdometry.pose.pose;
    lidarPathInOdom.poses.push_back(pose_stamped);
    lidarPathInOdom.header.stamp=lidarOdometry.header.stamp;
    lidarPathInOdom.header.frame_id="map";
    pub_frame_odom_path.publish(lidarPathInOdom);

    sensor_msgs::PointCloud2 plane_frame_cloud_msgs;
    pcl::toROSMsg(*currPlaneCloudPtr, plane_frame_cloud_msgs);
    plane_frame_cloud_msgs.header.stamp = lidarOdometry.header.stamp;
    plane_frame_cloud_msgs.header.frame_id = "map";
    pub_plane_frame_cloud.publish(plane_frame_cloud_msgs);

//    if(numFrame % 10 == 0){
//        double r,p,y;
//        tf::Quaternion tq(q_0_curr.x(),q_0_curr.y(),q_0_curr.z(),q_0_curr.w());
//        tf::Matrix3x3(tq).getRPY(r,p,y);
//        Eigen::Affine3d transCurd ;
//        pcl::getTransformation(t_0_curr.x(), t_0_curr.y(), t_0_curr.z(), r,p,y,transCurd);
//        pcl::PointCloud<PointType>::Ptr cloud_res(new pcl::PointCloud<PointType>());
//        pcl::transformPointCloud(*currPlaneCloudPtr, *cloud_res, transCurd);
//        *sumPlaneCloudPtr += *cloud_res;
//        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
//        downSizeFilterMap.setInputCloud(sumPlaneCloudPtr);
//        downSizeFilterMap.filter(*cloud_temp);
//        sensor_msgs::PointCloud2 res_cloud_msgs;
//        pcl::toROSMsg(*cloud_temp, res_cloud_msgs);
//        res_cloud_msgs.header.stamp = lidarOdometry.header.stamp;
//        res_cloud_msgs.header.frame_id = "map";
//        pub_sum_lidar_odom_cloud.publish(res_cloud_msgs);
//    }
}

void scanRegistration(){
    ceres::LossFunction *loss_function=new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization=new ceres::EigenQuaternionParameterization();
    ceres::Problem problem;
    problem.AddParameterBlock(para_q,4,q_parameterization);
    problem.AddParameterBlock(para_t,3);
    pcl::KdTreeFLANN<PointType> kdTreePlanLast;
    int last_plane_num=lastPlaneCloudPtr->points.size();
    int curr_plane_num=currPlaneCloudPtr->points.size();
    if(last_plane_num>10){
        kdTreePlanLast.setInputCloud(lastPlaneCloudPtr);
        for(int i_opt=0;i_opt<2;i_opt++){
            for (int i = 0; i < curr_plane_num; ++i) {
                PointType pointSeed;
                transformToLast(&currPlaneCloudPtr->points[i],&pointSeed);
                std::vector<float> pointSearchSqDis1;
                std::vector<int> indx1;
                kdTreePlanLast.nearestKSearch(pointSeed,1,indx1,pointSearchSqDis1);
                int p_ind_a=indx1[0];
                std::vector<float> pointSearchSqDis2;
                std::vector<int> indx2;
                kdTreePlanLast.nearestKSearch(lastPlaneCloudPtr->points[p_ind_a],30,indx2,pointSearchSqDis2);
                std::vector<int> v_indx5;
                std::vector<int> v_indx_row;
                int p_row=-1;
                if(indx2.size()<5) continue;
                int n=5;
                for (size_t i_kd = 0; i_kd < indx2.size(); ++i_kd) {
                    float f_indx=lastPlaneCloudPtr->points[indx2[i_kd]].intensity;
                    int i_indx=int(f_indx);
                    int row=100*(f_indx-i_indx+0.002);
                    if(i_kd==0){
                        p_row=row;
                    }
                    if(i_kd<5){
                        v_indx5.push_back(indx2[i_kd]);
                    }else{
                        if(row != p_row && row>=0 && row <=63){
                            v_indx_row.push_back(indx2[i_kd]);
                            n=i_kd;
                            if(v_indx_row.size()>=2){
                                break;
                            }
                        }
                    }
                }
                if(v_indx_row.size()==1){
                    v_indx5[4]=v_indx_row[0];
                }
                if(v_indx_row.size()==2){
                    v_indx5[3]=v_indx_row[0];
                    v_indx5[4]=v_indx_row[1];
                }

                if(pointSearchSqDis2[n]<1){
                    Eigen::Matrix<float, 5, 3> matA0;
                    Eigen::Matrix<float, 5, 1> matB0;
                    Eigen::Vector3f matX0;
                    matA0.setZero();
                    matB0.fill(-1);
                    matX0.setZero();
                    for (int j = 0; j < 5; ++j) {
                        matA0(j,0)=lastPlaneCloudPtr->points[v_indx5[j]].x;
                        matA0(j,1)=lastPlaneCloudPtr->points[v_indx5[j]].y;
                        matA0(j,2)=lastPlaneCloudPtr->points[v_indx5[j]].z;
                    }
                    matX0=matA0.colPivHouseholderQr().solve(matB0);
                    matX0.normalize();  //norm
                    bool planeValid = true;
                    for (int k = 0; k < 4; ++k) {
                        Eigen::Vector3d v_temp(
                                lastPlaneCloudPtr->points[v_indx5[k]].x-lastPlaneCloudPtr->points[v_indx5[k+1]].x,
                                lastPlaneCloudPtr->points[v_indx5[k]].y-lastPlaneCloudPtr->points[v_indx5[k+1]].y,
                                lastPlaneCloudPtr->points[v_indx5[k]].z-lastPlaneCloudPtr->points[v_indx5[k+1]].z
                        );
                        if(fabs(matX0(0)*v_temp[0]+matX0(1)*v_temp[1]+matX0(2)*v_temp[2])>planeMax){
                            planeValid=false;
                            break;
                        }
                    }
                    Eigen::Vector3d po(currPlaneCloudPtr->points[i].x,currPlaneCloudPtr->points[i].y,currPlaneCloudPtr->points[i].z);
                    Eigen::Vector3d pa(lastPlaneCloudPtr->points[p_ind_a].x,lastPlaneCloudPtr->points[p_ind_a].y,lastPlaneCloudPtr->points[p_ind_a].z);
                    Eigen::Vector3d norm(matX0[0],matX0[1],matX0[2]);
                    if(planeValid){
                        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<PlaneFeatureCost,1,4,3>
                                                         (new PlaneFeatureCost(po,pa,norm)),loss_function,para_q,para_t);
                    }
                }
            }
        }
        ceres::Solver::Options options;
        options.linear_solver_type=ceres::DENSE_QR;
        options.max_num_iterations=8;
        options.minimizer_progress_to_stdout=false;
        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);

        q_last_curr=Eigen::Map<Eigen::Quaterniond>(para_q);
        t_last_curr=Eigen::Map<Eigen::Vector3d>(para_t);
    }
}

void cldHandler(const sensor_msgs::PointCloud2ConstPtr &cldMsg) {
    mBuf.lock();
    planeQueue.push(*cldMsg);
    mBuf.unlock();
}

void cloudThread(){
    ros::Rate rate(10);
    while(1){
        rate.sleep();
        ros::Rate rate2(50);
        if(isDone==0) continue;
        while (planeQueue.size()>0){
            if(isDone==0) continue;
            isDone=0;
            numFrame++;
            rate2.sleep();
            mBuf.lock();
            currPlaneCloudPtr->clear();
            currHead=planeQueue.front().header;
            timePlane=planeQueue.front().header.stamp.toSec();
            pcl::fromROSMsg(planeQueue.front(),*currPlaneCloudPtr);
            planeQueue.pop();
            mBuf.unlock();
            if(flagStart==0){
                flagStart=1;
            }else{
                scanRegistration();
                publishResult();
            }
            *lastPlaneCloudPtr=*currPlaneCloudPtr;
            isDone=1;
        }
    }
}

int main(int argc, char **argv) {
    if(N_SCAN_ROW==16){
        planeMax=0.15;
    }
    if(N_SCAN_ROW==64){
        planeMax=0.05;
    }
    downSizeFilterMap.setLeafSize(0.4,0.4,0.4);

    ros::init(argc, argv, "LidarOdometry");
    ros::NodeHandle nh;
    sub_plane_frame_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/plane_frame_cloud1", 10, cldHandler);
    pub_plane_frame_cloud=nh.advertise<sensor_msgs::PointCloud2>("/plane_frame_cloud2",100);
    pub_frame_odometry = nh.advertise<nav_msgs::Odometry>("/frame_odom2", 100);
    pub_frame_odom_path = nh.advertise<nav_msgs::Path>("/frame_odom_path2", 100);
    pub_sum_lidar_odom_cloud = nh.advertise<sensor_msgs::PointCloud2>("/sum_lidar_odom_cloud2", 10);
    ROS_INFO("\033[1;32m----> LidarOdometry Started.\033[0m");
    std::thread t_cloud_thread(cloudThread);
    ros::spin();
    t_cloud_thread.join();
    return 0;
}
