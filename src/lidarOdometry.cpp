/**
 * Created by haocaichao on 2021/8/29
 *
 * 激光里程计的计算（帧帧配准）
 * （1）接收帧平面特征点云
 * （2）计算当前帧与上一帧之间位姿变换（帧帧配准，ceres优化）
 * （3）累计帧间变换，得到从起始帧开始的里程计
 * （4）发布点云、里程计、轨迹、地图
 */

#include "header.h"

typedef pcl::PointXYZI PointType;

int isDone=1;         //标识运算是否完成
float planeMax=0.5;   //平面判断门槛值
std::mutex mLock;      //多线程锁
pcl::VoxelGrid<PointType> downSizeFilterMap;  //定义点云下采样对象，用于点云抽稀

/**
 * 定义ceres优化中的代价函数
 * 点到平面的距离计算用向量表示
 * 在同一坐标系中，两点间(po,pa)向量值与平面法向量(norm)的点乘可得到距离
 */
struct PlaneFeatureCost{
    const Eigen::Vector3d _po,_pa,_norm;

    PlaneFeatureCost(Eigen::Vector3d po,Eigen::Vector3d pa,Eigen::Vector3d norm):
            _po(po),_pa(pa),_norm(norm){}

    template <typename T>
    bool operator()(const T* q,const T* t,T* residual) const {
        Eigen::Matrix<T,3,1> po_curr{T(_po[0]),T(_po[1]),T(_po[2])};
        Eigen::Matrix<T,3,1> pa_last{T(_pa[0]),T(_pa[1]),T(_pa[2])};
        Eigen::Matrix<T,3,1> p_norm{T(_norm[0]),T(_norm[1]),T(_norm[2])};
        Eigen::Quaternion<T> q_last_curr{q[3],q[0],q[1],q[2]};   //用于坐标系变换统一
        Eigen::Matrix<T,3,1> t_last_curr{t[0],t[1],t[2]};        //用于坐标系变换统一
        Eigen::Matrix<T,3,1> po_last;
        po_last=q_last_curr*po_curr+t_last_curr;
        residual[0]=((po_last-pa_last).dot(p_norm));
        return true;
    }
};

ros::Subscriber sub_plane_frame_cloud;      //接收平面特征点云
ros::Publisher pub_plane_frame_cloud;       //发布平面特征点云
ros::Publisher pub_frame_odometry;          //发布激光雷达里程计，由帧帧配准计算得到
ros::Publisher pub_frame_odom_path;         //发布激光雷达运动轨迹
ros::Publisher pub_sum_lidar_odom_cloud;    //发布拼接后的点云地图
//存储当前帧点云
pcl::PointCloud<PointType>::Ptr lastPlaneCloudPtr(new pcl::PointCloud<PointType>());
//存储上一帧点云
pcl::PointCloud<PointType>::Ptr currPlaneCloudPtr(new pcl::PointCloud<PointType>());
//存储拼接后总点云
pcl::PointCloud<PointType>::Ptr sumPlaneCloudPtr(new pcl::PointCloud<PointType>());
std::queue<sensor_msgs::PointCloud2> planeQueue;  //定义点云消息队列
nav_msgs::Path lidarPathInOdom;        //定义激光雷达运动轨迹
std_msgs::Header currHead;             //定义ros消息头变量
double timePlane=0;                    //定义平面点云帧时间戳变量
int numFrame=0;                        //定义帧计数变量
int flagStart=0;                       //定义是否开始标志
double para_q[4]={0,0,0,1};            //定义长度为4的数组，用于构成四元数
double para_t[3]={0,0,0};              //定义长度为3的数组，用于构成位移
Eigen::Quaterniond q_0_curr(1,0,0,0);  //起点到当前帧，四元数
Eigen::Vector3d t_0_curr(0,0,0);       //起点到当前帧，位移
Eigen::Quaterniond q_0_last(1,0,0,0);  //起点到上一帧，四元数
Eigen::Vector3d t_0_last(0,0,0);       //起点到上一帧，位移
//上一帧到当前帧，四元数
Eigen::Quaterniond q_last_curr=Eigen::Map<Eigen::Quaterniond>(para_q);
//上一帧到当前帧，位移
Eigen::Vector3d t_last_curr=Eigen::Map<Eigen::Vector3d>(para_t);

//将当前点坐标变换到上一帧坐标系中
void transformToLast(PointType const *const pi,PointType *const po){
    Eigen::Vector3d curr_point(pi->x,pi->y,pi->z);
    Eigen::Vector3d proj_point;
    proj_point=q_last_curr*curr_point+t_last_curr;
    po->x=proj_point.x();
    po->y=proj_point.y();
    po->z=proj_point.z();
    po->intensity=pi->intensity;
}

//发布点云、里程计、轨迹、地图
void publishResult(){
    //累计帧间变换，得到从起点开始的里程计
    q_0_curr = q_0_last * q_last_curr ;
    t_0_curr = t_0_last + q_0_last * t_last_curr;
    q_0_last = q_0_curr;
    t_0_last = t_0_curr;
    //发布里程计
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
    //发布里轨迹
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = lidarOdometry.header.stamp;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = lidarOdometry.pose.pose;
    lidarPathInOdom.poses.push_back(pose_stamped);
    lidarPathInOdom.header.stamp=lidarOdometry.header.stamp;
    lidarPathInOdom.header.frame_id="map";
    pub_frame_odom_path.publish(lidarPathInOdom);
    //发布平面特征点云
    sensor_msgs::PointCloud2 plane_frame_cloud_msgs;
    pcl::toROSMsg(*currPlaneCloudPtr, plane_frame_cloud_msgs);
    plane_frame_cloud_msgs.header.stamp = lidarOdometry.header.stamp;
    plane_frame_cloud_msgs.header.frame_id = "map";
    pub_plane_frame_cloud.publish(plane_frame_cloud_msgs);
    //发布拼接点云地图
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

//计算帧帧配准，得到帧间位姿变换
void frameRegistration(){
    //定义ceres优化对象与参数
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
            for (int i = 0; i < curr_plane_num; ++i) {    //遍历当前帧各平面点
                PointType pointSeed;
                //将当前帧此平面点坐标变换到上一帧坐标系中
                transformToLast(&currPlaneCloudPtr->points[i],&pointSeed);
                std::vector<float> pointSearchSqDis1;
                std::vector<int> indx1;
                //将变换后的此点作为种子点，查找上一帧中距离此点最近点的索引
                kdTreePlanLast.nearestKSearch(pointSeed,1,indx1,pointSearchSqDis1);
                int p_ind_a=indx1[0];
                std::vector<float> pointSearchSqDis2;
                std::vector<int> indx2;
                //将上面最近点作为种子点，查找上一帧中距离最近点的索引和距离
                kdTreePlanLast.nearestKSearch(lastPlaneCloudPtr->points[p_ind_a],30,indx2,pointSearchSqDis2);
                std::vector<int> v_indx5;
                std::vector<int> v_indx_row;
                int p_row=-1;
                if(indx2.size()<5) continue;
                int n=5;
                //挑选5个最近点，尽量满足有2个点不属于同一扫描线
                for (size_t i_kd = 0; i_kd < indx2.size(); ++i_kd) {
                    float f_indx=lastPlaneCloudPtr->points[indx2[i_kd]].intensity;
                    int i_indx=int(f_indx);
                    int row=100*(f_indx-i_indx+0.002);   //获取点索引
                    if(i_kd==0){
                        p_row=row;
                    }
                    if(i_kd<5){       //先将最近5个点选入
                        v_indx5.push_back(indx2[i_kd]);
                    }else{            //从第6个点开始，寻找与记录不同线的最近2个点
                        if(row != p_row && row>=0 && row <=63){
                            v_indx_row.push_back(indx2[i_kd]);
                            n=i_kd;
                            if(v_indx_row.size()>=2){
                                break;
                            }
                        }
                    }
                }
                if(v_indx_row.size()==1){       //如果不同线的只有1个点
                    v_indx5[4]=v_indx_row[0];
                }
                if(v_indx_row.size()==2){       //如果不同线的有2个点
                    v_indx5[3]=v_indx_row[0];
                    v_indx5[4]=v_indx_row[1];
                }

                if(pointSearchSqDis2[n]<1){     //如果5个点中最远的点小于1米，则利用5点估算平面法线
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
                    for (int k = 0; k < 4; ++k) {   //利用法向量计算各点到平面的距离
                        Eigen::Vector3d v_temp(
                                lastPlaneCloudPtr->points[v_indx5[k]].x-lastPlaneCloudPtr->points[v_indx5[k+1]].x,
                                lastPlaneCloudPtr->points[v_indx5[k]].y-lastPlaneCloudPtr->points[v_indx5[k+1]].y,
                                lastPlaneCloudPtr->points[v_indx5[k]].z-lastPlaneCloudPtr->points[v_indx5[k+1]].z
                        );
                        if(fabs(matX0(0)*v_temp[0]+matX0(1)*v_temp[1]+matX0(2)*v_temp[2])>planeMax){
                            planeValid=false;       //如果有点到平面的距离太大，则说明此5点不共面
                            break;
                        }
                    }
                    Eigen::Vector3d po(currPlaneCloudPtr->points[i].x,currPlaneCloudPtr->points[i].y,currPlaneCloudPtr->points[i].z);
                    Eigen::Vector3d pa(lastPlaneCloudPtr->points[p_ind_a].x,lastPlaneCloudPtr->points[p_ind_a].y,lastPlaneCloudPtr->points[p_ind_a].z);
                    Eigen::Vector3d norm(matX0[0],matX0[1],matX0[2]);
                    if(planeValid){                 //当找到了共面点，就利用种子点、最近点、平面法向量，构造点与平面共面的优化条件
                        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<PlaneFeatureCost,1,4,3>
                                                         (new PlaneFeatureCost(po,pa,norm)),loss_function,para_q,para_t);
                    }
                }
            }
        }
        //设置优化参数，并优化
        ceres::Solver::Options options;
        options.linear_solver_type=ceres::DENSE_QR;
        options.max_num_iterations=8;
        options.minimizer_progress_to_stdout=false;
        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);
        //得到优化结果，构成帧间变换的方向四元数与位移
        q_last_curr=Eigen::Map<Eigen::Quaterniond>(para_q);
        t_last_curr=Eigen::Map<Eigen::Vector3d>(para_t);
    }
}

//接收平面特征点云，添加到消息队列中
void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &cldMsg) {
    mLock.lock();
    planeQueue.push(*cldMsg);
    mLock.unlock();
}

//点云处理多线程函数
void cloudThread(){
    ros::Rate rate(10);                //控制处理频率为10hz，为最终频率
    while(ros::ok()){
        rate.sleep();
        ros::Rate rate2(50);           //内层处理频率为50hz
        if(isDone==0) continue;
        while (planeQueue.size()>0){
            if(isDone==0) continue;
            isDone=0;
            numFrame++;
            rate2.sleep();

            mLock.lock();               //锁线程，取数据
            currPlaneCloudPtr->clear();
            currHead=planeQueue.front().header;
            timePlane=planeQueue.front().header.stamp.toSec();
            pcl::fromROSMsg(planeQueue.front(),*currPlaneCloudPtr);
            planeQueue.pop();
            mLock.unlock();

            if(flagStart==0){          //标志起始帧
                flagStart=1;
            }else{                     //从第二帧开始计算帧帧配准
                frameRegistration();
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
    sub_plane_frame_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/plane_frame_cloud1", 10, cloudHandler);
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
