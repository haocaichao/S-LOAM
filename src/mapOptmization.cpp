#include "header.h"

typedef PointXYZIRPYT  PointTypePose6D;
typedef pcl::PointXYZI PointTypePose3D;
typedef pcl::PointXYZI PointType;

ros::Subscriber sub_plane_frame_cloud;
ros::Subscriber sub_frame_Odometry;
ros::Publisher pub_sum_map_cloud;      //点云地图
ros::Publisher pub_map_frame;          //当前帧
ros::Publisher pub_map_odometry;       //里程计
ros::Publisher pub_map_path;           //路径
nav_msgs::Path globalPath;
pcl::PointCloud<PointType>::Ptr currFramePlanePtr(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr mapCloudPtr(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypePose3D>::Ptr keyPose3DCloud(new pcl::PointCloud<PointTypePose3D>());
pcl::PointCloud<PointTypePose6D>::Ptr keyPose6DCloud(new pcl::PointCloud<PointTypePose6D>());
std::vector<pcl::PointCloud<PointType>> keyFrameVector;
pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());
pcl::VoxelGrid<PointType> downSizeFilterICP;
pcl::VoxelGrid<PointType> downSizeFilterMap;
std_msgs::Header currHead;
std::queue<sensor_msgs::PointCloud2ConstPtr> planeQueue;
std::queue<sensor_msgs::PointCloud2ConstPtr> planeBuf;
std::queue<nav_msgs::OdometryConstPtr> odometryBuf;
std::mutex mBuf;
double timePlane=0;
double timeOdom=0;
double arr6d[6]={0,0,0,0,0,0};
Eigen::Quaterniond q_fodom_0_curr(1,0,0,0);
Eigen::Vector3d t_fodom_0_curr(0,0,0);
Eigen::Affine3d T_fodom_0_curr=Eigen::Affine3d::Identity();
Eigen::Affine3d T_map_0_curr=Eigen::Affine3d::Identity();
Eigen::Affine3d trans_map_0_curr_i=Eigen::Affine3d::Identity();
Eigen::Affine3d trans_loop_adjust=Eigen::Affine3d::Identity();
int isDone=1;
int numFrame=0;
bool aLoopIsClosed = false;
size_t loopRecordIndex=0;
Eigen::Affine3d correctionLidarFrame;  // 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
std::map<int, int> loopIndexContainer;
gtsam::NonlinearFactorGraph gtSAMgraph;   // gtsam
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;
gtsam::ISAM2Params parameters;

void planeCloudHandler(const sensor_msgs::PointCloud2ConstPtr &planeCloudMsg) {
    mBuf.lock();
    planeBuf.push(planeCloudMsg);
    mBuf.unlock();
}

void odomHandler(const nav_msgs::Odometry::ConstPtr &odomMsg){
    mBuf.lock();
    odometryBuf.push(odomMsg);
    mBuf.unlock();
}

void addPose3D6D(double x,double y,double z,double roll,double pitch,double yaw,double time){
    PointTypePose3D p3d;
    PointTypePose6D p6d;
    p3d.x=x;
    p3d.y=y;
    p3d.z=z;
    p3d.intensity=keyPose3DCloud->size();   // index ,now size==0, before push_back
    p6d.x=p3d.x;
    p6d.y=p3d.y;
    p6d.z=p3d.z;
    p6d.intensity=p3d.intensity;
    p6d.roll=roll;
    p6d.pitch=pitch;
    p6d.yaw=yaw;
    p6d.time=time;
    keyPose3DCloud->push_back(p3d);
    keyPose6DCloud->push_back(p6d);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(time);
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;
    pose_stamped.pose.position.z = z;
    tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    globalPath.poses.push_back(pose_stamped);
}

//更新里程计轨迹
void updatePath(const PointTypePose6D& pose_in,int indx)
{
    globalPath.poses[indx].pose.position.x = pose_in.x;
    globalPath.poses[indx].pose.position.y = pose_in.y;
    globalPath.poses[indx].pose.position.z = pose_in.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    globalPath.poses[indx].pose.orientation.x = q.x();
    globalPath.poses[indx].pose.orientation.y = q.y();
    globalPath.poses[indx].pose.orientation.z = q.z();
    globalPath.poses[indx].pose.orientation.w = q.w();
}

//计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
bool isKeyFrame()
{
    if (keyPose3DCloud->points.empty())
        return true;
    PointTypePose6D thisPoint=keyPose6DCloud->back(); // 前一帧位姿
    Eigen::Affine3d T_key_0_last;
    pcl::getTransformation(thisPoint.x,thisPoint.y,thisPoint.z,thisPoint.roll,thisPoint.pitch,thisPoint.yaw,T_key_0_last);
    //当前帧与前一帧位姿变换增量
    Eigen::Affine3d transBetween = T_key_0_last.inverse() * T_map_0_curr;
    double x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
    // 旋转和平移量都较小，当前帧不设为关键帧
    if (abs(roll)  < 0.01 && abs(pitch) < 0.01 && abs(yaw)   < 0.01 &&
        sqrt(x*x + y*y + z*z) < 1)
        return false;
    return true;
}

//添加激光里程计因子
void addOdomFactor()
{
    gtsam::Pose3 currPose(gtsam::Rot3::RzRyRx(arr6d[3], arr6d[4], arr6d[5]),gtsam::Point3(arr6d[0], arr6d[1], arr6d[2]));
    if (keyPose3DCloud->points.size()==1){
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, currPose, priorNoise)); // 第一帧初始化先验因子
        initialEstimate.insert(0, currPose);  // 变量节点设置初始值
    }
    if (keyPose3DCloud->points.size()>1)
    {
        PointTypePose6D thisPoint=keyPose6DCloud->points[keyPose3DCloud->points.size()-2];
        gtsam::Pose3 lastPost(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                              gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        // 添加激光里程计因子 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(keyPose3DCloud->size()-2, keyPose3DCloud->size()-1, lastPost.between(currPose), odometryNoise));
        initialEstimate.insert(keyPose3DCloud->size()-1, currPose);  // 变量节点设置初始值
    }
}

// 闭环检测，找出闭环关键帧序号用于后续闭环判断；候选帧选择依据是此帧距离最近，时间久远超过阈值。
bool detectLoopFrameID(int *latestID, int *closestID)
{
    int loopKeyCur = - 1;  // 当前关键帧序号
    loopKeyCur = keyPose3DCloud->size() - 1;  // 当前关键帧序号
    int loopKeyPre = -1;
    // 当前帧已经添加过闭环对应关系，不再继续添加
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;
    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(keyPose3DCloud);
    kdtreeHistoryKeyPoses->radiusSearch(keyPose3DCloud->back(), 15, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    // 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if (abs(keyPose6DCloud->points[id].time - keyPose6DCloud->points[loopKeyCur].time) > 20)
        {
            loopKeyPre = id;
            break;
        }
    }
    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;
    *latestID = loopKeyCur;
    *closestID = loopKeyPre;
    loopRecordIndex=loopKeyCur+2;
    return true;
}

// 提取闭环关键帧周围的点云组成局部地图
void getLoopLocalMap(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
{
    nearKeyframes->clear();
    int cloudSize = keyPose6DCloud->size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize )
            continue;
        Eigen::Affine3d trans;
        PointTypePose6D pt6d=keyPose6DCloud->points[keyNear];
        pcl::getTransformation(pt6d.x, pt6d.y, pt6d.z, pt6d.roll, pt6d.pitch, pt6d.yaw,trans);
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(keyFrameVector[keyNear], *cloud_temp, trans);
        *nearKeyframes += *cloud_temp;
    }
    if (nearKeyframes->empty())
        return;
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

void addLoopFactor(){
    if (keyPose3DCloud->points.size()<5 || keyPose3DCloud->points.size()-1<=loopRecordIndex)
        return;
    int loopKeyCur;
    int loopKeyPre;
    if(detectLoopFrameID(&loopKeyCur, &loopKeyPre)){
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            getLoopLocalMap(cureKeyframeCloud, loopKeyCur, 0);  // 提取当前关键帧
            getLoopLocalMap(prevKeyframeCloud, loopKeyPre, 10);  // 提取闭环匹配关键帧前后相邻若干帧
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
        }
        static pcl::IterativeClosestPoint<PointType, PointType> icp;  // ICP参数设置
        icp.setMaxCorrespondenceDistance(50);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);
        if (icp.hasConverged() == false || icp.getFitnessScore() > 0.2)// 未收敛，或者匹配不够好
            return;
        // 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
        correctionLidarFrame=icp.getFinalTransformation().cast<double>();
        loopRecordIndex=loopRecordIndex+30;
        // 闭环优化前当前帧位姿
        Eigen::Affine3d tWrong;
        PointTypePose6D pt6d=keyPose6DCloud->points[loopKeyCur];
        pcl::getTransformation(pt6d.x, pt6d.y, pt6d.z, pt6d.roll, pt6d.pitch, pt6d.yaw,tWrong);
        // 闭环优化后当前帧位姿
        Eigen::Affine3d tCorrect = correctionLidarFrame * tWrong;
        double x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        // 闭环匹配帧的位姿
        PointTypePose6D thisPoint=keyPose6DCloud->points[loopKeyPre];
        gtsam::Pose3 poseTo(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                              gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

        loopIndexContainer[loopKeyCur] = loopKeyPre;
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(loopKeyCur, loopKeyPre, poseFrom.between(poseTo), constraintNoise));
        aLoopIsClosed = true;
    }
}

void runOptimize(){
    isam->update(gtSAMgraph, initialEstimate);   // 执行优化
    isam->update();
    if (aLoopIsClosed == true)
    {
        for (int j = 0; j < 6; ++j) {
            isam->update();
        }
    }
    // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
    gtSAMgraph.resize(0);
    initialEstimate.clear();
    isamCurrentEstimate = isam->calculateEstimate();  // 优化结果
}

void updateKeyPose(int i){
    keyPose3DCloud->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
    keyPose3DCloud->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
    keyPose3DCloud->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();
    keyPose6DCloud->points[i].x = keyPose3DCloud->points[i].x;
    keyPose6DCloud->points[i].y = keyPose3DCloud->points[i].y;
    keyPose6DCloud->points[i].z = keyPose3DCloud->points[i].z;
    keyPose6DCloud->points[i].roll  = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
    keyPose6DCloud->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
    keyPose6DCloud->points[i].yaw   = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();
    updatePath(keyPose6DCloud->points[i],i);  // 更新里程计轨迹
    pcl::getTransformation(keyPose6DCloud->points[i].x, keyPose6DCloud->points[i].y, keyPose6DCloud->points[i].z,
            keyPose6DCloud->points[i].roll, keyPose6DCloud->points[i].pitch,keyPose6DCloud->points[i].yaw,trans_map_0_curr_i);
    pcl::PointCloud<PointType>::Ptr cloud_res_i(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(keyFrameVector[i], *cloud_res_i, trans_map_0_curr_i);
    *mapCloudPtr += *cloud_res_i;
}

// 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
void correctPoses()
{
    int numPoses = isamCurrentEstimate.size();
    if (aLoopIsClosed == true)
    {
        mapCloudPtr->clear();
        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
        for (int i = 0; i < numPoses; ++i)
        {
            updateKeyPose(i);
        }
        aLoopIsClosed = false;
        trans_loop_adjust=trans_loop_adjust*correctionLidarFrame;
    }else{
        updateKeyPose(numPoses-1);
    }
    T_map_0_curr=trans_map_0_curr_i;
}

void publishResult(){
    Eigen::Quaterniond eq;
    eq=T_map_0_curr.rotation();
    Eigen::Vector3d ev=T_map_0_curr.translation();
    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "map";
    laserOdometry.child_frame_id = "map_child";
    laserOdometry.header.stamp = currHead.stamp;
    laserOdometry.pose.pose.orientation.x = eq.x();
    laserOdometry.pose.pose.orientation.y = eq.y();
    laserOdometry.pose.pose.orientation.z = eq.z();
    laserOdometry.pose.pose.orientation.w = eq.w();
    laserOdometry.pose.pose.position.x = ev.x();
    laserOdometry.pose.pose.position.y = ev.y();
    laserOdometry.pose.pose.position.z = ev.z();
    pub_map_odometry.publish(laserOdometry);

    globalPath.header.stamp=laserOdometry.header.stamp;
    globalPath.header.frame_id="map";
    pub_map_path.publish(globalPath);

    pcl::PointCloud<PointType>::Ptr cloud_res(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*currFramePlanePtr, *cloud_res, T_map_0_curr);
    sensor_msgs::PointCloud2 res_cloud_msgs;
    pcl::toROSMsg(*cloud_res, res_cloud_msgs);
    res_cloud_msgs.header.stamp = laserOdometry.header.stamp;
    res_cloud_msgs.header.frame_id = "map";
    pub_map_frame.publish(res_cloud_msgs);

    if(numFrame % 5 == 0){
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterMap.setInputCloud(mapCloudPtr);
        downSizeFilterMap.filter(*cloud_temp);
        sensor_msgs::PointCloud2 res_map_cloud_msgs;
        pcl::toROSMsg(*cloud_temp, res_map_cloud_msgs);
        res_map_cloud_msgs.header.stamp = currHead.stamp;
        res_map_cloud_msgs.header.frame_id = "map";
        pub_sum_map_cloud.publish(res_map_cloud_msgs);
    }
}

void mapOptimization(){
    if(isKeyFrame()){
        // get x, y, z, roll, pitch, yaw
        pcl::getTranslationAndEulerAngles(T_map_0_curr,arr6d[0],arr6d[1],arr6d[2],arr6d[3],arr6d[4],arr6d[5]);
        addPose3D6D(arr6d[0],arr6d[1],arr6d[2],arr6d[3],arr6d[4],arr6d[5],timeOdom);
        keyFrameVector.push_back(*currFramePlanePtr);  // save key frame
        addOdomFactor();   // 激光里程计因子
        addLoopFactor();  // 闭环因子
        runOptimize();    // 执行优化
        correctPoses();   // 更新里程计
        publishResult();
    }
}

void cloudThread(){
    ros::Rate rate(20);
    while(1){
        rate.sleep();
        if(isDone==0) continue;
        while (!odometryBuf.empty() && !planeBuf.empty()){
            if(isDone==0) continue;
            mBuf.lock();
            currHead.stamp=planeBuf.front()->header.stamp;
            timePlane=planeBuf.front()->header.stamp.toSec();
            timeOdom=odometryBuf.front()->header.stamp.toSec();
            if(std::fabs(timePlane-timeOdom)>0.005){
                printf("frame time unsync messeage! \n");
                mBuf.unlock();
                break;
            }
            isDone=0;
            numFrame++;
            currFramePlanePtr->clear();
            pcl::fromROSMsg(*planeBuf.front(),*currFramePlanePtr);
            planeBuf.pop();
            q_fodom_0_curr.x()=odometryBuf.front()->pose.pose.orientation.x;
            q_fodom_0_curr.y()=odometryBuf.front()->pose.pose.orientation.y;
            q_fodom_0_curr.z()=odometryBuf.front()->pose.pose.orientation.z;
            q_fodom_0_curr.w()=odometryBuf.front()->pose.pose.orientation.w;
            t_fodom_0_curr.x()=odometryBuf.front()->pose.pose.position.x;
            t_fodom_0_curr.y()=odometryBuf.front()->pose.pose.position.y;
            t_fodom_0_curr.z()=odometryBuf.front()->pose.pose.position.z;
            odometryBuf.pop();
            mBuf.unlock();
            T_fodom_0_curr=Eigen::Affine3d::Identity();
            T_fodom_0_curr.rotate(q_fodom_0_curr);
            T_fodom_0_curr.pretranslate(t_fodom_0_curr);
            T_map_0_curr=trans_loop_adjust*T_fodom_0_curr;
            mapOptimization();
            isDone=1;
        }
    }
}

int main(int argc, char **argv) {
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);
    downSizeFilterICP.setLeafSize(0.1,0.1,0.1);
    downSizeFilterMap.setLeafSize(0.4,0.4,0.4);

    ros::init(argc, argv, "MapOptmization");
    ros::NodeHandle nh;
    sub_plane_frame_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/plane_frame_cloud2", 10, planeCloudHandler);
    sub_frame_Odometry = nh.subscribe<nav_msgs::Odometry>("/frame_odom2", 100, odomHandler);
    pub_map_frame = nh.advertise<sensor_msgs::PointCloud2>("/map_frame_res3", 10);
    pub_sum_map_cloud = nh.advertise<sensor_msgs::PointCloud2>("/sum_map_cloud_res3", 10);
    pub_map_odometry = nh.advertise<nav_msgs::Odometry>("/map_odom_res3", 100);
    pub_map_path = nh.advertise<nav_msgs::Path>("/map_laser_path_res3", 100);
    ROS_INFO("\033[1;32m----> MapOptmization Started.\033[0m");
    std::thread t_cloud_thread(cloudThread);
    ros::spin();
    t_cloud_thread.join();
    return 0;
}
