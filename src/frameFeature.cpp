/**
 * Created by haocaichao on 2021/8/29
 *
 * 点云特征提取
 * （1）接收原始点云
 * （2）重建点云索引
 * （3）计算点云曲率
 * （4）提取平面特征点，点云抽稀
 * （5）发布点云
 */

#include "header.h"

typedef VelodynePointXYZIRT PointType;   //点类型名称重定义，用于接收点云中各点
typedef pcl::PointXYZI PointTypeOut;     //pcl点类型名称重定义，简化

ros::Subscriber sub_lidar_frame_cloud;   //定义ros订阅者，接收激光雷达
ros::Publisher pub_plane_frame_cloud;    //定义ros发布者，发布平面特征点云
ros::Publisher pub_org_frame_cloud;      //定义ros发布者，发布原始点云
//定义pcl点云对象，存储原始点云
pcl::PointCloud<PointType>::Ptr framePtr(new pcl::PointCloud<PointType>());
//定义pcl点云对象，存储平面特征点云
pcl::PointCloud<PointTypeOut>::Ptr framePlanePtr(new pcl::PointCloud<PointTypeOut>());
//定义容器，存储每线点云
std::vector<pcl::PointCloud<PointType>> v_scan_row = std::vector<pcl::PointCloud<PointType>>(N_SCAN_ROW);
//定义容器，存储每线点云中各点信息
std::vector<std::vector<PointInfo>> v_scan_row_info = std::vector<std::vector<PointInfo>>(N_SCAN_ROW);
float planeMin=0.5;      //定义平面曲率最小门槛值
int planeSpan=2;         //定义点间隔，用于抽稀
int rowIndexStart=0;     //定义点云线内点起点索引
int rowIndexEnd=0;       //定义点云线内点终点索引
pcl::VoxelGrid<PointTypeOut> downSizeFilterPlane;  //定义点云下采样对象，用于点云抽稀

//接收原始点云，处理，发布
void cldHandler(const sensor_msgs::PointCloud2ConstPtr &cldMsg) {
    framePtr->clear();                    //存储点云之前需要先清空
    framePlanePtr->clear();
    v_scan_row = std::vector<pcl::PointCloud<PointType>>(N_SCAN_ROW);
    v_scan_row_info = std::vector<std::vector<PointInfo>>(N_SCAN_ROW);
    //将ros点云消息类型转换为pcl点云对象
    pcl::fromROSMsg(*cldMsg, *framePtr);
    //遍历点云各点，重建点云索引
    for (size_t i = 0; i < framePtr->points.size(); ++i) {
        PointType point;
        point.x = framePtr->points[i].x;
        point.y = framePtr->points[i].y;
        point.z = framePtr->points[i].z;
        point.intensity = framePtr->points[i].intensity;
        int scanID = -1;
        int flag = 2;         //1-使用原始点云线号ring信息  2-根据垂向角度计算点云线号
        if (flag == 1) {
//            scanID = framePtr->points[i].ring;
        } else {
            //计算垂向角度
            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            if (N_SCAN_ROW == 16) {     //16点云线号计算
                if (angle >= -15 || angle <= 15) {
                    scanID = int((angle + 15) / 2 + 0.5);
                }
            }
            if (N_SCAN_ROW == 64) {     //64点云线号计算
                if (angle >= -24.33 || angle <= 2) {
                    if (angle >= -8.83){
                        scanID = int((2 - angle) * 3.0 + 0.5);
                    }else{
                        scanID = N_SCAN_ROW / 2 + int((-8.83 - angle) * 2.0 + 0.5);
                    }
                }
            }
        }
        if (scanID > -1 && scanID < N_SCAN_ROW) {        //每条扫描线是一个点云对象
            PointInfo p_info;
            p_info.value = 0;
            p_info.indexInRow = v_scan_row[scanID].size();
            point.intensity=p_info.indexInRow+scanID/100.0;
            v_scan_row[scanID].push_back(point);         //用数组存储各线点云数据
            v_scan_row_info[scanID].push_back(p_info);   //用另一个数组同步存储各点信息
        }
    }

    //计算点云曲率
    for (int i = 0+rowIndexStart; i < N_SCAN_ROW-rowIndexEnd; i++) {
        for (int j = 5; j < int(v_scan_row[i].size()) - 5; j++) {
            float diffX =
                    v_scan_row[i].points[j - 5].x + v_scan_row[i].points[j - 4].x + v_scan_row[i].points[j - 3].x +
                    v_scan_row[i].points[j - 2].x + v_scan_row[i].points[j - 1].x
                    - 10 * v_scan_row[i].points[j].x
                    + v_scan_row[i].points[j + 1].x + v_scan_row[i].points[j + 2].x + v_scan_row[i].points[j + 3].x +
                    v_scan_row[i].points[j + 4].x + v_scan_row[i].points[j + 5].x;
            float diffY =
                    v_scan_row[i].points[j - 5].y + v_scan_row[i].points[j - 4].y + v_scan_row[i].points[j - 3].y +
                    v_scan_row[i].points[j - 2].y + v_scan_row[i].points[j - 1].y
                    - 10 * v_scan_row[i].points[j].y
                    + v_scan_row[i].points[j + 1].y + v_scan_row[i].points[j + 2].y + v_scan_row[i].points[j + 3].y +
                    v_scan_row[i].points[j + 4].y + v_scan_row[i].points[j + 5].y;
            float diffZ =
                    v_scan_row[i].points[j - 5].z + v_scan_row[i].points[j - 4].z + v_scan_row[i].points[j - 3].z +
                    v_scan_row[i].points[j - 2].z + v_scan_row[i].points[j - 1].z
                    - 10 * v_scan_row[i].points[j].z
                    + v_scan_row[i].points[j + 1].z + v_scan_row[i].points[j + 2].z + v_scan_row[i].points[j + 3].z +
                    v_scan_row[i].points[j + 4].z + v_scan_row[i].points[j + 5].z;
            //存储各线各点曲率值
            v_scan_row_info[i][j].value = (diffX * diffX + diffY * diffY + diffZ * diffZ);
        }
    }

    //遍历各线，再遍历各点，根据曲率门槛值筛选出平面特征点云
    for (int i = 0+rowIndexStart; i < N_SCAN_ROW-rowIndexEnd; i++) {
        size_t jstart = 0;
        for (size_t j = 0; j < v_scan_row_info[i].size(); j++) {
            if (j >= jstart && v_scan_row_info[i][j].value < planeMin) {
                PointTypeOut pt;
                pt.x = v_scan_row[i][v_scan_row_info[i][j].indexInRow].x;
                pt.y = v_scan_row[i][v_scan_row_info[i][j].indexInRow].y;
                pt.z = v_scan_row[i][v_scan_row_info[i][j].indexInRow].z;
                pt.intensity = v_scan_row[i][v_scan_row_info[i][j].indexInRow].intensity;
                framePlanePtr->push_back(pt);
                jstart = j + planeSpan;      //按指定间隔提取点云，相当于抽稀
            }
        }
    }

    //点云下采样，抽稀
    pcl::PointCloud<PointTypeOut>::Ptr cloud_temp(new pcl::PointCloud<PointTypeOut>());
    downSizeFilterPlane.setInputCloud(framePlanePtr);
    downSizeFilterPlane.filter(*cloud_temp);
    //发布平面特征点云
    sensor_msgs::PointCloud2 planeCloudMsg;
    pcl::toROSMsg(*framePlanePtr, planeCloudMsg);      //将pcl点云对象转换为ros点云消息类型
    planeCloudMsg.header.stamp = cldMsg->header.stamp;
    planeCloudMsg.header.frame_id = "map";
    pub_plane_frame_cloud.publish(planeCloudMsg);
    //发布原始点云
    sensor_msgs::PointCloud2 orgCloudMsg;
    orgCloudMsg=*cldMsg;
    orgCloudMsg.header.frame_id="map";
    pub_org_frame_cloud.publish(orgCloudMsg);
}


int main(int argc, char **argv) {
    //针对不同线数激光雷达数据，设置不同参数
    if(N_SCAN_ROW==16){
        planeMin=0.05;
        planeSpan=3;
    }
    if(N_SCAN_ROW==64){
        planeMin=0.005;
        planeSpan=25;
        rowIndexStart=5;
        rowIndexEnd=5;
    }
    downSizeFilterPlane.setLeafSize(0.2,0.2,0.2);   //设置抽稀参数

    ros::init(argc, argv, "FrameFeature");
    ros::NodeHandle nh;
    //订阅原始激光雷达数据
    sub_lidar_frame_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, cldHandler);
    //发布平面特征点云
    pub_plane_frame_cloud = nh.advertise<sensor_msgs::PointCloud2>("/plane_frame_cloud1", 100);
    //发布原始点云
    pub_org_frame_cloud = nh.advertise<sensor_msgs::PointCloud2>("/org_frame_cloud1", 100);
    ROS_INFO("\033[1;32m----> FrameFeature Started.\033[0m");
    ros::spin();
    return 0;
}
