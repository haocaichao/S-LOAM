#include "header.h"

typedef VelodynePointXYZIRT PointType;
typedef pcl::PointXYZI PointTypeOut;

ros::Subscriber sub_lidar_frame_cloud;
ros::Publisher pub_plane_frame_cloud;
ros::Publisher pub_org_frame_cloud;
pcl::PointCloud<PointType>::Ptr framePtr(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypeOut>::Ptr framePlanePtr(new pcl::PointCloud<PointTypeOut>());
//存储每线点云
std::vector<pcl::PointCloud<PointType>> v_scan_row = std::vector<pcl::PointCloud<PointType>>(N_SCAN_ROW);
std::vector<std::vector<PointInfo>> v_scan_row_info = std::vector<std::vector<PointInfo>>(N_SCAN_ROW);
float planeMin=0.5;
int planeSpan=2;
int rowIndexStart=0;
int rowIndexEnd=0;

void cldHandler(const sensor_msgs::PointCloud2ConstPtr &cldMsg) {
    framePtr->clear();
    framePlanePtr->clear();
    v_scan_row = std::vector<pcl::PointCloud<PointType>>(N_SCAN_ROW);
    v_scan_row_info = std::vector<std::vector<PointInfo>>(N_SCAN_ROW);
    pcl::fromROSMsg(*cldMsg, *framePtr);
    for (size_t i = 0; i < framePtr->points.size(); ++i) {
        PointType point;
        point.x = framePtr->points[i].x;
        point.y = framePtr->points[i].y;
        point.z = framePtr->points[i].z;
        point.intensity = framePtr->points[i].intensity;
        int scanID = -1;
        int flag = 2;
        if (flag == 1) {
//            scanID = framePtr->points[i].ring;
        } else {
            float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
            if (N_SCAN_ROW == 16) {
                if (angle >= -15 || angle <= 15) {
                    scanID = int((angle + 15) / 2 + 0.5);
                }
            }
            if (N_SCAN_ROW == 64) {
                if (angle >= -24.33 || angle <= 2) {
                    if (angle >= -8.83){
                        scanID = int((2 - angle) * 3.0 + 0.5);
                    }else{
                        scanID = N_SCAN_ROW / 2 + int((-8.83 - angle) * 2.0 + 0.5);
                    }
                }
            }
        }
        if (scanID > -1 && scanID < N_SCAN_ROW) {
            PointInfo p_info;
            p_info.value = 0;
            p_info.flag = 0;
            p_info.indexInRow = v_scan_row[scanID].size();
            v_scan_row_info[scanID].push_back(p_info);
            point.intensity=p_info.indexInRow+scanID/100.0;
            v_scan_row[scanID].push_back(point);
        }
    }

    //计算曲率
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
            // curve
            v_scan_row_info[i][j].value = (diffX * diffX + diffY * diffY + diffZ * diffZ);
        }
    }

    for (int i = 0+rowIndexStart; i < N_SCAN_ROW-rowIndexEnd; i++) {
        size_t jstart = 0;
        for (size_t j = 0; j < v_scan_row_info[i].size(); j++) {      // add plane points and jump 10,row points 1800
            if (j >= jstart && v_scan_row_info[i][j].value < planeMin) {
                PointTypeOut pt;
                pt.x = v_scan_row[i][v_scan_row_info[i][j].indexInRow].x;
                pt.y = v_scan_row[i][v_scan_row_info[i][j].indexInRow].y;
                pt.z = v_scan_row[i][v_scan_row_info[i][j].indexInRow].z;
                pt.intensity = v_scan_row[i][v_scan_row_info[i][j].indexInRow].intensity;
                framePlanePtr->push_back(pt);
                jstart = j + planeSpan;
            }
        }
    }
    sensor_msgs::PointCloud2 planeCloudMsg;
    pcl::toROSMsg(*framePlanePtr, planeCloudMsg);
    planeCloudMsg.header.stamp = cldMsg->header.stamp;
    planeCloudMsg.header.frame_id = "map";
    pub_plane_frame_cloud.publish(planeCloudMsg);

    sensor_msgs::PointCloud2 orgCloudMsg;
    orgCloudMsg=*cldMsg;
    orgCloudMsg.header.frame_id="map";
    pub_org_frame_cloud.publish(orgCloudMsg);
}

int main(int argc, char **argv) {
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

    ros::init(argc, argv, "FrameFeature");
    ros::NodeHandle nh;
    sub_lidar_frame_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, cldHandler);
    pub_plane_frame_cloud = nh.advertise<sensor_msgs::PointCloud2>("/plane_frame_cloud1", 100);
    pub_org_frame_cloud = nh.advertise<sensor_msgs::PointCloud2>("/org_frame_cloud1", 100);
    ROS_INFO("\033[1;32m----> FrameFeature Started.\033[0m");
    ros::spin();
    return 0;
}
