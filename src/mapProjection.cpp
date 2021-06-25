#include "utility.h"
#include "radar_sam/cloud_info.h"


// struct OusterPointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//     (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
// )

//或许这里需要设置radar的数据结构 但暂时先就用pcl的pointcloudxyz

// Use the Velodyne point format as a common representation
// using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class mapProjection : public ParamServer
{
private:

    ros::Subscriber subMap;
    
    ros::Publisher pubGridmap;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;  //点云输入

    pcl::PointCloud<PointType>::Ptr   fullCloud;   //全点云 不知道什么意思
    pcl::PointCloud<PointType>::Ptr   extractedCloud;  //提取的点云 不知道什么意思

    cv::Mat rangeMat;  
    radar_sam::cloud_info cloudInfo;
    double timeScanCur;  //对于激光 一帧点云有开始和结束两个时间 对于毫米波 我们只使用timeScanCur
    double timeScanEnd;  //这个之后一定不再使用
    std_msgs::Header cloudHeader;


public:
    mapProjection()
    {
        subMap = nh.subscribe<sensor_msgs::PointCloud2>("radar_sam/mapping/map_global", 5, &mapProjection::mapHandler, this, ros::TransportHints().tcpNoDelay());
        
        // subMap = nh.subscribe<sensor_msgs::PointCloud2>("/radar_points2", 5, &mapProjection::mapHandler, this, ros::TransportHints().tcpNoDelay());
        //这里的回调函数应该要改//TODO:

        pubGridmap = nh.advertise<sensor_msgs::Image> ("radar_sam/mapping/gridmap", 1); //提取的点云，往外发，我们可能不需要这个

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointType>());\
        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        rangeMat = cv::Mat(N_ROW, N_COLUMN, CV_8UC1, cv::Scalar(255));
    }

    ~mapProjection(){}

    void mapHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) //TODO:
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        projectPointCloud();
        
        publishClouds();
        
        resetParameters();

    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg); //放进点云队列
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud  //点云covert
        currentCloudMsg = std::move(cloudQueue.front()); //拿到当前点云
        cloudQueue.pop_front();  //队列里pop一个
        pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn); //转成pcl消息 laserCloudIn
        

        // get timestamp 获取时间
        cloudHeader = currentCloudMsg.header;
        //手动做时延处理 这非常不合理 只是为了跑通代码
        // cloudHeader.stamp.sec -=2;
        timeScanCur = cloudHeader.stamp.toSec(); // TODO:权宜之计
        // timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
        // timeScanEnd = timeScanCur +0.2; //这里我的想法是，让timeScanEnd等于下一帧的时间，现在获取这个可能会出错，所以暂时加入magic number
        timeScanEnd =timeScanCur;
  

        return true;
    }

    void projectPointCloud() //这里有两个选择，一个是根本就不做投影之后就用点云 但可能架构改很大 一个是强行为毫米波雷达作投影 计算time和ring//TODO:
    //现在，我们选择不做投影，因为匹配不在这里，所以我们基本什么都不做，我们保留之前试图投影radar时的代码
    //我们看到，原来的架构里，这里主要输出两个东西，一个是rangemap，一个是fullCloud，我们保留fullCloud
    {
   
        // 重新进行radar投影 目的是投影 x行（一行代表一个角度）y列 （一列代表一个距离），每个位置的值是平均高度（或者数量）
        //下面是试图radar投影的代码
        int cloudSize = laserCloudIn->points.size();
        int index = 0;
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // thisPoint.intensity = laserCloudIn->points[i].intensity;

            //可以考虑用这里来写index就像liosam里一样，但是不知道有什么意义，暂时不做 TODO:
            // int index = columnIdn + rowIdn * N_COLUMN;
            //一度把这个放在最后，结果因为rangeMap的提前退出机制 使得index不对 太傻了
            index++;

            //投影ver2 投影 x行（一行代表一个x距离）y列 （一列代表一个y距离），每个位置的值是平均高度（或者数量）
            // rowIdn计算  
            int rowIdn = round(thisPoint.x/rangeResolution +N_ROW/2);

            if (rowIdn < 0 || rowIdn >= N_ROW)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            // columnIdn计算
            int columnIdn = round(thisPoint.y/rangeResolution +N_COLUMN/2);

            if (columnIdn < 0 || columnIdn >= N_COLUMN)
                continue;

            //这里做 如果这个位置已经有值了该做什么处理（什么都不做，或者计算平均高度，或者计算数量）
            if (rangeMat.at<uchar>(rowIdn, columnIdn) != 255)
            {
                //我们暂时计算平均高度
                // rangeMat.at<uchar>(rowIdn, columnIdn) = (thisPoint.z + rangeMat.at<uchar>(rowIdn, columnIdn) )/2;
                rangeMat.at<uchar>(rowIdn, columnIdn) = conventHeight2Pixel(-2,5,(thisPoint.z + rangeMat.at<uchar>(rowIdn, columnIdn) )/2);
                continue;
            }
            // thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);//这里使用time来去畸变 实际上radar不需要

            // 这里写 到底rangeMap的值是什么  （用range 或者平均高度 或者数量）
            //用range的话不太合理，因为range已经被编码进列了
            //用数量的话，绝大多数情况会是二值的
            //原版Newman是用power这种量 我们暂时使用平均高度
            rangeMat.at<uchar>(rowIdn, columnIdn) = conventHeight2Pixel(-2,5,thisPoint.z);
            
        }
    }
    
    //发布rangeMat
    void publishClouds()
    {
        sensor_msgs::Image msg = *(cv_bridge::CvImage(std_msgs::Header(), "mono8", rangeMat).toImageMsg());
        msg.header = cloudHeader;

        pubGridmap.publish(msg);

    }
    int conventHeight2Pixel(int heightlow , int heighthigh, float height)
    {
        return floor((height-heightlow)/(heighthigh-heightlow)*255);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_sam");

    //打开DEBUG调试信息
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);

    mapProjection IP;
    
    ROS_INFO("\033[1;32m----> Map Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
