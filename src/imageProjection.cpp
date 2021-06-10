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

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;  // imu队列的当前位置指针
    bool firstPointFlag; // firstPointFlag 还不知道什么意思
    Eigen::Affine3f transStartInverse;  //一个变换，不知道什么意思

    // pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr laserCloudIn;  //点云输入

    // pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;   //全点云 不知道什么意思
    pcl::PointCloud<PointType>::Ptr   extractedCloud;  //提取的点云 不知道什么意思

    int deskewFlag;    //去畸变的flag
    cv::Mat rangeMat;  //深度图 最终我们是不用这个的 我们或许会使用gridmap
    //TODO: 一个gridmap的结构体
    //为了快速先有个架构，我们先用icp，那么就不需要gridmap，而需要的只是pcl的pointcloud
    //所以
    pcl::PointCloud<PointType>::Ptr CloudForICP;
    
    bool odomDeskewFlag;   //odom去畸变flag
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    radar_sam::cloud_info cloudInfo;
    double timeScanCur;  //对于激光 一帧点云有开始和结束两个时间 对于毫米波 我们只使用timeScanCur
    double timeScanEnd;  //这个之后一定不再使用
    std_msgs::Header cloudHeader;


public:
    ImageProjection():
    deskewFlag(0)
    {
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        //imu的订阅 应该不变

        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        //这里的回调函数应该要改//TODO:

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("radar_sam/deskew/cloud_deskewed", 1); //提取的点云，往外发，我们可能不需要这个
        pubLaserCloudInfo = nh.advertise<radar_sam::cloud_info> ("radar_sam/deskew/cloud_info", 1);   //点云数据的发送

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        // tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        // fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullCloud->points.resize(Radar_target_number);


        // cloudInfo.startRingIndex.assign(N_ROW, 0);
        // cloudInfo.endRingIndex.assign(N_ROW, 0);

        cloudInfo.pointColInd.assign(N_ROW*N_COLUMN, 0);
        cloudInfo.pointRange.assign(N_ROW*N_COLUMN, 0);

        resetParameters();
        cout<<"[DEBUG]91" <<endl;

    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_ROW, N_COLUMN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
        cout<<"[DEBUG]9" <<endl;

    }

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg) //TODO:
    {
        cout<<"[DEBUG]ImageProjection cloudHandler : into" <<endl;
        if (!cachePointCloud(laserCloudMsg))
            return;
        cout<<"[DEBUG]1" <<endl;

        if (!deskewInfo())
            return;
        cout<<"[DEBUG]2" <<endl;

        projectPointCloud();
        cout<<"[DEBUG]3" <<endl;
        
        cloudExtraction();
        cout<<"[DEBUG]4" <<endl;
        
        publishClouds();
        cout<<"[DEBUG]5" <<endl;
        
        resetParameters();
        cout<<"[DEBUG]6" <<endl;

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
        pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn); //转成ros消息 laserCloudIn
        

        // get timestamp 获取时间
        cloudHeader = currentCloudMsg.header;
        //手动做时延处理 这非常不合理 只是为了跑通代码
        // cloudHeader.stamp.sec -=2;
        timeScanCur = cloudHeader.stamp.toSec(); // TODO:权宜之计
        // timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
        // timeScanEnd = timeScanCur +0.2; //这里我的想法是，让timeScanEnd等于下一帧的时间，现在获取这个可能会出错，所以暂时加入magic number
        timeScanEnd =timeScanCur;
        //让timeScanEnd = timeScanCur+（两帧时间差）

        // check dense flag  //查看是否dense 我们肯定不做这个
        // if (laserCloudIn->is_dense == false)
        // {
        //     ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        //     ros::shutdown();
        // }

        // check ring channel
        // static int ringFlag = 0;
        // if (ringFlag == 0)
        // {
        //     ringFlag = -1;
        //     for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
        //     {
        //         if (currentCloudMsg.fields[i].name == "ring")
        //         {
        //             ringFlag = 1;
        //             break;
        //         }
        //     }
        //     if (ringFlag == -1)
        //     {
        //         ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
        //         ros::shutdown();
        //     }
        // }

        // check point time
        // if (deskewFlag == 0)
        // {
        //     deskewFlag = -1;
        //     for (auto &field : currentCloudMsg.fields)
        //     {
        //         if (field.name == "time" || field.name == "t")
        //         {
        //             deskewFlag = 1;
        //             break;
        //         }
        //     }
        //     if (deskewFlag == -1)
        //         ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        // }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan 确认IMU可用，原来的逻辑是：
        //  if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd) //TODO:
        // {
        //     ROS_DEBUG("Waiting for IMU data ...");
        //     return false;
        // }
        // imu不能为空，imu队列里最前的一个的时间不能大于当前点云时间 imu队列最后一个的时间不能小于当前点云时间（end）
        //也即 在时间先后上有 ： imuFront  ->  timecur -> timeend  -> imuBack
        //在我们雷达的情况下，我们定义这样的时间：
        //imuFront ->timecur ->timeNextFrame ->imuback
        //所以修改如下  （其实是对timeScanEnd的修改）
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd) //TODO:
        {
            ROS_DEBUG("Waiting for IMU data ...");
            ROS_DEBUG_STREAM("imu Queue front time = "<<(int)(imuQueue.front().header.stamp.toSec()));
            ROS_DEBUG_STREAM("timeScanCur = "<<(int)timeScanCur);

            ROS_DEBUG_STREAM("imu Queue back time = "<<(int)(imuQueue.back().header.stamp.toSec()));
            ROS_DEBUG_STREAM("timeScanEnd = "<<(int)timeScanEnd);



            return false;
        }
        imuDeskewInfo(); //看下面的代码，似乎这里不用改，但好像和timeScanCur 有关系 待定TODO:

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();//如果imu队列里的front时间比timeScanCur提前太多，就pop掉，直到接近
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)//遍历imu队列
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur) //在imu队列里，现在应该只有第一项在timeScanCur前了吧？所以应该只做一次
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
                //反正是获得了当前帧的rpy估计，存在cloudInfo里

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            //在imu队列第一帧，初始化imuRot为0
            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity在这之后的帧，首先得到角速度
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation然后实际上做了个积分
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }
        //最终，imuRot成为了在这帧期间imu返回的角度变化，注意，每帧初始都设置成0，所以这是个相对值
        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        //遍历odo队列
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }//最终是找到odomMsg时间正好比timeScancur大的地方

        //转成tf
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        //获得此时rpy
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        //这就是我们在mapOptimization里的初始猜测，所以我们知道，这个猜测是来源于odom
        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        //这里想拿到当前帧结束时的odom，对radar没有意义，我们先来看看它用这个来做什么
        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        //如果odom的最后一项时间都比ScanEnd小，那就没必要找了
        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        //遍历
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        //如果位姿的不确定性不一样，就return，不理解这是用来做什么的
        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        //transBegin
        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        //transEnd
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        //得到相对变换
        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        //从中拿到rpy和欧拉角，但实际上没有用? 我猜测可能本来是想加一条odom的factor，但最后没加 TODO:
        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        // newPoint.intensity = point->intensity; //对于radar 注释掉

        return newPoint;
    }

    void projectPointCloud() //这里有两个选择，一个是根本就不做投影之后就用点云 但可能架构改很大 一个是强行为毫米波雷达作投影 计算time和ring//TODO:
    //现在，我们选择不做投影，因为匹配不在这里，所以我们基本什么都不做，我们保留之前试图投影radar时的代码
    //我们看到，原来的架构里，这里主要输出两个东西，一个是rangemap，一个是fullCloud，我们保留fullCloud
    {
        cout<<"[DEBUG]11" <<endl;

        // int cloudSize = laserCloudIn->points.size();
        // int index =0;
        // for (int i = 0; i < cloudSize; ++i)
        // {
        //     PointType thisPoint;
        //     thisPoint.x = laserCloudIn->points[i].x;
        //     thisPoint.y = laserCloudIn->points[i].y;
        //     thisPoint.z = laserCloudIn->points[i].z;
        //     // int index = columnIdn + rowIdn * Horizon_SCAN;
        //     //原版的index如上，我们直接++好了，但可能需要调试
        //     fullCloud->points[index] = thisPoint;
        //     index++;
        // }
        // 上面是 如果不进行投影的话  把上面打开 而把下面全部关闭

        cout<<"[DEBUG]12" <<endl;

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
            fullCloud->points[index] = thisPoint;
            index++;

            // //投影ver1 投影 x行（一行代表一个角度）y列 （一列代表一个距离），每个位置的值是平均高度（或者数量）
            // // rowIdn计算  
            // float range = pointDistance(thisPoint);
            // if (range < radarMinRange || range > radarMaxRange)
            //     continue;


            // // int rowIdn = laserCloudIn->points[i].ring;
            // float Angle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI;

            // float ang_zero = 180;  //预留的用于调整角度初值的量 防止rowIdn有负值 设置为180意味着，冲后为角度的0点
            // float ang_res_y = 1; //角分辨率 暂时用1度 那么一共360个行
            // int rowIdn = (Angle + ang_zero) / ang_res_y;

            // if (rowIdn < 0 || rowIdn >= N_ROW)
            //     continue;

            // if (rowIdn % downsampleRate != 0)
            //     continue;

            // // columnIdn计算
            // float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            // int columnIdn = round(sqrt( thisPoint.x*thisPoint.x + thisPoint.y*thisPoint.y)/rangeResolution);

            // if (columnIdn < 0 || columnIdn >= N_COLUMN)
            //     continue;

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
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
            {
                //我们暂时计算平均高度
                rangeMat.at<float>(rowIdn, columnIdn) = (thisPoint.z + rangeMat.at<float>(rowIdn, columnIdn) )/2;
                continue;
            }
            // thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);//这里使用time来去畸变 实际上radar不需要

            // 这里写 到底rangeMap的值是什么  （用range 或者平均高度 或者数量）
            //用range的话不太合理，因为range已经被编码进列了
            //用数量的话，绝大多数情况会是二值的
            //原版Newman是用power这种量 我们暂时使用平均高度
            rangeMat.at<float>(rowIdn, columnIdn) = thisPoint.z;
            
        }
    }

    //本来是，把rangeMap存在Cloudinfo里，以及得到extractedCloud变量
    //而且似乎，所谓extractedCloud其实就是把当前帧所有点给push_back
    //补充一个知识，pcl::pointCLoud内部是用vector的形式存储points，而且其的push_bach方法(做了重载)实际是往points里push_back
    //我们这里只保留extractedCloud，之后看看这个东西怎么用
    void cloudExtraction()
    {
        int count = 0;
        int cloudSize = laserCloudIn->points.size();
        cout<<"[DEBUG]13" <<endl;

        // extract segmented cloud for lidar odometry
        for (int i = 0; i < cloudSize; ++i)
        {
            // save extracted cloud
            extractedCloud->push_back(fullCloud->points[i]);
            // size of extracted cloud
            ++count;
        }
        cout<<"[DEBUG]14" <<endl;

        // int count = 0;
        // // extract segmented cloud for lidar odometry
        // for (int i = 0; i < N_SCAN; ++i)
        // {
        //     cloudInfo.startRingIndex[i] = count - 1 + 5;

        //     for (int j = 0; j < Horizon_SCAN; ++j)
        //     {
        //         if (rangeMat.at<float>(i,j) != FLT_MAX)
        //         {
        //             // mark the points' column index for marking occlusion later
        //             cloudInfo.pointColInd[count] = j;
        //             // save range info
        //             cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
        //             // save extracted cloud
        //             extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
        //             // size of extracted cloud
        //             ++count;
        //         }
        //     }
        //     cloudInfo.endRingIndex[i] = count -1 - 5;
        // }
    }
    
    //发布cloudinfo
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;        
        cout<<"[DEBUG]15" <<endl;

        sensor_msgs::Image msg = *(cv_bridge::CvImage(std_msgs::Header(), "bgr8", rangeMat).toImageMsg());
        cloudInfo.rangeMap = msg;
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame); 
        cout<<"[DEBUG]17" <<endl;

        pubLaserCloudInfo.publish(cloudInfo);
        cout<<"[DEBUG]16" <<endl;


    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_sam");

    //打开DEBUG调试信息
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
