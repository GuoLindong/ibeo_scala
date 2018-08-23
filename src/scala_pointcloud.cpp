// 2018-08-23 CyberC3@SJTU

#define Pi (3.14159265)

#include <cstdlib>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <ibeosdk/scala.hpp>
#include <ibeosdk/CommandId.hpp>
#include <ibeosdk/devices/IbeoDeviceBase.hpp>
#include <ibeosdk/devices/IbeoScala.hpp>
#include <ibeosdk/devices/IbeoDevice.hpp>
#include <ibeosdk/devices/IbeoEthDevice.hpp>
#include <ibeosdk/devices/IbeoEthType.hpp>
#include <ibeosdk/IpHelper.hpp>

#include <ibeo_scala/Object.h>
#include <ibeo_scala/ObjectArray.h>
#include <ibeo_scala/Point2D.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/PCLPointCloud2.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl-1.7/pcl/common/common_headers.h>
#include <pcl-1.7/pcl/common/transforms.h>

ibeosdk::IbeoSDK ibeoSDK;
const ibeosdk::Version::MajorVersion majorVersion(5);
const ibeosdk::Version::MinorVersion minorVersion(2);
const ibeosdk::Version::Revision revision(2);
const ibeosdk::Version::PatchLevel patchLevel;
const ibeosdk::Version::Build build;
const std::string info = "IbeoSdkScalaNodeROS";

ibeosdk::Version appVersion(majorVersion, minorVersion, revision, patchLevel, build, info);

ibeosdk::TimeConversion tc;

class AllScalaListener : public ibeosdk::DataListener<ibeosdk::FrameEndSeparator>,
                         public ibeosdk::DataListener<ibeosdk::Scan2208>,
                         public ibeosdk::DataListener<ibeosdk::ObjectListScala2271>,
                         public ibeosdk::DataListener<ibeosdk::DeviceStatus6303>{

public:
    ros::NodeHandle nh;
    ros::Publisher pub_scala_points;
    ros::Publisher pub_scala_objects;

    ibeo_scala::Object object;
    ibeo_scala::Point2D contour_points;
    ibeo_scala::ObjectArray object_array;

    float min_distance;

    AllScalaListener()
    {
        pub_scala_points = nh.advertise<sensor_msgs::PointCloud2>("scala_points", 5);
        pub_scala_objects = nh.advertise<ibeo_scala::ObjectArray>("scala_objects", 5);
    }

    virtual ~AllScalaListener(){}

public:
    void setMinDistance(float min)
    {
        min_distance = min;
    }

    void onData(const ibeosdk::FrameEndSeparator* const fes)
    {
        //ibeosdk::logInfo << std::setw(5) << fes->getSerializedSize() << " Bytes "
        //                        << "Frame received: # " << fes->getFrameId() << std::endl;
    }

    void onData(const ibeosdk::Scan2208* const scanList)
    {
        std::vector<ibeosdk::SubScan2208> subscan = scanList->getSubScans();
        std::vector<ibeosdk::ScanPoint2208> scanpoint;
        float dis,angle;
        int flags;

        pcl::PointCloud<pcl::PointXYZI>::Ptr scala_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        scala_cloud->is_dense = false;

        for(int i = 0; i < subscan.size(); ++i)
        {
            scanpoint = subscan[i].getScanPoints();
            int scan_point_size = scanpoint.size();
            //printf("points num: %i\n", scan_point_size);

            for(int j = 0; j < scan_point_size; ++j)
            {
                pcl::PointXYZI point;
                ibeosdk::ScanPoint2208 scala_point = scanpoint[j];
                flags = scala_point.getFlags();
                if(!(flags & (ibeosdk::ScanPoint2208::SPFML_Ground | ibeosdk::ScanPoint2208::SPFML_Dirt | ibeosdk::ScanPoint2208::SPFML_Rain)))
                {
                    dis = scala_point.getRadialDistance()/100.0;   // 米
                    if (dis<min_distance)
                        continue;
                    angle = -90 + (scala_point.getHorizontalAngle()/32.0);  //
                    //layer to phi
                    int layer = scala_point.getLayerId();//0123
                    double phi = layer * 0.8 - 1.2;
                    point.intensity = scala_point.getEchoPulseWidth();
                    point.z = dis * (sin(DEG2RAD(phi)));
                    point.x = dis * (cos(DEG2RAD(phi))) * (cos(DEG2RAD(angle)));
                    point.y = dis * (cos(DEG2RAD(phi))) * (sin(DEG2RAD(angle)));
                    scala_cloud->push_back(point);
                }
            }
        }

        sensor_msgs::PointCloud2 cloud_to_pub;
        pcl::toROSMsg(*scala_cloud, cloud_to_pub);//pcl::toROSMsg() should be done before setting frame_id
        cloud_to_pub.header.frame_id = "scala";
        cloud_to_pub.header.stamp = ros::Time::now();

        pub_scala_points.publish(cloud_to_pub);
    }

    void onData(const ibeosdk::ObjectListScala2271* const objectList)
    {
        std::vector<ibeosdk::ObjectScala2271> object_points = objectList->getObjects();
        object_array.objects.clear();
        //printf("objects num: %i\n", (int)object_points.size());

        for(int i =0; i<object_points.size();++i)
        {
            object.id = object_points[i].getObjectId();
            object.age = object_points[i].getFilteredObjectAttributes().getObjectAge();
            object.classification = object_points[i].getFilteredObjectAttributes().getClassification();
            object.velocity.x = object_points[i].getFilteredObjectAttributes().getRelativeVelocitySigma().getX()/100.0;
            object.velocity.y = object_points[i].getFilteredObjectAttributes().getRelativeVelocitySigma().getX()/100.0;
            object.object_closest.x = object_points[i].getFilteredObjectAttributes().getPositionClosestObjectPoint().getX()/100.0;
            object.object_closest.y = object_points[i].getFilteredObjectAttributes().getPositionClosestObjectPoint().getY()/100.0;
            object.object_box_size.x = object_points[i].getFilteredObjectAttributes().getObjectBoxSizeSigma().getX()/100.0;
            object.object_box_size.y = object_points[i].getFilteredObjectAttributes().getObjectBoxSizeSigma().getY()/100.0;

            for(int j = 0; j < object_points[i].getFilteredObjectAttributes().getContourPoints().size();++j)
            {
                contour_points.x = object_points[i].getFilteredObjectAttributes().getContourPoints().at(j).getXSigma()/100.0;
                contour_points.y = object_points[i].getFilteredObjectAttributes().getContourPoints().at(j).getYSigma()/100.0;
                object.contour.push_back(contour_points);
            }
            object_array.objects.push_back(object);
        }
        object_array.header.frame_id = "scala";
        object_array.header.stamp = ros::Time::now();
        pub_scala_objects.publish(object_array);
    }

    void onData(const ibeosdk::DeviceStatus6303* const status)
    {
        // ibeosdk::logInfo << std::setw(5) << status->getSerializedSize() << " Bytes "
        //                                  << "DevStat 0x6306 received" << std::endl;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "scala_node");
    ros::NodeHandle pnh("~");

    int scala_type;
    std::string device_destination_ip;//for scala B3
    std::string device_ip;//for scala B2
    std::string port;
    std::string log_level;
    std::string multicast_interface_ip;
    bool use_multicast;
    float min_distance;

    pnh.param<int>("scala_type", scala_type, 3);
    pnh.param<std::string>("device_destination_ip", device_destination_ip, "224.100.100.100");
    pnh.param<std::string>("device_ip", device_ip, "192.168.1.53");
    pnh.param<std::string>("port", port, "22017");
    pnh.param<std::string>("multicast_interface_ip", multicast_interface_ip, "224.0.0.1");
    pnh.param<std::string>("log_level", log_level, "Quiet");
    pnh.param<bool>("use_multicast", use_multicast, true);
    pnh.param<float>("min_distance", min_distance, 0.5);

    std::cerr << argv[0] << " Version " << appVersion.toString();
    std::cerr << " using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;

    //log file and log_level
    const off_t magLogFileSize = 1000000;
    ibeosdk::LogFileManager logFileManager;
    ibeosdk::LogFile::setTargetFileSize(magLogFileSize);
    const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString(log_level);
    ibeosdk::LogFile::setLogLevel(ll);
    logFileManager.start();

    //network address configuration
    std::stringstream ss;
    ss.str(port);
    uint16_t portVal;
    ss >> portVal;
    const boost::asio::ip::address_v4 interface_ip; //gateway
    interface_ip.from_string(multicast_interface_ip);

    boost::shared_ptr<ibeosdk::IbeoScala> scala;

    switch (scala_type)
    {
        //using scalaB2
        case 2:
            scala.reset(new ibeosdk::IbeoScala(device_ip,
                                               portVal,
                                               ibeosdk::IbeoTypeEthTcp()));
            break;
        //using scala B3
        case 3:
            if (use_multicast) {
                scala.reset(new ibeosdk::IbeoScala(device_destination_ip,
                                                   portVal,
                                                   ibeosdk::IbeoTypeEthUdpMulticast(),
                                                   interface_ip));
            } else {
                scala.reset(new ibeosdk::IbeoScala(device_destination_ip,
                                                   portVal,
                                                   ibeosdk::IbeoTypeEthUdp(),
                                                   interface_ip));
            }
            break;
        default:
            printf("should set correct scala_type in launch file.\n");
            break;
    }

    AllScalaListener allScalaListener;
    allScalaListener.setMinDistance(min_distance);
    scala->setLogFileManager(&logFileManager);
    scala->registerListener(&allScalaListener);
    scala->getConnected();

    if(!scala->isConnected())
    {
        printf("!scala.isConnected\n");
        return -1;
    }

    ros::spin();
    ros::waitForShutdown();
    return 0;
}






