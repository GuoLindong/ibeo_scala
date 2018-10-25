// 2018-08-23 CyberC3@SJTU

#define Pi (3.14159265)

#include <cstdlib>
#include <iostream>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
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
#include <ibeo_scala/VehicleState.h>

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
                         public ibeosdk::DataListener<ibeosdk::VehicleStateBasicLux>{

public:
    ros::NodeHandle nh;
    ros::Publisher pub_scala_points;
    ros::Publisher pub_scala_objects;
    ros::Publisher pub_scala_vehicle_state;

    //ibeo_scala::Object object;
    //ibeo_scala::ObjectArray object_array;

    float min_distance;

    AllScalaListener()
    {
        pub_scala_points = nh.advertise<sensor_msgs::PointCloud2>("scala_points", 1);
        pub_scala_objects = nh.advertise<ibeo_scala::ObjectArray>("scala_objects", 1);
        pub_scala_vehicle_state = nh.advertise<ibeo_scala::VehicleState>("scala_vehicle_state",1);
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
                    dis = scala_point.getRadialDistance()/100.0;   // m
                    if (dis<min_distance)
                        continue;
                    angle = -90 + (scala_point.getHorizontalAngle()/32.0);  //
                    //layer to phi
                    int layer = scala_point.getLayerId();//0123
                    double phi = layer * 0.8 - 1.2;
                    point.intensity = scala_point.getEchoPulseWidth();
                    //for B2.x
//                    point.z = dis * (sin(DEG2RAD(phi)));
//                    point.x = dis * (cos(DEG2RAD(phi))) * (cos(DEG2RAD(angle)));
//                    point.y = dis * (cos(DEG2RAD(phi))) * (sin(DEG2RAD(angle)));
                    //for B3.0
                    point.z = dis * (sin(DEG2RAD(phi)));
                    point.x = dis * -(cos(DEG2RAD(phi))) * (sin(DEG2RAD(angle)));
                    point.y = dis * (cos(DEG2RAD(phi))) * (cos(DEG2RAD(angle)));
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
        std::vector<ibeosdk::ObjectScala2271> object_list = objectList->getObjects();
        ibeo_scala::ObjectArray object_array;
        int object_list_size = object_list.size();
//        ROS_INFO("object number: %i", object_list_size);
        for(int i =0; i<object_list_size; ++i)
        {
            ibeo_scala::Object object;
            object.id = object_list[i].getObjectId();
            object.age = object_list[i].getFilteredObjectAttributes().getObjectAge();
            object.classification = object_list[i].getFilteredObjectAttributes().getClassification();
            object.hidden_state_age = object_list[i].getFilteredObjectAttributes().getHiddenStatusAge();
            object.dynamic_flag = object_list[i].getFilteredObjectAttributes().getDynamicFlag();
            object.velocity_absolute.x = object_list[i].getFilteredObjectAttributes().getAbsoluteVelocity().getX()/100.0;
            object.velocity_absolute.y = object_list[i].getFilteredObjectAttributes().getAbsoluteVelocity().getY()/100.0;
//            object.velocity_relative.x = object_list[i].getFilteredObjectAttributes().getRelativeVelocity().getX()/100.0;
//            object.velocity_relative.y = object_list[i].getFilteredObjectAttributes().getRelativeVelocity().getY()/100.0;
            object.object_closest.x = object_list[i].getFilteredObjectAttributes().getPositionClosestObjectPoint().getX()/100.0;
            object.object_closest.y = object_list[i].getFilteredObjectAttributes().getPositionClosestObjectPoint().getY()/100.0;
            for (int k = 0; k < 10; ++k) {
                ibeo_scala::Point2D reference_point_coord;
                object_list[i].getFilteredObjectAttributes().setReferencePointLocation(ibeosdk::RefPointBoxLocation(k));
                reference_point_coord.x = object_list[i].getFilteredObjectAttributes().getReferencePointCoord().getX()/100.0;
                reference_point_coord.y = object_list[i].getFilteredObjectAttributes().getReferencePointCoord().getY()/100.0;
                object.reference_points.push_back(reference_point_coord);
            }
            object.object_box_orientation = object_list[i].getFilteredObjectAttributes().getObjectBoxOrientation()/100.0;
            object.object_box_height = object_list[i].getFilteredObjectAttributes().getObjectBoxHeight()/100.0;
            object.object_box_size.x = object_list[i].getFilteredObjectAttributes().getObjectBoxSize().getX()/100.0;
            object.object_box_size.y = object_list[i].getFilteredObjectAttributes().getObjectBoxSize().getY()/100.0;
//            for(int j = 0; j < object_list[i].getFilteredObjectAttributes().getContourPoints().size();++j)
//            {
//                contour_points.x = object_list[i].getFilteredObjectAttributes().getContourPoints()[j].getX()/100.0;
//                contour_points.y = object_list[i].getFilteredObjectAttributes().getContourPoints()[j].getY()/100.0;
//                object.contour.push_back(contour_points);
//            }
            object_array.objects.push_back(object);
        }
        object_array.header.frame_id = "scala";
        object_array.header.stamp = ros::Time::now();
        pub_scala_objects.publish(object_array);
    }

    void onData(const ibeosdk::VehicleStateBasicLux* const state)
    {
        ibeo_scala::VehicleState vehicle_state;
        vehicle_state.current_yaw_rate = state->getCurrentYawRate()/10000.0;
        vehicle_state.longitudinal_velocity = state->getLongitudinalVelocity()/100.0;
        pub_scala_vehicle_state.publish(vehicle_state);
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

    boost::shared_ptr<ibeosdk::IbeoScala> scala(new ibeosdk::IbeoScala(device_ip,
                                                                       portVal,
                                                                       ibeosdk::IbeoTypeEthTcp()));

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



/*enum ibeosdk::rawObjectClass::RawObjectClass

0 RawObjectClass_Unclassified
1 RawObjectClass_UnknownSmall
2 RawObjectClass_UnknownBig
3 RawObjectClass_Pedestrian
4 RawObjectClass_Bike
5 RawObjectClass_Car
6 RawObjectClass_Truck
7 RawObjectClass_Overdrivable
8 RawObjectClass_Underdrivable
9 RawObjectClass_Bicycle
10 RawObjectClass_Motorbike
11 RawObjectClass_Infrastructure  */


/*enum ibeosdk::RefPointBoxLocation
 *
0 RPL_CenterOfGravity
1 RPL_FrontLeft
2 RPL_FrontRight
3 RPL_RearRight
4 RPL_RearLeft
5 RPL_FrontCenter
6 RPL_RightCenter
7 RPL_RearCenter
8 RPL_LeftCenter
9 RPL_ObjectCenter
10 RPL_Unknown */






