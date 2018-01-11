//
//
#include <ibeosdk/scala.hpp>
#include <ibeosdk/IpHelper.hpp>
#include <cstdlib>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <ibeo_scala_node/Object.h>
#include <ibeo_scala_node/ObjectArray.h>
#include <ibeo_scala_node/Point2D.h>

#include <iostream>



ibeosdk::IbeoSDK ibeoSDK;
const ibeosdk::Version::MajorVersion majorVersion(5);
const ibeosdk::Version::MinorVersion minorVersion(2);
const ibeosdk::Version::Revision revision(2);
const ibeosdk::Version::PatchLevel patchLevel;
const ibeosdk::Version::Build build;
const std::string info = "IbeoSdkScalaLiveDemo";

ibeosdk::Version appVersion(majorVersion, minorVersion, revision, patchLevel, build, info);

ibeosdk::TimeConversion tc;
class AllScalaListener : public ibeosdk::DataListener<ibeosdk::FrameEndSeparator>,
                         public ibeosdk::DataListener<ibeosdk::Scan2208>,
                         public ibeosdk::DataListener<ibeosdk::ObjectListScala2271>,
                         public ibeosdk::DataListener<ibeosdk::DeviceStatus6303>{

public:
    ros::NodeHandle nh;
    ros::Publisher pubScalaScan;
    ros::Publisher pubScalaObject;

    ibeo_scala_node::Object object;
    ibeo_scala_node::Point2D contour_points;
    ibeo_scala_node::ObjectArray objectArray;

    AllScalaListener()
    {
        pubScalaScan = nh.advertise<sensor_msgs::PointCloud2>("/scala_scan", 5);
        pubScalaObject = nh.advertise<ibeo_scala_node::ObjectArray>("/scala_object",5);
    }

    virtual ~AllScalaListener(){}

public:
    void onData(const ibeosdk::FrameEndSeparator* const fes)
    {
        //ibeosdk::logInfo << std::setw(5) << fes->getSerializedSize() << " Bytes "
        //                        << "Frame received: # " << fes->getFrameId() << std::endl;
    }

    void onData(const ibeosdk::Scan2208* const scanList)
    {
        
        std::vector<ibeosdk::SubScan2208> subscan = scanList->getSubScans();
        //std::vector <ibeosdk::ScanPoint2208> scan_points = scanList->get
        // std::cout << "number scan: " << subscan.size() << std::endl;
        std::vector<ibeosdk::ScanPoint2208> scanpoint;
        double dis,angle,x,y;
        int flags;
        int sub_scan_size = subscan.size();

        for(int i = 0; i < sub_scan_size; ++i)
        {
            scanpoint = subscan[i].getScanPoints();
            //std::cout << "pointsize: " << scanpoint.size() << std::endl;
            int scan_point_size = scanpoint.size();
            for(int j = 0; j < scanpoint.size(); ++j)
            {
                flags = scanpoint[j].getFlags();
                if(true)
                //if(!flags & (ibeosdk::ScanPoint2208::SPFML_Ground | ibeosdk::ScanPoint2208::SPFML_Dirt | ibeosdk::ScanPoint2208::SPFML_Rain))
                {
                    dis = scanpoint[j].getRadialDistance()/100.0;   // 米
                    angle = 90 - (scanpoint[j].getHorizontalAngle()/32.0);  //
                }

            }

        }

//        sensor_msgs::PointCloud2 pointcloud;
//        pointcloud.header.frame_id = "scala";
//        pointcloud.header.stamp = ros::Time::now();
//        pubScalaScan.publish(pointcloud);

    }

    void onData(const ibeosdk::ObjectListScala2271* const objectList)
    {
        std::vector<ibeosdk::ObjectScala2271> object_points = objectList->getObjects();
        objectArray.objects.clear();
        std::cout << "object num: " << object_points.size() << std::endl;

        for(int i =0; i<object_points.size();++i)
        {
            object.id = object_points.at(i).getObjectId();
            object.age = object_points.at(i).getFilteredObjectAttributes().getObjectAge();
            object.classification = object_points.at(i).getFilteredObjectAttributes().getClassification();
            object.velocity.x = object_points.at(i).getFilteredObjectAttributes().getRelativeVelocitySigma().getX()/100.0;
            object.velocity.y = object_points.at(i).getFilteredObjectAttributes().getRelativeVelocitySigma().getX()/100.0;
            object.object_closest.x = object_points.at(i).getFilteredObjectAttributes().getPositionClosestObjectPoint().getX()/100.0;
            object.object_closest.y = object_points.at(i).getFilteredObjectAttributes().getPositionClosestObjectPoint().getY()/100.0;
            object.object_box_size.x = object_points.at(i).getFilteredObjectAttributes().getObjectBoxSizeSigma().getX()/100.0;
            object.object_box_size.y = object_points.at(i).getFilteredObjectAttributes().getObjectBoxSizeSigma().getY()/100.0;

            for(int j = 0; j < object_points.at(i).getFilteredObjectAttributes().getContourPoints().size();++j)
            {
                contour_points.x = object_points.at(i).getFilteredObjectAttributes().getContourPoints().at(j).getXSigma()/100.0;
                contour_points.y = object_points.at(i).getFilteredObjectAttributes().getContourPoints().at(j).getYSigma()/100.0;

                object.contour.push_back(contour_points);
            }
            objectArray.objects.push_back(object);
        }
        objectArray.header.frame_id = "";
        objectArray.header.stamp = ros::Time::now();
        pubScalaObject.publish(objectArray);
    }

    void onData(const ibeosdk::DeviceStatus6303* const status)
    {
       // ibeosdk::logInfo << std::setw(5) << status->getSerializedSize() << " Bytes "
       //                                  << "DevStat 0x6306 received" << std::endl;

    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "scala_pointcloud");

    ros::NodeHandle pnh("~");
    // ip param
    std::string device_ip;
    int port;
    pnh.param<std::string>("device_ip",device_ip,"192.168.1.52");
    pnh.param<int>("port", port, 12004);
    std::cerr << argv[0] << " Version " << appVersion.toString();
    std::cerr << " using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;

    const off_t magLogFileSize = 1000000;
    ibeosdk::LogFileManager logFileManager;
    ibeosdk::LogFile::setTargetFileSize(magLogFileSize);
    const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Release");
    ibeosdk::LogFile::setLogLevel(ll);
    logFileManager.start();

    AllScalaListener allScalaListener;
    ibeosdk::IbeoScala scala(device_ip, port, ibeosdk::IbeoTypeEthTcp()); //B2
    //ibeosdk::IbeoScala scala(device_ip, port, ibeosdk::IbeoTypeEthUdp()); //B3
    scala.setLogFileManager(&logFileManager);
    scala.registerListener(&allScalaListener);

    scala.getConnected();
    ros::Rate loop_rate(25);     //25Hz
    while(ros::ok())
    {
        if(!scala.isConnected())
        {
            std::cout << "Failed!\n";
            ROS_INFO("!scala.isConnected\n");
            return -1;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}




