//
// Created by guolindong on 18-10-10.
//

#include "scala_rviz_display.h"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZ>);
int step = 0;
int objects_number_previous = 0;
int objects_eff_number_counter = 0;

void split(const string& src, const string& delim, vector<string>& dest) {
    string str = src;
    string::size_type start = 0, index;
    string substr;
    index = str.find_first_of(delim, start);    //在str中查找(起始：start) delim的任意字符的第一次出现的位置
    while (index != string::npos) {
        substr = str.substr(start, index - start);
        dest.push_back(substr);
        start = str.find_first_not_of(delim, index);    //在str中查找(起始：index) 第一个不属于delim的字符出现的位置
        if (start == string::npos) return;
        index = str.find_first_of(delim, start);
    }
}


void ScalaRvizDisplay::Init(){
    sub_pointcloud = nh.subscribe("/scala_points", 1, &ScalaRvizDisplay::PointCloudCallback, this);
    sub_dynamic_objects = nh.subscribe("/scala_objects", 1, &ScalaRvizDisplay::ObjectsCallback, this);
    sub_fix = nh.subscribe("/Inertial/gps/fix", 1, &ScalaRvizDisplay::GpsCallback, this);
    sub_heading = nh.subscribe("/Inertial/heading", 1, &ScalaRvizDisplay::HeadingCallback, this);
    sub_velocity = nh.subscribe("/Inertial/gps/vel", 1, &ScalaRvizDisplay::VelocityCallback, this);
    sub_pose = nh.subscribe("/pose", 1, &ScalaRvizDisplay::PoseCallback, this);

    pub_track = nh.advertise<nav_msgs::Path>("track", 1);
    pub_trajectory = nh.advertise<nav_msgs::Path>("trajectory", 1);
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    pub_marker_text = nh.advertise<visualization_msgs::MarkerArray>("objects_text", 1);
    pub_dynamic_objects_pose = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("objects_pose", 1);
    pub_dynamic_objects_box = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("objects_box", 1);
    pub_marker_arrow = nh.advertise<visualization_msgs::MarkerArray>("objects_arrow", 1);
    pub_points = nh.advertise<sensor_msgs::PointCloud2>("points",1);
    pub_global_map = nh.advertise<sensor_msgs::PointCloud2>("global_map",1);

    tf_listener_map = new tf::TransformListener();

    pcl::io::loadPLYFile<pcl::PointXYZ>("/home/guolindong/cyberfly_ws/src/ibeo_scala/tiggo_sjtu_map - Cloud.ply", *global_map);

    //road_file.open("/home/guolindong/cyberfly_ws/src/ibeo_scala/road.txt");

    ifstream infile("/home/guolindong/cyberfly_ws/src/ibeo_scala/road_sjtu.txt", ios::in);
    path.header.frame_id = "map";

    if(infile.good()) {
        ROS_INFO("infile good");
        while(!infile.fail()) {
            vector<string> results;
            string word;
            string delim(" ");
            string textline;
            getline(infile, textline);
//            ROS_INFO("%s",textline.c_str());
            split(textline, delim, results);
//            ROS_INFO("results size %i", (int)results.size());
            if(results.size()!=6)continue;
            else{
                geometry_msgs::PoseStamped pose_temp;
                pose_temp.pose.position.x = atof(results[0].c_str());
                pose_temp.pose.position.y = atof(results[1].c_str());
                pose_temp.pose.position.z = atof(results[2].c_str());
                pose_temp.pose.orientation.x = atof(results[3].c_str());
                pose_temp.pose.orientation.y = atof(results[4].c_str());
                pose_temp.pose.orientation.z = atof(results[5].c_str());
                pose_temp.pose.orientation.w = 1;//atof(results[6].c_str());
                path.poses.push_back(pose_temp);
            }
        }
    }
    infile.close();
    ROS_INFO("infile closed");

}


void ScalaRvizDisplay::PointCloudCallback(const sensor_msgs::PointCloud2& msg){
    step++;
    if(step==100)step=0;
    sensor_msgs::PointCloud2 points(msg);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr points_pcl(new pcl::PointCloud<pcl::PointXYZI>());
//    pcl::fromROSMsg(msg, *points_pcl);
//    pcl::toROSMsg(*points_pcl, points);
    points.header.stamp = ros::Time::now();
    points.header.frame_id = "scala";
    pub_points.publish(points);

    //save point cloud
//    const sensor_msgs::PointCloud2 cloud_out;
//    pcl::PointCloud<pcl::PointXYZI>::Ptr obs(new pcl::PointCloud<pcl::PointXYZI>());
//    pcl::fromROSMsg(points, *obs);
//    tf_listener_map->waitForTransform("/map","/points", ros::Time::now(), ros::Duration(0.03));
//    pcl_ros::transformPointCloud("/map",*obs,*obs, *tf_listener_map);
//    if(step%10==0)*global_map += *obs;
//    printf("map size:%i\n", (int)global_map->size());

//    if(step%2==0){
//        road_file <<current_pose.pose.position.x<<"\t"
//              <<current_pose.pose.position.y<<"\t"
//              <<current_pose.pose.position.z<<"\t"
//              <<current_pose.pose.orientation.x<<"\t"
//              <<current_pose.pose.orientation.y<<"\t"
//              <<current_pose.pose.orientation.z<<"\t"
//              <<current_pose.pose.orientation.w<<std::endl;
//    }

}

void ScalaRvizDisplay::ObjectsCallback(const ibeo_scala::ObjectArray& msg){
    jsk_recognition_msgs::BoundingBoxArray box_array;
    box_array.header.stamp = ros::Time::now();
    box_array.header.frame_id = "/objects";
    jsk_recognition_msgs::BoundingBox box_object;
    box_object.header.frame_id = "/objects";
    box_object.header.stamp = ros::Time::now();

    visualization_msgs::MarkerArray text_array;
    visualization_msgs::Marker text_object;
    text_object.header.frame_id = "/objects";
    text_object.header.stamp = ros::Time::now();
    text_object.ns = "object_state";
    text_object.action = visualization_msgs::Marker::MODIFY;
    text_object.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_object.scale.z = 0.7;
    text_object.color.r = 200;
    text_object.color.g = 200;
    text_object.color.b = 200;
    text_object.color.a = 1;

    visualization_msgs::MarkerArray arrow_array;
    visualization_msgs::Marker arrow_object;
    arrow_object.header.frame_id = "/objects";
    arrow_object.header.stamp = ros::Time::now();
    arrow_object.type = visualization_msgs::Marker::ARROW;
    arrow_object.scale.y = 0.05;
    arrow_object.scale.z = 0.05;
    arrow_object.color.r = 180;
    arrow_object.color.g = 0;
    arrow_object.color.b = 180;
    arrow_object.color.a = 1;

    float vx, vy, vz, velocity;
    float roll, pitch, yaw;
    roll = 0.0;
    pitch = 0.0;
    int objects_number = msg.objects.size();

    for (int i = 0; i < objects_number; i++) {
        ibeo_scala::Object obj = msg.objects[i];
        if(obj.age<6) continue;
        vector<float> orientation; //w,x,y,z
        vx = obj.velocity_absolute.x;
        vy = obj.velocity_absolute.y;
        velocity = sqrt(vx*vx+vy*vy);

        std::string obj_class;
        switch (obj.classification) {
            case 0: obj_class = "UnCls";
                break;
            case 1: obj_class = "UnknS";
                break;
            case 2: obj_class = "UnknB";
                break;
            case 3: obj_class = "Ped";break;
            case 4: obj_class = "Bike";break;
            case 5: obj_class = "Car";break;
            case 6: obj_class = "Truck";break;
            case 7: obj_class = "Overdrivable";break;//Overdrivable
            case 8: obj_class = "Underdrivable";break;//Underdrivable
            case 9: obj_class = "Bicyc";break;
            case 10: obj_class = "Motor";break;
            case 11: obj_class = "internal";break;
            default:obj_class = " ";break;
        }

        std::string dynamic_flag;
        switch (obj.dynamic_flag) {
            case 16: dynamic_flag = "D/M/I";break;
            case 32: dynamic_flag = "D/S/I";break;
            case 48: dynamic_flag = "inter/I";break;
            case 64: dynamic_flag = "PS/I";break;
            case 17: dynamic_flag = "D/M/T";break;
            case 33: dynamic_flag = "D/S/T";break;
            case 49: dynamic_flag = "inter/T";break;
            case 65: dynamic_flag = "PS/T";break;
            default:dynamic_flag = "";break;
        }

        //object boxes
        yaw = obj.object_box_orientation/180.0*Pi;//box orientation fill e.yaw
        orientation = EulerToQuaternion(roll, pitch, yaw);
        SetQuaternion(box_object.pose, orientation);
        SetPosition(box_object.pose, obj.reference_points[0].x, obj.reference_points[0].y, (float)0.0);
//        double obj_width = GetDistance(obj.reference_points[1].x, obj.reference_points[1].y,
//                                       obj.reference_points[2].x, obj.reference_points[2].y);
//        double obj_length = GetDistance(obj.reference_points[2].x, obj.reference_points[2].y,
//                                       obj.reference_points[3].x, obj.reference_points[3].y);
//        box_object.dimensions.x = obj_length;
//        box_object.dimensions.y = obj_width;
        box_object.dimensions.x = obj.object_box_size.x;
        box_object.dimensions.y = obj.object_box_size.y;

        if(!obj.object_box_height)box_object.dimensions.z = 0.01;
        else box_object.dimensions.z = obj.object_box_height;
        box_object.label = obj.classification;
        box_object.value = obj.id;
        if(box_object.dimensions.x>0 &&box_object.dimensions.y>0 ){
            box_array.boxes.push_back(box_object);
        }

        //object state text
        text_object.id = i;
        SetPosition(text_object.pose, obj.reference_points[9].x, obj.reference_points[9].y, (float)1.0);
        std::ostringstream str;
        str<<"Id:"<<obj.id<<std::endl
           <<"Age:"<<obj.age<<std::endl
           <<"DyF:"<<dynamic_flag<<std::endl
           <<"Vel:"<<velocity<<std::endl
           <<"Cls:"<<obj_class;
        text_object.text=str.str();
        if(dynamic_flag.size()>1){
            text_array.markers.push_back(text_object);
        }

        //object arrow
        arrow_object.id = i;
        SetPosition(arrow_object.pose, obj.reference_points[9].x, obj.reference_points[9].y, (float)0.0);
        orientation = EulerToQuaternion(roll, pitch, atan2(vy, vx));
        SetQuaternion(arrow_object.pose, orientation);
        arrow_object.scale.x = velocity;
        if(dynamic_flag.size()>1) {
            arrow_array.markers.push_back(arrow_object);
        }
    }

    pub_dynamic_objects_box.publish(box_array);
    pub_marker_text.publish(text_array);
    pub_marker_arrow.publish(arrow_array);

}

void ScalaRvizDisplay::GpsCallback(const sensor_msgs::NavSatFix& msg){
    double x, y;
    Gps2Meter(msg.latitude, msg.longitude, x, y);
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "pose";
    vector<double> orien;
    orien = EulerToQuaternion(0.0, 0.0, heading);
    SetQuaternion(pose.pose, orien);
    SetPosition(pose.pose, x, y, (double)0.0);
    pub_pose.publish(pose);
    vehicle_x = x;
    vehicle_y = y;
}

void ScalaRvizDisplay::HeadingCallback(const tiggo_msgs::Heading& msg){
    heading = msg.data;
}

void ScalaRvizDisplay::VelocityCallback(const geometry_msgs::TwistWithCovarianceStamped& msg){
    double vx = msg.twist.twist.linear.x;
    double vy = msg.twist.twist.linear.y;
    double vz = msg.twist.twist.linear.z;
    current_velocity = sqrt(vx*vx+vy*vy+vz*vz);
}

void ScalaRvizDisplay::PoseCallback(const geometry_msgs::PoseStamped &msg) {
    q.setX(msg.pose.orientation.x);
    q.setY(msg.pose.orientation.y);
    q.setZ(msg.pose.orientation.z);
    q.setW(msg.pose.orientation.w);
    transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
//    printf("pose_callback_over\n");
    path.header.stamp = ros::Time::now();
    pub_trajectory.publish(path);
}

void ScalaRvizDisplay::Gps2Meter(double lat, double lon, double &out_x, double &out_y)
{
    double GPS_OriginX = 500000.0011;
    double GPS_OriginY = 3434266.6602;
    double GPS_OffsetX = 0.0;
    double GPS_OffsetY = 0.0;
    double Ellipse_L0 = 121.41572714;
    double DisenableCtlNode = 1;

    static const double Ellipse_a = 6378137;
    static const double Ellipse_b = 6356752.3142;
    static const double PI = 3.14159265358;
    static const double Ellipse_n = (Ellipse_a - Ellipse_b) / (Ellipse_a + Ellipse_b);
    static const double Ellipse_e = sqrt(Ellipse_a*Ellipse_a - Ellipse_b*Ellipse_b) / Ellipse_a;
    static const double Ellipse_ee = sqrt(Ellipse_a*Ellipse_a - Ellipse_b*Ellipse_b) / Ellipse_b;
    static const double Ellipse_C0 = (Ellipse_a + Ellipse_b)*(1 + 0.25*pow(Ellipse_n, 2) + 0.015625*pow(Ellipse_n, 4))*0.5;
    static const double Ellipse_C1 = -1.5*Ellipse_n + 0.5625*pow(Ellipse_n, 3) - 0.09375*pow(Ellipse_n, 5);
    static const double Ellipse_C2 = 0.9375*pow(Ellipse_n, 2) - 0.46875*pow(Ellipse_n, 4);
    static const double Ellipse_C3 = -35 / 48 * pow(Ellipse_n, 3) + 0.41015625*pow(Ellipse_n, 5);
    static const double Ellipse_C4 = 0.615234375*pow(Ellipse_n, 4);

    double Ellipse_lat = lat*PI / 180;
    double Ellipse_lon = (lon - Ellipse_L0)*PI / 180;
    double Ellipse_N = Ellipse_a / sqrt(1 - pow(Ellipse_e*sin(Ellipse_lat), 2));
    double Ellipse_t = tan(Ellipse_lat);
    double Ellipse_g = Ellipse_ee*cos(Ellipse_lat);
    double Ellipse_m = cos(Ellipse_lat)*Ellipse_lon;

    double Ellipse_X = Ellipse_C0*(Ellipse_lat + Ellipse_C1*sin(2 * Ellipse_lat) + Ellipse_C2*sin(4 * Ellipse_lat) + Ellipse_C3*sin(6 * Ellipse_lat) + Ellipse_C4*sin(8 * Ellipse_lat));

    double tempy = Ellipse_X + 0.5*Ellipse_N*Ellipse_t*pow(Ellipse_m, 2);
    tempy += 0.041666666666666666666666666667*Ellipse_N*Ellipse_t*(5 - pow(Ellipse_t, 2) + 9 * pow(Ellipse_g, 2) + 4 * pow(Ellipse_g, 4))*pow(Ellipse_m, 4);
    tempy += 0.0013888888888888888888888888889*Ellipse_N*Ellipse_t*(61 - 58 * pow(Ellipse_t, 2) + pow(Ellipse_t, 4) + 270 * pow(Ellipse_g, 2) - 330 * pow(Ellipse_g, 2)*pow(Ellipse_t, 2))*pow(Ellipse_m, 6);
    tempy += 0.0000248015873015873*Ellipse_t*Ellipse_N*pow(Ellipse_m, 8)*(1385 - 3111 * pow(Ellipse_t, 2) + 543 * pow(Ellipse_t, 4) - pow(Ellipse_t, 6));

    double tempx = Ellipse_N*Ellipse_m + Ellipse_N*pow(Ellipse_m, 3)*(1 - pow(Ellipse_t, 2) + pow(Ellipse_g, 2))*0.16666666666666666666666666666666666667;
    tempx += Ellipse_N*(5 - 18 * pow(Ellipse_t, 2) + pow(Ellipse_t, 4) + 14 * pow(Ellipse_g, 2) - 58 * pow(Ellipse_g, 2)*pow(Ellipse_t, 2))*pow(Ellipse_m, 5)*0.008333333333333333333333333333;
    tempx += Ellipse_N*(61 - 479 * pow(Ellipse_t, 2) + 179 * pow(Ellipse_t, 4) - pow(Ellipse_t, 6))*pow(Ellipse_m, 7)*0.000198412698412698 + 500000;

    tempx = tempx * 100 - GPS_OriginX * 100;     //cm
    tempy = tempy * 100 - GPS_OriginY * 100;     //cm

    tempx += GPS_OffsetX * 100;
    tempy += GPS_OffsetY * 100;

    out_x = (tempx) / 100.0;
    out_y = (tempy) / 100.0;
}

//EulerToQuaternion, euler in rad
template<typename T>
vector<T> ScalaRvizDisplay::EulerToQuaternion(T roll, T pitch, T yaw)
{
    vector<T> q;
    double x, y, z, w;
    double a = roll/2.0;
    double b = pitch/2.0;
    double g = yaw/2.0;
    w = cos(a)*cos(b)*cos(g) + sin(a)*sin(b)*sin(g);
    x = sin(a)*cos(b)*cos(g) - cos(a)*sin(b)*sin(g);
    y = cos(a)*sin(b)*cos(g) + sin(a)*cos(b)*sin(g);
    z = cos(a)*cos(b)*sin(g) - sin(a)*sin(b)*cos(g);
    q.push_back(w);
    q.push_back(x);
    q.push_back(y);
    q.push_back(z);
    return q;
}

//QuaternionToEuler, euler in rad
template<typename T>
vector<T> ScalaRvizDisplay::QuaternionToEuler(vector<T> q)
{
    vector<T> e;
    T roll = atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
    T pitch = asin(2*(q[0]*q[2]-q[1]*q[3]));
    T yaw = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
    e.push_back(roll);
    e.push_back(pitch);
    e.push_back(yaw);
    return e;
}

template<typename T>
void ScalaRvizDisplay::SetPosition(geometry_msgs::Pose &pose, T x, T y, T z)
{
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
}

template<typename T>
void ScalaRvizDisplay::SetQuaternion(geometry_msgs::Pose &pose, vector<T> q)
{
    pose.orientation.x = q[1];
    pose.orientation.y = q[2];
    pose.orientation.z = q[3];
    pose.orientation.w = q[0];
}

double ScalaRvizDisplay::GetDistance(double x1, double y1, double x2, double y2)
{
    double dx = (x1 - x2);
    double dy = (y1 - y2);
    return sqrt(dx*dx+dy*dy);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scala_rviz_display");
    ScalaRvizDisplay scala_rviz_dispplay;
    scala_rviz_dispplay.Init();

    ros::MultiThreadedSpinner spinner(8);
    ros::Rate r(20);

    while(ros::ok()){

        //publish global map
        if(true) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_map (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (global_map);//这个参数得是指针，类对象不行
            pass.setFilterFieldName ("x");//设置想在哪个坐标轴上操作
            pass.setFilterLimits (scala_rviz_dispplay.vehicle_x-150, scala_rviz_dispplay.vehicle_x+150);//将x轴的0到1范围内
            pass.setFilterLimitsNegative (false);//保留（true就是删除，false就是保留而删除此区间外的）
            pass.filter (*extracted_map);//输出到结果指针

            pcl::PassThrough<pcl::PointXYZ> pass2;
            pass2.setInputCloud (extracted_map);//这个参数得是指针，类对象不行
            pass2.setFilterFieldName ("y");//设置想在哪个坐标轴上操作
            pass2.setFilterLimits (scala_rviz_dispplay.vehicle_y-150, scala_rviz_dispplay.vehicle_y+150);//将x轴的0到1范围内
            pass2.setFilterLimitsNegative (false);//保留（true就是删除，false就是保留而删除此区间外的）
            pass2.filter (*extracted_map);//输出到结果指针

            pcl::toROSMsg(*extracted_map, scala_rviz_dispplay.map);
            scala_rviz_dispplay.map.header.frame_id = "map";
            scala_rviz_dispplay.map.header.stamp = ros::Time::now();
            scala_rviz_dispplay.pub_global_map.publish(scala_rviz_dispplay.map);
        }

        spinner.spin();
        r.sleep();
    }
    ros::waitForShutdown();

    scala_rviz_dispplay.road_file.close();

//    printf("saving point cloud with %i points...\n", (int)global_map->size());
//    pcl::io::savePLYFile("/home/guolindong/cyberfly_ws/src/ibeo_scala/global_map.ply",*global_map, true);
//    printf("point cloud saved successfully.\n");

    return 0;
}