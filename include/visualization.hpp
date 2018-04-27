#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

using namespace std;

visualization_msgs::Marker getMarkers(vector<geometry_msgs::PointStamped> &pointArr, string &frame){

    visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id= frame;
    sphere_list.header.stamp= ros::Time::now();
    sphere_list.ns= "spheres";
    sphere_list.action= visualization_msgs::Marker::ADD;
    sphere_list.pose.orientation.w= 1.0;

    sphere_list.id = 0;

    sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;


    // POINTS markers use x and y scale for width/height respectively
    sphere_list.scale.x = 0.1;
    sphere_list.scale.y = 0.1;
    sphere_list.scale.z = 0.1;

    // Points are red
    sphere_list.color.r = 1.0f;
    sphere_list.color.a = 1.0;

    for (auto&& p : pointArr){
        sphere_list.points.push_back(p.point);
    }

    return sphere_list;

}