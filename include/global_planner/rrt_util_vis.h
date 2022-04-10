#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

int count=0;

void init_line(visualization_msgs::Marker* line_msg, double scale){
    
    line_msg->header.frame_id = "map";
    line_msg->id = count++;
    line_msg->type = visualization_msgs::Marker::LINE_LIST;
    line_msg->action = visualization_msgs::Marker::ADD;
    line_msg->pose.orientation.w = 1.0;
    line_msg->scale.x = scale;
}

void pub_line(visualization_msgs::Marker* line_msg, ros::Publisher* line_pub, 
                double x1, double y1, double x2, double y2){

    line_msg->header.stamp = ros::Time::now();

    geometry_msgs::Point p1,p2;
    std_msgs::ColorRGBA c1 , c2;

    p1.x = x1;
    p1.y = y1;
    p1.z = 0.0;

    p2.x = x2;
    p2.y = y2;
    p2.z = 0.0;

    c1.r = 0.0;  // 1.0=255
    c1.g = 0.0;
    c1.b = 1.0;
    c1.a = 1.0;  // alpha

    c2.r = 0.0;  // 1.0=255
    c2.g = 0.0;
    c2.b = 1.0;
    c2.a = 1.0;  // alpha

    line_msg->points.push_back(p1);
    line_msg->points.push_back(p2);

    line_msg->colors.push_back(c1);
    line_msg->colors.push_back(c2);

    // Publish line_msg
    line_pub->publish(*line_msg);

}

void init_point(visualization_msgs::Marker* point_msg){
    
    point_msg->header.frame_id = "map";
    point_msg->id = count++;
    point_msg->type = visualization_msgs::Marker::SPHERE;
    point_msg->action = visualization_msgs::Marker::ADD;
    point_msg->pose.orientation.w = 1.0;
    point_msg->scale.x = 0.05;
    point_msg->scale.y = 0.05;
    point_msg->scale.z = 0.05;
}

void pub_point(visualization_msgs::Marker* point_msg, ros::Publisher* point_pub, 
                double x, double y){

    point_msg->header.stamp = ros::Time::now();

    geometry_msgs::Point p1;
    
    point_msg->pose.position.x = x;
    point_msg->pose.position.y = y;
    point_msg->pose.position.z = 0.0;

    point_msg->color.a = 1.0;
    point_msg->color.r = 1.0;
    point_msg->color.g = 0.0;
    point_msg->color.b = 0.0;

    // Publish line_msg
    point_pub->publish(*point_msg);

}