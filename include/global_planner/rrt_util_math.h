#include <math.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

#define PI 3.141593


double randomDouble(double Min , double Max){

    double rand = (double)( std::rand() / (double)RAND_MAX) ;            //double = double/double IIMMPP static_cast ?
    return Min + rand * (Max - Min);
}//returns double in world frame

double getDistance(const geometry_msgs::Point point1, const geometry_msgs::Point point2){
    
    double distance = sqrt(pow(point2.y - point1.y, 2) + pow(point2.x - point1.x, 2));
    return distance;
}

bool inFreeSpace(const geometry_msgs::Point point, costmap_2d::Costmap2DROS* costmap_ros){

    costmap_2d::Costmap2D* costmap_;
        
    costmap_ = costmap_ros->getCostmap();
    costmap_2d::MapLocation map_loc;

    if(!costmap_->worldToMap(point.x ,point.y, map_loc.x, map_loc.y)){
        ROS_WARN("point is outside map bounds");
        return false;
    }

    if(costmap_->getCost(map_loc.x, map_loc.y) > 230){
        return false;
    }
  
    return true;
}

bool edgeInFreeSpace(const std::vector<geometry_msgs::Point> edge,costmap_2d::Costmap2DROS* costmap_ros){
    
    bool result{true};
    double dist = getDistance(edge[0] , edge[1]);
    double num_parts = dist/0.01;                   //let 0.01 be the resolution
    geometry_msgs::Point edge_pt;

    for(int i=0; i < num_parts ; i++){
        edge_pt.x = edge[0].x + i * (edge[1].x - edge[0].x) / num_parts;
        edge_pt.y = edge[0].y + i * (edge[1].y - edge[0].y) / num_parts;         //understand this logic deeply

        if(!inFreeSpace(edge_pt , costmap_ros)){
            result = false;
            break;
        }
    }

    return result;

}


geometry_msgs::Point getRandomState(costmap_2d::Costmap2DROS* costmap_ros){
    
    geometry_msgs::Point randomState;
    randomState.z = 0;
    costmap_2d::Costmap2D* costmap_;
    costmap_ = costmap_ros->getCostmap();

    bool isPointFree = false;

    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();

    while(!isPointFree){
        randomState.x = randomDouble(origin_x + costmap_->getSizeInMetersX()/4 , origin_x + 3*costmap_->getSizeInMetersX()/4);
        randomState.y = randomDouble(origin_y + costmap_->getSizeInMetersY()/4 , origin_y + 3*costmap_->getSizeInMetersY()/4);
        isPointFree = inFreeSpace(randomState, costmap_ros);
    }

    return randomState;

}



