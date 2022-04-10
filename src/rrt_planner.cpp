#include <pluginlib/class_list_macros.h>
#include "global_planner/rrt_planner.h"


PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;


namespace global_planner {

    GlobalPlanner::GlobalPlanner (){

    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        ROS_WARN("Global planner constructor");
        initialize(name, costmap_ros);
    }


    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){    

        if(!initialized_){
            ROS_INFO("Initializing RRTGlobalPlanner.");

            name_ = name;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros->getCostmap(); 
            initialized_ = true;
            
        }
    }


    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
        
        if(!initialized_){
            ROS_ERROR("RRT Not intialized");
            initialize(name_, costmap_ros_);
            return false;                         //exits make plan
        }

        double goal_tol = 0.3;      //0.2
        int iter = 10000;
        double d = 0.3;            //0.2

        ROS_ERROR("STARTING..........");
        //ros::Duration(1.0).sleep();
        rrt Tree = generateRRT(start , goal,costmap_ros_, goal_tol , iter , d);

        if(Tree.solution_found){
            ROS_WARN("Returing Path");
            //plan.push_back(start);
            //plan.push_back(goal);
            //ROS_WARN("plan size , %d", plan.size());
            return getPlan(&Tree , &plan , start , goal);
        }

        ROS_ERROR("RRT Failed to find a path");
        return false;
    }


}; //namespace ends