#include <pluginlib/class_list_macros.h>
#include "global_planner/rrt_planner.h"


PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;


namespace global_planner {

    GlobalPlanner::GlobalPlanner (){

    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
       
        initialize(name, costmap_ros);
    }


    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){    

        if(!initialized_){
            ROS_INFO("Initializing RRTGlobalPlanner.");

            name_ = name;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros->getCostmap(); 

            ros::NodeHandle nh;
            plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
            tree_pub_ = nh.advertise<visualization_msgs::Marker>("tree", 1);
            node_pub_ = nh.advertise<visualization_msgs::Marker>("node", 1);

            initialized_ = true;
            
        }
    }


    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
        
        if(!initialized_){
            ROS_ERROR("RRT Not intialized");
            initialize(name_, costmap_ros_);
            return false;
        }
        int a =0;

        while(a < 10000){
            geometry_msgs::Point rand_state1, rand_state2;

            rand_state1 = getRandomState(costmap_ros_);
            rand_state2 = getRandomState(costmap_ros_);

            visualization_msgs::Marker node_msg1;
            init_point(&node_msg1);
            visualization_msgs::Marker node_msg2;
            init_point(&node_msg2);
            pub_point(&node_msg1 , &node_pub_ , rand_state1.x, rand_state1.y);
            pub_point(&node_msg2 , &node_pub_ , rand_state2.x, rand_state2.y);

            ROS_WARN("created two points");
            ros::Duration(0.1).sleep();

            std::vector<geometry_msgs::Point> edge;
            edge.push_back(rand_state1);
            edge.push_back(rand_state2);

            if (edgeInFreeSpace(edge , costmap_ros_)){
                visualization_msgs::Marker line_msg;
                init_line(&line_msg);
                pub_line(&line_msg, &tree_pub_, rand_state1.x , rand_state1.y, rand_state2.x , rand_state2.y);
            }else{
                ROS_ERROR("edge not free");
            }

            
            
            a++;    
        }
        //rand_state2 = getRandomState(costmap_ros_);
        //pub_point(&node_msg , &node_pub_ , rand_state2.x, rand_state2.y);

        
           
        

    

        //publish second point.

        //add edge


    }


}; //namespace ends