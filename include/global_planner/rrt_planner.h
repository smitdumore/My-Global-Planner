#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <math.h>
#include <vector>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "rrt_util_math.h"
#include "rrt_util_vis.h"

using std::string;

#ifndef RRT_PLANNER_CPP
#define RRT_PLANNER_CPP

namespace global_planner {

    class GlobalPlanner : public nav_core::BaseGlobalPlanner {
        public:

        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan);
        /***********************************************/
        ros::NodeHandle nh;
        ros::Publisher plan_pub_;
        ros::Publisher tree_pub_;
        //ros::Publisher node_pub_ = nh.advertise<visualization_msgs::Marker>("node", 1);


        private:
        
            std::string name_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            bool initialized_= false;
    };
    
};
#endif

struct tree_node
{
    int parent_id;          //parent index in the nodes vector
    geometry_msgs::Point node_coord;   //own coordinates x and y 
};

class rrt{
    
    public:
        geometry_msgs::Point wx_i;         //world or map
        /************************/
        std::vector<tree_node> tree_nodes;
        std::vector<std::vector<geometry_msgs::Point>> tree_edges;
        /************************/

        ros::NodeHandle nh;
        ros::Publisher node_pub_ = nh.advertise<visualization_msgs::Marker>("node", 1);
        ros::Publisher tree_pub_ = nh.advertise<visualization_msgs::Marker>("tree", 1);

        costmap_2d::Costmap2DROS* costmap;
        bool solution_found;

        //constuctor
        rrt(geometry_msgs::Point init_point, costmap_2d::Costmap2DROS* costmap_ros){
            this->wx_i = init_point;
            this->costmap = costmap_ros;

            tree_node start_node;
            start_node.parent_id = 0;
            start_node.node_coord = init_point;
            add_vertex(start_node);
        }

        void add_vertex(tree_node new_node){
            tree_nodes.push_back(new_node);     //    this keyword??
        }

        void add_edge(geometry_msgs::Point point1, geometry_msgs::Point point2){
            std::vector<geometry_msgs::Point> temp;
            temp.push_back(point1);
            temp.push_back(point2);
            tree_edges.push_back(temp);
        }
        
        ~rrt();
};


tree_node getNearestNeighbor(const geometry_msgs::Point rand_point, const rrt* T){

    //geometry_msgs::Point nearest_neighbor;
    tree_node near_node;
    double current_distance= 1000000;
    double min_dist = 1000000;
    int parent_id;


    for(int i=0 ; i < T->tree_nodes.size() ; i++){

        if(rand_point.x != T->tree_nodes.at(i).node_coord.x && rand_point.y != T->tree_nodes.at(i).node_coord.y){
            current_distance = getDistance(rand_point , T->tree_nodes.at(i).node_coord);
            if(current_distance < min_dist){
                near_node = T->tree_nodes.at(i);
                min_dist = current_distance;
                parent_id = i;
            }
        }
    }
    

    near_node.parent_id = parent_id;
 
    return near_node;                //returns the nearest node already in the tree
}

tree_node extendTree(geometry_msgs::Point rand_point, tree_node near_node , double d,const std::vector<tree_node>* tree_nodes){

    tree_node new_node;
    new_node.node_coord.z = 0;
    int parent_id=0;

    double theta = atan2(rand_point.y - near_node.node_coord.y , rand_point.x - near_node.node_coord.x);
    new_node.node_coord.x = near_node.node_coord.x +  d*cos(theta);
    new_node.node_coord.y = near_node.node_coord.y +  d*sin(theta);

    for(int i=0;i<tree_nodes->size(); i++){
        if(tree_nodes->at(i).node_coord == near_node.node_coord){
            parent_id = i;
        }
    }

    //new_node.parent_id = near_node.parent_id;                          //near node's index no its parent MISTAKE
    new_node.parent_id = parent_id;

    return new_node;
}

rrt generateRRT(geometry_msgs::PoseStamped init_pose , geometry_msgs::PoseStamped final_pose , costmap_2d::Costmap2DROS* costmap_ros,
                double goal_tol , int iter , double d){

    // ALL POSES ARE IN WORLD FRAME
    
    rrt T(init_pose.pose.position , costmap_ros); //constructor called
    //start node position and parent set


    geometry_msgs::Point rand_point;
    tree_node node_new, node_near;
    std::vector<geometry_msgs::Point> edge;
    bool isEdgeFree = false;

    /***********************************************/
    //***************Construct tree****************//
    /***********************************************/
    for(int i=0 ; i < iter ; i++){

        rand_point = getRandomState(T.costmap);

        node_near = getNearestNeighbor(rand_point , &T);

        node_new = extendTree(rand_point , node_near , d , &(T.tree_nodes));

        edge.push_back(node_near.node_coord);
        edge.push_back(node_new.node_coord);

        isEdgeFree = edgeInFreeSpace(edge , T.costmap);
        edge.clear();

        if(isEdgeFree){
            //ROS_INFO("This is the new node, sleeping__");
            ros::Duration(0.05).sleep();
            visualization_msgs::Marker new_marker;
            init_point(&new_marker);
            pub_point(&new_marker,&(T.node_pub_),node_new.node_coord.x,node_new.node_coord.y);
            
            T.add_vertex(node_new);
            T.add_edge(node_near.node_coord, node_new.node_coord);

            visualization_msgs::Marker line_marker;
            init_line(&line_marker, 0.01);
            pub_line(&line_marker , &(T.tree_pub_),node_near.node_coord.x, node_near.node_coord.y , node_new.node_coord.x , node_new.node_coord.y);
            //ROS_INFO("Connected nearest to random ,sleeping___");

        }else{continue;}

        if (getDistance(node_new.node_coord , final_pose.pose.position) < goal_tol){
            ROS_INFO("GOAL FOUND************************************************");
            T.solution_found = true;
            break;
        }
    } 

    return T;
}

bool getPlan(rrt *T, std::vector<geometry_msgs::PoseStamped>* plan,
            const geometry_msgs::PoseStamped &start,const geometry_msgs::PoseStamped &goal){

    //last node pushed in the tree_nodes vector must be the closest to the goal
    // so just trace it to back to its parent until we reach start point
    

    plan->clear();
    geometry_msgs::PoseStamped temp_pose;
    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    
    temp_pose.header.frame_id = "map";
    temp_pose.pose.orientation = goal.pose.orientation;

    //last node's id
    int current_id = T->tree_nodes.size()-1;

    //this is only for visualisation
    tree_node prev_node;
    prev_node.node_coord = T->tree_nodes.at(current_id).node_coord;

    while(current_id != 0){

        temp_pose.pose.position = T->tree_nodes.at(current_id).node_coord;
        plan->push_back(temp_pose);
        current_id = T->tree_nodes.at(current_id).parent_id;

        //setting the oreintation for next iteration
        double dy , dx, yaw;
        dy = prev_node.node_coord.y - T->tree_nodes.at(current_id).node_coord.y;
        dx = prev_node.node_coord.x - T->tree_nodes.at(current_id).node_coord.x;
        yaw = atan2(dy, dx);

        quat_tf.setRPY(0,0,yaw);
        quat_msg = tf2::toMsg(quat_tf);
        temp_pose.pose.orientation = quat_msg;

        visualization_msgs::Marker line_marker;
        init_line(&line_marker, 0.05);
        pub_line(&line_marker , &(T->tree_pub_),T->tree_nodes.at(current_id).node_coord.x, T->tree_nodes.at(current_id).node_coord.y
                 , prev_node.node_coord.x , prev_node.node_coord.y);
        ROS_INFO("publishing the path, sleeping______");
        ros::Duration(0.2).sleep();

        prev_node = T->tree_nodes.at(current_id);
    }

    temp_pose.pose.position = T->tree_nodes.at(0).node_coord;
    temp_pose.pose.orientation = start.pose.orientation;
    plan->push_back(temp_pose);

    //possible causes of code crash
    //1. poses pushed into plan do not have a valid orientation
    //2. pushing start position twice ??
    //3. normalise quaternion ??
    //4.  

    std::reverse(plan->begin() , plan->end());
    
    return true;
}



