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
#include <map>
#include <queue>
#include <vector>
#include <visualization_msgs/Marker.h>

using std::string;

#ifndef DIJKSTRA_PLANNER_CPP
#define DIJKSTRA_PLANNER_CPP

namespace global_planner {

    class GlobalPlanner : public nav_core::BaseGlobalPlanner {
        public:

        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan);

        struct Cell{
            uint x;
            uint y;
            int dist;               //current dist of cell //so far distance // so far sum of costs

            bool operator<(const Cell &c1) const{                     //this operator overloading helps std::map to arranges cells
                return ((c1.x < x) || ( c1.x == x && c1.y < y));      
            }

            bool operator()(Cell const& c1, Cell const& c2){          //min or max heap?
                return c1.dist < c2.dist;
            } 

        };

        // struct pq_util{
        //     bool operator()(Cell const& c1, Cell const& c2){
        //         return c1.dist > c2.dist;
        //     } 
        // };

        

        ros::NodeHandle nh_;
        int count=0;

        ros::Publisher vis_cells;
        bool isValid(Cell);
        costmap_2d::Costmap2D* costmap_ros_;
        void vis(int ,int, costmap_2d::Costmap2D*, bool);
        int size_x = 0, size_y = 0;

        bool plan_pushed = false;
        
    };
};
#endif

