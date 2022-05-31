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

        // Overloading BaseGlobalPlanner functions

        /**
         * @brief - This function initializes the global cost map and other variables
         * @param name 
         * @param costmap_ros 
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief Main planner loop. Planner is implemented in this function
         * @param start - robot start pose
         * @param goal - goal pose
         * @param plan - Plan is a vector of poses
         * @return true - success in finding the path
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                                        std::vector<geometry_msgs::PoseStamped>& plan);

        // Definition of a cell for grid based planning
        struct Cell{
            // x and y coordinate of cell in map 
            // strictly positive
            uint x;
            uint y;

            // cummulative dist of cell from root
            int dist;

            //this operator overloading helps std::map to arranges cells
            bool operator<(const Cell &c1) const{                  
                return ((c1.x < x) || ( c1.x == x && c1.y < y));      
            }

            // bool operator()(Cell const& c1, Cell const& c2){          
            //     return c1.dist < c2.dist;
            // } 

        };

        // utility function for min heap priority queue
        struct pq_util{
            bool operator()(Cell const& c1, Cell const& c2){             //min heap
                return c1.dist > c2.dist;
            } 
        };

        ros::NodeHandle nh_;
        
        int count=0;
        ros::Publisher vis_cells; 
        costmap_2d::Costmap2D* costmap_ros_;
        int size_x = 0, size_y = 0;
        bool plan_pushed = false;

        /**
         * @brief This funcitons checks whether a cell is inside the gloabl cost map limits
         * @param cell
         * @return true/false
         */
        bool isValid(Cell);
        
        /**
         * @brief - creates sphere list visualisations for the planner 
         */
        void vis(int ,int, costmap_2d::Costmap2D*, bool);
    };
};
#endif

