#include <pluginlib/class_list_macros.h>
#include "global_planner/dijkstra_planner.h"


PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

// Below arrays detail all 8 possible movements from a cell
// 8 connected
int row[] = { -1, 0, 0, 1 , 1 , -1 , -1 , 1};
int col[] = { 0, -1, 1, 0 , 1 ,  1 , -1 , -1};

//4 connected
// int row[] = { 1, 0, 0, -1};
// int col[] = { 0, 1, -1, 0};

int rowSize = sizeof(row)/sizeof(row[0]);

namespace global_planner {
    

    GlobalPlanner::GlobalPlanner (){

    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
       
        initialize(name, costmap_ros);
    }


    /**
     * @brief - This function initializes the global cost map and other variables
     * @param name 
     * @param costmap_ros 
     */
    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        
        costmap_ros_ = costmap_ros->getCostmap(); 

        // global cost map limits
        size_x = costmap_ros_->getSizeInCellsX(); 
        size_y = costmap_ros_->getSizeInCellsY();

        vis_cells = nh_.advertise<visualization_msgs::Marker>("cell_viz", 5);
           
    }

    /**
     * @brief This funcitons checks whether a cell is inside the gloabl cost map limits
     * @param cell
     * @return true/false
     */
    bool GlobalPlanner::isValid(Cell cell)
    {
        if( cell.x >= size_x || cell.y >= size_y || cell.x < 0 || cell.y < 0){ 
            return false;
        }
        return true;
    }

    /**
     * @brief - creates sphere list visualisations for the planner 
     */
    void GlobalPlanner::vis(int x, int y, costmap_2d::Costmap2D* cost_map , bool path_cell){
        
        double cell_x, cell_y;

        cost_map->mapToWorld(x, y, cell_x, cell_y);    

        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.header.frame_id = "map";
        marker.id = count++;
        marker.header.stamp = ros::Time();

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;   
        marker.scale.z = 0.05;
        marker.color.a = 0.5;           // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        if(path_cell){
            marker.color.r = 0.0;     
        }else{
            marker.color.g = 0.0;
            marker.color.a = 1.0;       //publishing the path
            marker.scale.x = 0.06;
            marker.scale.y = 0.06;   
            marker.scale.z = 0.06;
        }

        marker.pose.position.x = cell_x ;
        marker.pose.position.y = cell_y ;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        //cout << "Sleeping for 0.01 second!" << endl;
        ros::Duration(0.001).sleep();

        vis_cells.publish(marker);
    }
    
    /**
     * @brief Main planner loop. Planner is implemented in this function
     * @param start - robot start pose in world frame
     * @param goal - goal pose in world frame
     * @param plan - Plan is a vector of poses
     * @return true - success in finding the path
     */
    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        if(plan_pushed){
            ROS_WARN("Following old plan");
            return true;
        }
        
        ROS_WARN("DIJKSTRA callback");

        uint    mx_i, my_i, mx_f, my_f;         //costmap cell coordinates (strictly positive) in map
        double  wx_i, wy_i, wx_f, wy_f;         //world coordinates (+ive or -ive)  

        wx_i = start.pose.position.x;       //i stands for initial or start 
        wy_i = start.pose.position.y;
        wx_f = goal.pose.position.x;        //f stands for final or goal
        wy_f = goal.pose.position.y;

        costmap_ros_->worldToMap(wx_i, wy_i, mx_i, my_i);       //conversion of worlds coordinates to map indices 
        costmap_ros_->worldToMap(wx_f, wy_f, mx_f, my_f);

        std::priority_queue< Cell , std::vector<Cell> , pq_util> pq;     //min heap
        
        Cell src;
        src.x = mx_i;
        src.y = my_i;
        src.dist = 0;
        
        pq.push(src);
        
        std::map<Cell, bool> visited;
        visited[src] = true;             
        
        std::map<Cell , Cell> parent;
        parent[src] = src;

        std::map<Cell, int> dist;                               //current updated costs till now
        dist[src] = 0;  
        
        while(!pq.empty()){

            Cell curr_cell = pq.top();                          //current cell being explored
            pq.pop();

            uint x = curr_cell.x;
            uint y = curr_cell.y;

            //curr_cell distance will be already filled, beacuse we are setting it before push into the queue
            
            visited[curr_cell] = true;
            dist[curr_cell] = curr_cell.dist;
            
            //adding neighbours to the queue and updating distances
            for(int i=0 ; i < rowSize ; i++){
                
                uint n_x = x + row[i];
                uint n_y = y + col[i];
                
                Cell neigh;
                neigh.x = n_x;
                neigh.y = n_y;
                //neigh.dist = set later

                int neigh_cost = (int)costmap_ros_->getCost(n_x, n_y); 
                if( neigh_cost > 200){continue;}

                if(isValid(neigh)){
                    if(visited.find(neigh) == visited.end()){                            //i.e new cell found 
                        
                        //assigning all dists as infinity if not already
                        if(dist.find(neigh) == dist.end()){
                            dist[neigh] = 10000;
                            neigh.dist  = 10000;
                        }
 
                        //dist[curr_cell] should exist
                        if(curr_cell.dist + neigh_cost  < dist[neigh] ){
                            
                            vis(n_x,n_y,costmap_ros_, true);
                            neigh.dist  = curr_cell.dist + neigh_cost;
                            dist[neigh] = curr_cell.dist + neigh_cost;
                            visited[neigh] = true;
                            parent[neigh] = curr_cell;

                            pq.push(neigh);
                        }

                    }
                }
            } 

            //if goal found
                if(x == mx_f && y == my_f){
                    ROS_ERROR("solution found");
                    plan_pushed = true;

                    Cell last_cell = curr_cell;
                    std::vector<Cell> my_path;

                    while(true){
                        my_path.push_back(last_cell);
                        if(last_cell.x == src.x && last_cell.y == src.y){ break;}       // operator overload
                        last_cell = parent[last_cell];
                    }

                    std::reverse(my_path.begin() , my_path.end());

                    for(int i=0 ; i < my_path.size(); i++){

                        __uint32_t mx = my_path.at(i).x;
                        __uint32_t my = my_path.at(i).y;
                        double wx, wy;

                        costmap_ros_->mapToWorld(mx, my, wx, wy);

                        geometry_msgs::PoseStamped path_pose = start;                   // copying frameids

                        path_pose.pose.position.x = wx;
                        path_pose.pose.position.y = wy;

                        plan.push_back(path_pose);
                        vis(my_path.at(i).x ,my_path.at(i).y ,costmap_ros_ , false);    //vis path
                    }

                    return true;
                }


            
        } // ends
          
        ROS_ERROR("NOT FOUND, visted everything");
        count = 0;
        
        return false;
        
        
    }


}; //namespace ends
