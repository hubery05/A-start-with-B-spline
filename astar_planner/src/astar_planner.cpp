#include "astar_planner.h"
#include"cubic_spline.h"
#include "bspline.h"
#include <pluginlib/class_list_macros.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_planner{
    AstarPlanner::AstarPlanner(){}
    
    AstarPlanner::AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }
    
    void AstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            map_size = width * height;
            OGM.resize(map_size);

            for (int i = 0; i < width; i++)
            {
                for (int j = 0; j < height; j++)
                {
                    unsigned int cost = static_cast<int>(costmap_->getCost(j, i));
                    //get_cost << cost << endl;
                    //cout << "i:, j:" << cost << endl;
                    
                    if (cost == 0)
                        OGM[i * width + j] = true;
                    else
                        OGM[i * width + j] = false;
                    
                }
            }

            frame_id_ = costmap_ros->getGlobalFrameID();
            
            ros::NodeHandle private_nh("~/" + name);
            
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            replan_pub_ = private_nh.advertise<nav_msgs::Path>("re_plan", 1);
            
            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized... doing nothing");
    }
    
    bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
          {
        
        if(!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }
        
        ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, 
                    goal.pose.position.x,goal.pose.position.y);
        //ros::Time time_1 = ros::Time::now();
        double wx = start.pose.position.x;
        double wy = start.pose.position.y;
        unsigned int start_x, start_y;
        costmap_->worldToMap(wx, wy, start_x, start_y);
        int start_index = costmap_->getIndex(start_x, start_y);

        
        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

        unsigned int goal_x, goal_y;
        costmap_->worldToMap(wx, wy, goal_x, goal_y);
        int goal_index = costmap_->getIndex(goal_x, goal_y);
        
        vector<float> gCosts(map_size, infinity);
        vector<int> cameFrom(map_size, -1);
        
        multiset<Node> priority_costs;
        
        gCosts[start_index] = 0;
        
        Node currentNode;
        currentNode.index = start_index;
        currentNode.cost = gCosts[start_index] + 0;
        priority_costs.insert(currentNode);
        
        vector<geometry_msgs::PoseStamped> replan;
        
        plan.clear();
        replan.clear();
        
        while(!priority_costs.empty())
        {
            // Take the element from the top
            currentNode = *priority_costs.begin();
            //Delete the element from the top
            priority_costs.erase(priority_costs.begin());
            if (currentNode.index == goal_index){
                break;
            }
            // Get neighbors
            vector<int> neighborIndexes = get_neighbors(currentNode.index);
            
            for(int i = 0; i < neighborIndexes.size(); i++){
                if(cameFrom[neighborIndexes[i]] == -1){
                  gCosts[neighborIndexes[i]] = gCosts[currentNode.index] + getMoveCost(currentNode.index, neighborIndexes[i]);
                  Node nextNode;
                  nextNode.index = neighborIndexes[i];
                  //nextNode.cost = gCosts[neighborIndexes[i]];    //Dijkstra Algorithm
                  nextNode.cost = gCosts[neighborIndexes[i]] + getHeuristic(neighborIndexes[i], goal_index);    //A* Algorithm
                  cameFrom[neighborIndexes[i]] = currentNode.index;
                  priority_costs.insert(nextNode);
                }
            }
        }
        
        if(cameFrom[goal_index] == -1){
            cout << "Goal not reachable, failed making a global path." << endl;
            return false;
        }
        
        if(start_index == goal_index)
            return false;
        //Finding the best path
        vector<int> bestPath;
        currentNode.index = goal_index;
        while(currentNode.index != start_index){
            bestPath.push_back(cameFrom[currentNode.index]);
            currentNode.index = cameFrom[currentNode.index];
        }
        reverse(bestPath.begin(), bestPath.end());
        
        vector<int> Path;
        int path;
        for(int i = 0; i < bestPath.size(); i=i+3){
            path = bestPath[i];
            Path.push_back(path);
        }
        //cout << "/***********/" << "bestPath.size():" << bestPath.size() << "*****" <<"Path.size():" << Path.size() << endl;
        
        vector<double> x_set, y_set;
        for(int i = 0; i < Path.size(); i++){
          unsigned int tmp1, tmp2;
          costmap_->indexToCells(Path[i], tmp1, tmp2);
          double x, y;
          costmap_->mapToWorld(tmp1,tmp2, x, y);
          x_set.push_back(x);
          y_set.push_back(y);
        }
        
        
        /*
        //delete some point, result:overtime
        vector<double> x_del, y_del;
        for(int i = 0; i < x_set.size(); i=i+2){
          double x_, y_;
          x_ = x_set[i];
          y_ = y_set[i];
          x_del.push_back(x_);
          x_del.push_back(y_);
        }
        */
        
        /*
        //cubic_spline, result: a little smooting
        vector<double> r_x, r_y;
        Spline2D csp_obj(x_set, y_set);
        //Spline2D csp_obj(x_del, y_del);
        for(float i=0; i<csp_obj.s.back(); i+=0.1){
          array<float, 2> point = csp_obj.calc_postion(i);
          r_x.push_back(point[0]);
          r_y.push_back(point[1]);
        }
        */
        
        
        //b-spline
        Points points;
        points = B_Spline(x_set,y_set);
        
        ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < points.points_x.size(); i++){

          geometry_msgs::PoseStamped pose;
          pose.header.stamp = plan_time;
          pose.header.frame_id = costmap_ros_->getGlobalFrameID();
          //pose.pose.position.x = x_del[i];
          //pose.pose.position.y = x_del[i];
          
          pose.pose.position.x = points.points_x[i];
          pose.pose.position.y = points.points_y[i];
          pose.pose.position.z = 0.0;

          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;

          replan.push_back(pose);
        }


        //ros::Time plan_time = ros::Time::now();
        for(int i = 0; i < bestPath.size(); i++){
          unsigned int tmp1, tmp2;
          costmap_->indexToCells(bestPath[i], tmp1, tmp2);
          double x, y;
          costmap_->mapToWorld(tmp1,tmp2, x, y);

          geometry_msgs::PoseStamped pose;
          pose.header.stamp = plan_time;
          pose.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose.pose.position.x = x;
          pose.pose.position.y = y;
          pose.pose.position.z = 0.0;

          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;

          plan.push_back(pose);
        }


        plan.push_back(goal);
        replan.push_back(goal);
        
        publishPlan(plan);
        publishRePlan(replan);
        
        //ros::Time time_2 = ros::Time::now();
        //ROS_INFO("Time is %f ms", (time_2 - time_1).toSec() * 1000.0);
        return true;
       
    }
    
    double AstarPlanner::getMoveCost(int firstIndex, int secondIndex)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(firstIndex, tmp1, tmp2);
        int firstXCord = tmp1,firstYCord = tmp2;
        costmap_->indexToCells(secondIndex, tmp1, tmp2);
        int secondXCord = tmp1, secondYCord = tmp2;
        
        int difference = abs(firstXCord - secondXCord) + abs(firstYCord - secondYCord);
        // Error checking
        if(difference != 1 && difference != 2){
            ROS_ERROR("Astar global planner: Error in getMoveCost - difference not valid");
            return 1.0;
        }
        if(difference == 1)
            return 1.0;
        else
            return 1.4;
    }
    
    double AstarPlanner::getHeuristic(int cell_index, int goal_index)
    {
        unsigned int tmp1, tmp2;
        costmap_->indexToCells(cell_index, tmp1, tmp2);
        int startX = tmp1, startY = tmp2;
        costmap_->indexToCells(goal_index, tmp1, tmp2);
        int goalX = tmp1, goalY = tmp2;
        
        return abs(goalY - startY) + abs(goalX - startX);
    }
    
    bool AstarPlanner::isInBounds(int x, int y)
    {
        if( x < 0 || y < 0 || x >= height || y >= width)
            return false;
        return true;
    }
    
    vector<int> AstarPlanner::get_neighbors(int current_cell)
    {   
        vector<int> neighborIndexes;
        
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                unsigned tmp1, tmp2;
                costmap_->indexToCells(current_cell, tmp1, tmp2);
                int nextX = tmp1 + i;
                int nextY = tmp2 + j;
                int nextIndex = costmap_->getIndex(nextX, nextY);
                if(!( i == 0 && j == 0) && isInBounds(nextX, nextY) && OGM[nextIndex])
                {
                    neighborIndexes.push_back(nextIndex);
                }
            }
        }
        return neighborIndexes;
    }
    
    
    void AstarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
    }
    
    void AstarPlanner::publishRePlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_repath;
    gui_repath.poses.resize(path.size());

    gui_repath.header.frame_id = frame_id_;
    gui_repath.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_repath.poses[i] = path[i];
    }

    replan_pub_.publish(gui_repath);
    }
};
// Required for multiset sorting
bool operator <(const Node& x, const Node& y) {
  return x.cost < y.cost;
}
