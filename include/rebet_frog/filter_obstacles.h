// Example of custom SyncActionNode (synchronous action)
// without ports.


#include <iostream>
#include <vector>
#include <algorithm>
#include "nav_msgs/srv/get_map.hpp"
#include <math.h>
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

using PoseStamped = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::Point;

using namespace BT;

void explore_group(int row, int col, std::vector<std::vector<double>>& group, std::vector<std::vector<bool>>& visited, std::vector<std::vector<int>>& grid) {
    int rows = grid.size();
    int columns = grid[0].size();

    if (row < 0 || row >= rows || col < 0 || col >= columns) {
        return;
    }

    if (grid[row][col] != 100 || visited[row][col]) {
        return;
    }

    visited[row][col] = true;
    group.push_back({(double)row, (double)col});

    std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    for (const auto& dir : directions) {
        int dr = dir.first;
        int dc = dir.second;
        explore_group(row + dr, col + dc, group, visited, grid);
    }
}

std::vector<std::vector<std::vector<double>>> find_groups(std::vector<std::vector<int>>& grid) {
    int rows = grid.size();
    int columns = grid[0].size();
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(columns, false));
    std::vector<std::vector<std::vector<double>>> groups;

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < columns; ++col) {
            if (grid[row][col] == 100 && !visited[row][col]) {
                std::vector<std::vector<double>> new_group;
                explore_group(row, col, new_group, visited, grid);
                groups.push_back(new_group);
            }
        }
    }

    // Find the largest connected group
    std::vector<std::vector<double>> map_border;
    size_t max_size = 0;

    for (const auto& group : groups) {
        if (group.size() > max_size) {
            max_size = group.size();
            map_border = group;
        }
    }

    // Remove the largest group from the list
    groups.erase(std::remove(groups.begin(), groups.end(), map_border), groups.end());

    return groups;
}

bool filter_out_errors(std::vector<double> & pt)
{
    
    if(pt[0] > 1.8 || pt[0] < -1.8)
    {
        return false;
    }
    if(pt[1] > 1.8 || pt[1] < -1.8)
    {
        return false;
    }
    return true;

}

class FilterObstacles : public BT::SyncActionNode
{
public:
    static constexpr const char* OBS_POS = "obstacle_positions";
    static constexpr const char* OBS_NUM = "obstacle_number";
    static constexpr const char* MAP_IN = "map_to_filter";
    


    FilterObstacles(const std::string & instance_name,
                          const BT::NodeConfig& conf) :
        BT::SyncActionNode(instance_name, conf)

    {}


    static PortsList providedPorts()
    {
    PortsList child_ports = { 
                InputPort<nav_msgs::msg::OccupancyGrid>(MAP_IN),
                OutputPort<std::vector<Point>>(OBS_POS),
                OutputPort<int>(OBS_NUM),
            };

    return child_ports;
    }

    BT::NodeStatus tick() override
    {
        nav_msgs::msg::OccupancyGrid mission_map;
        auto res = getInput(MAP_IN,mission_map);

        if(!res)
        {
            //No map to filter.
            return BT::NodeStatus::FAILURE;
        }
        
        std::vector<std::vector<int>> map_rows;
        for (size_t x = 0; x < mission_map.data.size();x+=mission_map.info.width)
        {
            map_rows.push_back(std::vector<int>(mission_map.data.begin()+x, mission_map.data.begin()+x+mission_map.info.width));
        }


        //ASSERT len(chunks) == height

        //Crucially the notion of 0,0 are upside down, 
        std::reverse(map_rows.begin(), map_rows.end());
        //So I make it so indexing matches up and down of robot.

        std::stringstream mm;
        for (std::vector<int> & map_row : map_rows)
            {
                
                for (int & cell : map_row)
                {
                    mm << cell << " ";
                }
                mm << "\n";
            }
        

        std::vector<std::vector<std::vector<double>>> obstacles = find_groups(map_rows); //this contains the rows, col of each obstacles

        std::stringstream ob;
        for (std::vector<std::vector<double>> obstacle : obstacles)
        {
            ob << "obstacle: \n";
            for (std::vector<double> points : obstacle)
            {
                ob << "x: " << points[0] << " y: " << points[1] << "\n";
            }
        }

        double origin_x = mission_map.info.origin.position.x;
        double origin_y = mission_map.info.origin.position.y;



        for (std::vector<std::vector<double>> & obstacle : obstacles) 
        {
            for (std::vector<double> & pt : obstacle)
            {
                std::stringstream ss;
                std::stringstream tt;

                ss << "origin x : " << origin_x << " resolution: " << mission_map.info.resolution <<  "pt[1]: " << pt[1] << std::endl;
                tt << "origin y : " << origin_y << " resolution: " << mission_map.info.resolution <<  "pt[0]: " << pt[0] << std::endl;

                pt[1] = origin_x + ( (pt[1]*mission_map.info.resolution) + (mission_map.info.resolution/2.0));
                pt[0] = origin_y + ( (pt[0]*mission_map.info.resolution) + (mission_map.info.resolution/2.0));

            
            }
        }

        std::vector<std::vector<double>> points_to_visit = {};

        for (std::vector<std::vector<double>> & obstacle : obstacles) 
        {
            std::vector<double> all_the_ys;
            std::vector<double> all_the_xs;
            double horizontal_middle;
            double vertical_bottom;
            for (std::vector<double> & pt : obstacle)
            {
                all_the_ys.push_back(pt[0]);
                all_the_xs.push_back(pt[1]);
            }
            //Once again as the pt's are in row,col this is the reverse of x,y so we handle them in reverse.
            horizontal_middle = (*std::max_element(all_the_ys.begin(), all_the_ys.end()) + *std::min_element(all_the_ys.begin(), all_the_ys.end())) / 2.0;
            vertical_bottom = *std::min_element(all_the_xs.begin(), all_the_xs.end());

            points_to_visit.push_back({vertical_bottom, -horizontal_middle + 0.12});
            // The offset to avoid driving into the pose being inside of the obstacle is now in visitobstacle_action
        }

        std::vector<Point> route_poses = {};



        for (std::vector<double> & pt : points_to_visit)
        {
            if(filter_out_errors(pt))
            {
                Point point;
                // pose.header.frame_id = "map";
                point.x = pt[0];
                point.y = pt[1];
                // std::vector<double> q = quaternion_from_euler_again(0, 0, pt[2]);
                // pose.pose.orientation.x = q[0];
                // pose.pose.orientation.y = q[1];
                // pose.pose.orientation.z = q[2];
                // pose.pose.orientation.w = q[3];
                route_poses.push_back(point);
            }
        }






        // std::vector<Point> reordered_visiting = {};

        // double min_dist = 9999999.0;
        // PoseStamped min_point;


        // Odometry odom_obj;

        // getInput(POSE_IN,odom_obj);
        // for (PoseStamped & rt_pose : route_poses)
        // {
        //     double dist_from_origin = euclidean_distance(odom_obj.pose.pose.position.x, odom_obj.pose.pose.position.y, rt_pose.x,rt_pose.y);

        //     if(dist_from_origin < min_dist)
        //     {
        //         min_dist = dist_from_origin;
        //         min_point = rt_pose;
        //     }

        // }
        
        // reordered_visiting.push_back(min_point);
        // route_poses.erase(std::remove(route_poses.begin(), route_poses.end(), min_point), route_poses.end());

        // while(!route_poses.empty())
        // {
        //     Point last_entry = reordered_visiting.back();
        //     // reordered_visiting.push_back(
        //     //     min(route_poses,key=lambda rt_pose: euclidean_distance(last_entry.pose.position.x, last_entry.pose.position.y, rt_pose.pose.position.x,rt_pose.pose.position.y)))
            
            
        //     min_dist = 9999999.0;
        //     Point re_min_point;
        //     for (Point & rt_pose : route_poses)
        //     {
        //         double dist_from_prev = euclidean_distance(last_entry..x, last_entry.y, rt_pose.x,rt_pose.y);

        //         if(dist_from_prev < min_dist)
        //         {
        //             min_dist = dist_from_prev;
        //             re_min_point = rt_pose;
        //         }

        //     }
        //     reordered_visiting.push_back(re_min_point);
        //     route_poses.erase(std::remove(route_poses.begin(), route_poses.end(), re_min_point), route_poses.end());
        // }

        // std::stringstream ss;

        // for (const auto& pose : reordered_visiting) {    
        //     ss << "(x: " << pose.pose.position.x << ", y: " << pose.pose.position.y << ")" << std::endl;
        // }

        // std::cout << ss.str() << std::endl;

        // setOutput(OBS_POS, reordered_visiting);

        // setOutput(OBS_NUM, (int)reordered_visiting.size());

        setOutput(OBS_POS, route_poses);
        setOutput(OBS_NUM, (int)route_poses.size());

        return BT::NodeStatus::SUCCESS;
    }

    private:
        // std::vector<double> quaternion_from_euler_again(double ai, double aj, double ak) 
        // {
        //     ai /= 2.0;
        //     aj /= 2.0;
        //     ak /= 2.0;
        //     double ci = cos(ai);
        //     double si = sin(ai);
        //     double cj = cos(aj);
        //     double sj = sin(aj);
        //     double ck = cos(ak);
        //     double sk = sin(ak);
        //     double cc = ci*ck;
        //     double cs = ci*sk;
        //     double sc = si*ck;
        //     double ss = si*sk;

        //     return {cj*sc - sj*cs, cj*ss + sj*cc, cj*cs - sj*sc, cj*cc + sj*ss};
        // }

        double euclidean_distance(double x1, double y1, double x2, double y2)
        {
            return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
        }


};
