//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
/** 在此处导入在规划器中的你需要的库 */
/** 全局规划器接口*/

#include <fstream>
#include <list>
#include <string>
#include <vector>

// In ros lib
// #include <ros/ros.h>
// #include <pluginlib/class_list_macros.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <costmap_2d/costmap_2d.h>
// #include <nav_core/base_global_planner.h>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/GetMap.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>
// #include <tf/tf.h>

using std::string;

#ifndef FULL_COVERAGE_PATH_PLANNER_FULL_COVERAGE_PATH_PLANNER_H
#define FULL_COVERAGE_PATH_PLANNER_FULL_COVERAGE_PATH_PLANNER_H

#include "common.h"

#ifndef dabs
#define dabs(a)     ((a) >= 0 ? (a):-(a))
#endif
#ifndef dmin
#define dmin(a, b)   ((a) <= (b) ? (a):(b))
#endif
#ifndef dmax
#define dmax(a, b)   ((a) >= (b) ? (a):(b))
#endif
#ifndef clamp
#define clamp(a, lower, upper)    dmax(dmin(a, upper), lower)
#endif

enum
{
  eDirNone = 0,
  eDirRight = 1,
  eDirUp = 2,
  eDirLeft = -1,
  eDirDown = -2,
};

namespace full_coverage_path_planner
{
class FullCoveragePathPlanner
{
public:
    /**
     * @brief  Default constructor for the NavFnROS object
     */
    FullCoveragePathPlanner();
    // FullCoveragePathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    /**
     * @brief  发布路径的可视化
     */
    // void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

    ~FullCoveragePathPlanner(){}

    /**
     * @brief  执行规划的接口
     */
    virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) = 0;
    /**
     * 将内部表示的形式转换为路径点发布的路径
     * @param start 机器人的起始位置
     * @param goalpoints 螺旋算法的过程路径点
     * @param plan  格式化输出的规划点，与实际场景结合，为封装器
     */
    void parsePointlist2Plan(const geometry_msgs::PoseStamped& start, 
                                std::list<Point_t> const& goalpoints,
                                std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief  解析地图和初始点。
     * 给定单个cell的大小(解决膨胀地图和遍历栅格移动问题)，将ROS占据栅格地图(global map)转换为规划内部地图(vector vector bool)表示，并设置出发点（o_k）
     * @param cpp_grid_ ROS占用网格表示。 高于65的单元格被视为已占用。这里已经二值化处理所以不用管预处理。
     * @param grid 算法内的网格表示，二维数组表示的01地图
     * @param tileSize 单元格的大小（以米为单位）。 这可以是机器人的大小。初始默认可以设置为robotRadius = toolRadius
     * @param realStart 机器人的起始位置（以米为单位）——外部给出，来进行预处理
     * @param scaledStart 机器人在网格上的起始位置
     * @return success
     * @note  这里将 外部输入的全局地图 cpp_grid_和 初始点 realStart 处理成 
     *              规划使用的内部地图 grid     和 初始点 scaledStart
     *              并设置tileSize来进行辅助膨胀变换
     */
    bool parseGrid(nav_msgs::OccupancyGrid const& cpp_grid_,
                    std::vector<std::vector<bool>>& grid,
                    float robotRadius,
                    float toolRadius,
                    geometry_msgs::PoseStamped const& realStart,
                    Point_t& scaledStart);

    /**
    * @todo 将ros接口对接的变量剥离，尝试使用common构建输入
    */  
    ros::Publisher plan_pub_;               ///> ros发布规划，剥离暂不使用
    ros::ServiceClient cpp_grid_client_;    ///> ros请求全局地图请求端，剥离暂不使用
    nav_msgs::OccupancyGrid cpp_grid_;      ///> ros暂存全局地图，剥离暂不使用
    geometry_msgs::PoseStamped previous_goal_;  ///> 将内部表示点转换为发布格式的路径点使用，剥离暂不使用

    float robot_radius_;     ///> 机器人半径
    float tool_radius_;      ///> 工具半径，规划半径
    float plan_resolution_;  ///> 地图分辨率
    float tile_size_;        ///> 单元格的大小（以米为单位）。 这可以是机器人的大小
    fPoint_t grid_origin_;   ///> 位置点结构体,适用于所有结构体
    bool initialized_;       ///> 是否初始化的标志量

    struct spiral_cpp_metrics_type
    {
        int visited_counter;       ///> 遍历数量
        int multiple_pass_counter; ///> 重复遍历数量
        int accessible_counter;    ///> 可达点数量
        double total_area_covered; ///> 全部覆盖区域
    };
    spiral_cpp_metrics_type spiral_cpp_metrics_;
};

/**
 * Sort function for sorting Points on distance to a POI
 */
struct ComparatorForPointSort
{
    explicit ComparatorForPointSort(Point_t poi) : _poi(poi){}

    bool operator()(const Point_t& first, const Point_t& second) const{
        return distanceSquared(first, _poi) < distanceSquared(second, _poi);
    }
private:
    Point_t _poi;
};

}

#endif // FULL_COVERAGE_PATH_PLANNER_FULL_COVERAGE_PATH_PLANNER_H