//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <list>
#include <string>
#include <vector>
#include <fstream>
//in ros
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


using std::string;

#ifndef FULL_COVERAGE_PATH_PLANNER_SPIRAL_STC_H
#define FULL_COVERAGE_PATH_PLANNER_SPIRAL_STC_H

#include "full_coverage_path_planner/full_coverage_path_planner.h"

namespace full_coverage_path_planner{

class SpiralSTC:public full_coverage_path_planner::FullCoveragePathPlanner
{
public:
  /**
   * 找到一条路径，该路径从init一直向内盘旋，直到在网格中看到障碍物为止
   * @param grid 2D布尔网格单元，true ==占用/阻塞/障碍 occupied/blocked/obstacle
   * @param init 初始坐标点
   * @param visited 2D布尔网格单元遍历标志量. true == visited,所有被螺旋遍历过的点
   * @return 螺旋遍历的一系列点集合
   */
  static std::list<gridNode_t> spiral(std::vector<std::vector<bool>> const &grid, std::list<gridNode_t> &init,
                                      std::vector<std::vector<bool>> &visited);

  /**
   * 执行Spiral-STC（生成树覆盖）覆盖路径规划。
   * 本质上，机器人向前移动直到遇到障碍物或被访问的节点，然后向右转（形成螺旋形）
   * 当卡在螺旋的中间时，使用A *再次走出并开始新的螺旋，直到a *找不到通往未发现细胞的路径
   * @param grid 2D布尔网格单元，true ==占用/阻塞/障碍 occupied/blocked/obstacle
   * @param init 初始坐标点
   * @param multiple_pass_counter 记录重复点数量
   * @param visited_counter  记录遍历点数量
   * @return 一系列螺旋点
   */
  static std::list<Point_t> spiral_stc(std::vector<std::vector<bool>> const &grid,
                                        Point_t &init,
                                        int &multiple_pass_counter,
                                        int &visited_counter);

  /**
   * @brief 给定世界目标，计算计划
   * @param start 起始点
   * @param goal  终止点
   * @param plan  规划点集合
   * @return 如果找到有效的规划，则为true，否则为false
   */
  bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);

  /**
   * @brief  FullCoveragePathPlanner对象的初始化函数
   * @param  costmap 指向成本图的ROS包装器的指针，用于规划，这里暂时不需要这个，后续需要可以进行封装传入
   * @note   costmap_2d::Costmap2DROS* costmap_ros
   */
  void initialize();
};

} // namespace full_coverage_path_planner

#endif // FULL_COVERAGE_PATH_PLANNER_SPIRAL_STC_H