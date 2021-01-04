//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
// Created by nobleo on 6-9-18.
// Edit by zhangyuan on 12-21-20.
#include <climits>
#include <fstream>
#include <list>
#include <vector>

#ifndef FULL_COVERAGE_PATH_PLANNER_COMMON_H
#define FULL_COVERAGE_PATH_PLANNER_COMMON_H

/** 
* @struct Point_t
* @brief 位置点结构体，int整形，适用于栅格地图情况
*/
typedef struct
{
  int x; ///> 位置点x坐标
  int y; ///> 位置点y坐标
}
Point_t;

inline std::ostream &operator << (std::ostream &os, Point_t &p)
{
  return os << "(" << p.x << ", " << p.y << ")";
}

/** 
* @struct gridNode_t
* @brief 栅格地图节点（栅格点）结构体
*/
typedef struct
{
  Point_t pos; ///> 栅格点位置
  int cost;    ///> 从起始点到当前栅格点的路径代价
  int he;      ///> 从当前栅格点到目标点的最小启发式路径代价
}
gridNode_t;

inline std::ostream &operator << (std::ostream &os, gridNode_t &g)
{
  return os << "gridNode_t(" << g.cost << ", " << g.he << ", " << g.pos  << ")";
}

/** 
* @struct fPoint_t
* @brief 位置点结构体，float浮点型，适用于所有坐标点
*/
typedef struct
{
  float x; ///> 位置点x坐标
  float y; ///> 位置点y坐标
}
fPoint_t;

inline std::ostream &operator << (std::ostream &os, fPoint_t &p)
{
  return os << "(" << p.x << ", " << p.y << ")";
}

/** 
* @brief 枚举标志变量
* @note
* eNodeOpen:位置开放，eNodeVisited:位置是否被遍历
*/
enum
{
  eNodeOpen = false,
  eNodeVisited = true
};

/**
 * 找到点poi距离goals集合最近的距离
 * @param poi 起始点
 * @param goals 潜在最近下一个点的集合 
 * @return 点poi距离goals集合最近的距离
 */
int distanceToClosestPoint(Point_t poi, std::list<Point_t> const &goals);

/**
 * 计算两点之间的距离，平方
 */
int distanceSquared(const Point_t &p1, const Point_t &p2);

/**
 * 执行从起始点到heuristic_goals中的点之一的A*短路路径查找
 * Perform A* shorted path finding from init to one of the points in heuristic_goals
 * @param grid 2D布尔网格单元，true ==占用/阻塞/障碍 occupied/blocked/obstacle
 * @param init 初始坐标点
 * @param cost 遍历一个自由节点的成本，int
 * @param visited 2D布尔网格单元遍历标志量. true == visited
 * @param open_space A*需要找到通往路径的空间。仅用于启发式和直接搜索
 * @param pathNodes 形成从init到heuristic_goals中最接近点的路径的节点
 * @return true：未找到路径， false：找到路径
 */
bool a_star_to_open_space(std::vector<std::vector<bool>> const &grid, gridNode_t init, int cost,
                          std::vector<std::vector<bool>> &visited, std::list<Point_t> const &open_space,
                          std::list<gridNode_t> &pathNodes);

/**
 * 根据内部表示打印网格
 * @param grid     栅格地图
 * @param visited  遍历情况
 * @param fullPath 全覆盖路径
 */
void printGrid(std::vector<std::vector<bool> > const& grid,
               std::vector<std::vector<bool> > const& visited,
               std::list<Point_t> const& path);

/**
 * 根据内部表示打印网格
 * @param grid     栅格地图
 * @param visited  遍历情况
 * @param fullPath 全覆盖路径
 * @param start    起始点
 * @param end      终止点
 */
void printGrid(std::vector<std::vector<bool> > const& grid,
               std::vector<std::vector<bool> > const& visited,
               std::list<gridNode_t> const& path,
               gridNode_t start,
               gridNode_t end);

/**
 * 打印栅格地图情况
 */
void printGrid(std::vector<std::vector<bool> > const& grid);

/**
 * 将2D布尔网格（栅格地图）转换为特定点的Point_t（点）列表
 * @param grid 2D栅格地图
 * @param value_to_search 与该值匹配的点将被返回
 * @return 具有给定value_to_search的点的列表
 */
std::list<Point_t> map_2_goals(std::vector<std::vector<bool> > const& grid, bool value_to_search);

#endif  // FULL_COVERAGE_PATH_PLANNER_COMMON_H