// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
// Created by nobleo on 27-9-18.
// edit by zhangyuan on 12-30-20.

#include <stdlib.h>
#include <time.h>
#include <vector>
#include <full_coverage_path_planner/common.h>

#ifndef FULL_COVERAGE_PATH_PLANNER_UTIL_H
#define FULL_COVERAGE_PATH_PLANNER_UTIL_H
/**
 * @brief   构建一个x*y的栅格地图
 *
 * @param x 水平方向上的元素数（列）（cols）
 * @param y 垂直方向上的元素数（行）（rows）
 * @param fill 用什么来填充行？
 * @return 容器。 内部向量的大小为x，外部向量的大小为y大小
 */
std::vector<std::vector<bool> > makeTestGrid(int x, int y, bool fill = false);

/**
 * @brief   用少量随机障碍物填充测试网格
 * @param grid 地图
 * @param obstacle_fraction 在0到100之间，标记为障碍占据单元格的百分比是多少
 * @return 标识是否成功
 */
bool randomFillTestGrid(std::vector<std::vector<bool> > &grid, float obstacle_fraction);

/**
 * @brief   重构比较符号
 */
bool operator==(const Point_t &lhs, const Point_t &rhs);

/**
 * @struct  比较点的大小，先比较x后比较y
 * @brief   重构比较符号
 */
struct CompareByPosition
{
  bool operator()(const Point_t &lhs, const Point_t &rhs)
  {
    if (lhs.x != rhs.x)
    {
      return lhs.x < rhs.x;
    }
    return lhs.y < rhs.y;
  }
};

#endif  // FULL_COVERAGE_PATH_PLANNER_UTIL_H