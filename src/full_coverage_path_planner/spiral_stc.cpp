//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/full_coverage_path_planner.h"
#include "full_coverage_path_planner/spiral_stc.h"

namespace full_coverage_path_planner{
void SpiralSTC::initialize(){
    if(initialized_){
        // 初始化中设置机器人半径，单位m
        robot_radius_ = 0.165f;
        // 初始化中设置机器人tool radius，单位m
        tool_radius_ = 0.165f;
        initialized_ = true;
    }
}

std::list<gridNode_t> SpiralSTC::spiral(std::vector<std::vector<bool>> const& grid, std::list<gridNode_t>& init,
                                        std::vector<std::vector<bool>>& visited)
{
  int dx, dy, dx_prev, x2, y2, i, nRows = grid.size(), nCols = grid[0].size();
  // Spiral filling of the open space
  // Copy incoming list to 'end'
  std::list<gridNode_t> pathNodes(init);
  // Create iterator for gridNode_t list and let it point to the last element of end
  std::list<gridNode_t>::iterator it = --(pathNodes.end());
  if (pathNodes.size() > 1)  // if list is length 1, keep iterator at end
    it--;                    // Let iterator point to second to last element

  gridNode_t prev = *(it);
  bool done = false;
  while (!done)
  {
    if (it != pathNodes.begin())
    {
      // turn ccw
      dx = pathNodes.back().pos.x - prev.pos.x;
      dy = pathNodes.back().pos.y - prev.pos.y;
      dx_prev = dx;
      dx = -dy;
      dy = dx_prev;
    }
    else
    {
      // Initialize spiral direction towards y-axis
      dx = 0;
      dy = 1;
    }
    done = true;

    for (int i = 0; i < 4; ++i)
    {
      x2 = pathNodes.back().pos.x + dx;
      y2 = pathNodes.back().pos.y + dy;
      if (x2 >= 0 && x2 < nCols && y2 >= 0 && y2 < nRows)
      {
        if (grid[y2][x2] == eNodeOpen && visited[y2][x2] == eNodeOpen)
        {
          Point_t new_point = { x2, y2 };
          gridNode_t new_node =
          {
            new_point,  // Point: x,y
            0,          // Cost
            0,          // Heuristic
          };
          prev = pathNodes.back();
          pathNodes.push_back(new_node);
          it = --(pathNodes.end());
          visited[y2][x2] = eNodeVisited;  // Close node
          done = false;
          break;
        }
      }
      // try next direction cw
      dx_prev = dx;
      dx = dy;
      dy = -dx_prev;
    }
  }
  return pathNodes;
}

std::list<Point_t> SpiralSTC::spiral_stc(std::vector<std::vector<bool>> const& grid,
                                          Point_t& init,
                                          int &multiple_pass_counter,
                                          int &visited_counter)
{
  int x, y, nRows = grid.size(), nCols = grid[0].size();
  // Initial node is initially set as visited so it does not count
  multiple_pass_counter = 0;
  visited_counter = 0;

  std::vector<std::vector<bool>> visited;
  visited = grid;  // Copy grid matrix
  x = init.x;
  y = init.y;

  Point_t new_point = { x, y };
  gridNode_t new_node =
  {
    new_point,  // Point: x,y
    0,          // Cost
    0,          // Heuristic
  };
  std::list<gridNode_t> pathNodes;
  std::list<Point_t> fullPath;
  pathNodes.push_back(new_node);
  visited[y][x] = eNodeVisited;

  std::cout << "Grid before walking is: ";
  printGrid(grid, visited, fullPath);

  // 进行第一次螺旋填充
  pathNodes = SpiralSTC::spiral(grid, pathNodes, visited);
  std::list<Point_t> goals = map_2_goals(visited, eNodeOpen);  // 获取全部未覆盖的点的位置
  // 将点添加至fullPath，模拟遍历过程
  std::list<gridNode_t>::iterator it;
  for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
  {
    Point_t newPoint = { it->pos.x, it->pos.y };
    visited_counter++;
    fullPath.push_back(newPoint);
  }
  // 从pathNodes列表中删除所有元素，最后一个元素除外
  pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));

  std::cout <<"Current grid after first spiral is" << std::endl;
  printGrid(grid, visited, fullPath);
  printf("There are %d goals remaining.",goals.size());

  while (goals.size() != 0)
  {
    // 从pathNodes列表中删除所有元素，最后一个元素除外
    // 最后一点是新搜索的起点，A*从此开始扩展路径,此处A*可以修改为JPS
    pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
    visited_counter--;  // 第一点已经在while前算过，这里减去
    // 使用A*规划最近的开放节点的路径，将路径放入全覆盖路径中
    // “目标”本质上是地图，因此我们使用“目标”来确定从潜在路径的末端到最近的自由空间的距离
    bool resign = a_star_to_open_space(grid, pathNodes.back(), 1, visited, goals, pathNodes);
    if (resign){
      printf("A_star_to_open_space未找到路径。");
      break;
    }

    // 更新遍历栅格，计算重复盖数量
    for (it = pathNodes.begin(); it != pathNodes.end(); ++it){
      if (visited[it->pos.y][it->pos.x]){
        multiple_pass_counter++;
      }
      visited[it->pos.y][it->pos.x] = eNodeVisited;
    }
    if (pathNodes.size() > 0){
      multiple_pass_counter--;  // 第一个点已经被计算为遍历，需要减去
    }
    std::cout << "Grid with path marked as visited is:" << std::endl;
    gridNode_t SpiralStart = pathNodes.back();
    printGrid(grid, visited, pathNodes, pathNodes.front(), pathNodes.back());

    // 以现在的点进行螺旋填充
    pathNodes = spiral(grid, pathNodes, visited);
    std::cout << "Visited grid updated after spiral:"<< std::endl;
    printGrid(grid, visited, pathNodes, SpiralStart, pathNodes.back());
    goals = map_2_goals(visited, eNodeOpen);  // 获取全部未覆盖的点的位置

    for(it = pathNodes.begin(); it != pathNodes.end(); ++it)
    {
      Point_t newPoint = { it->pos.x, it->pos.y };
      visited_counter++;
      fullPath.push_back(newPoint);
    }
  }

  return fullPath;
}

bool SpiralSTC::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                         std::vector<geometry_msgs::PoseStamped>& plan)
{
  // 检测初始化—配置robot_r,tool_r
  if (!initialized_)
  {
    std::cout << "此规划器还没有被初始化，请检查使用初始化函数后再尝试。";
    return false;
  }
  else
  {
    std::cout << "初始化完成 Initialized!";
  }

  clock_t begin = clock(); // 衡量计算时间
  Point_t startPoint;

  /**
   * @todo 从服务中获取到地图进行处理，成为规划可用地图,global map -> std::vector<std::vector<bool>> const& grid
   */
  std::vector<std::vector<bool>> grid;
  std::cout << "请求全局地图!!"<< std::endl;
  
  nav_msgs::GetMap grid_req_srv;
  
  if (!cpp_grid_client_.call(grid_req_srv))
  {
    ROS_ERROR("Could not retrieve grid from map_server");
    return false;
  }
  /**
   * @todo 给定单个图块的大小，将ROS占据栅格地图转换为规划内部地图表示，并设置出发点
   */
  if (!parseGrid(grid_req_srv.response.map, grid, robot_radius_ * 2, tool_radius_ * 2, start, startPoint))
  {
    std::cout << "无法解析输入地图！" << std::endl;
    return false;
  }
  //打印初始地图和起始点位置
  std::cout <<"初始地图为:" << std::endl;
  std::list<Point_t> printPath;
  printPath.push_back(startPoint);
  printGrid(grid, grid, printPath);
  // 启用规划，并输出路径
  std::list<Point_t> goalPoints = spiral_stc(grid, startPoint,
  											spiral_cpp_metrics_.multiple_pass_counter,
                                            spiral_cpp_metrics_.visited_counter);
  std::cout <<"初始全覆盖完成!" << std::endl;

  /**
   * @todo 将内部表示的点的形式转换为发布格式的路径点，plan为引用
   *  parsePointlist2Plan(start, goalPoints, plan);
   */

  // 打印相关指标
  spiral_cpp_metrics_.accessible_counter = spiral_cpp_metrics_.visited_counter
                                            - spiral_cpp_metrics_.multiple_pass_counter;
  spiral_cpp_metrics_.total_area_covered = (4.0 * tool_radius_ * tool_radius_) * spiral_cpp_metrics_.accessible_counter;
  std::cout <<"指标数据一览：" << std::endl;
  std::cout <<"全部遍历数量(栅格点)    : " << spiral_cpp_metrics_.visited_counter;
  std::cout <<"全部重复遍历数量(栅格点): " << spiral_cpp_metrics_.multiple_pass_counter;
  std::cout <<"全部可达点数量(栅格点)  : "<< spiral_cpp_metrics_.accessible_counter;
  std::cout <<"全部覆盖区域面积(m^2)  : "<< spiral_cpp_metrics_.total_area_covered;

  /**
   * @todo 发布路径点，将修改的格式后的plan集合发布到blackport中
   *   publishPlan(plan);
   */
  std::cout <<"发布路径规划！" << std::endl;
  std::cout <<"路径规划发布成功！" << std::endl;

  // 记录时间相关指标
  clock_t end = clock();
  double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
  std::cout << "消耗时间(ms): " << elapsed_secs << std::endl;

  return true;
}

}