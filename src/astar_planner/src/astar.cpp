#include "astar_planner/astar.hpp"
#include <iostream>
#include <limits>
#include <cmath> 
#include <algorithm> // reverse 사용을 위해 필요

namespace astar_planner
{

// Parameters

const int SAFETY_MARGIN = 5;        // 장애물로부터 5칸 띄우기 (약 0.5m)
const int SMOOTHING_ITERATIONS = 5; // 경로를 5번 문질러서 부드럽게 만들기
const double SMOOTH_WEIGHT = 0.5;   // 스무딩 강도 (0.0 ~ 1.0)

AStar::AStar()
: map_width_(0), map_height_(0)
{
}

AStar::~AStar()
{
}

void AStar::setMap(const std::vector<std::vector<int>>& map)
{
  map_ = map;
  if (!map_.empty()) {
    map_height_ = map_.size();
    map_width_ = map_[0].size();
  }
}

double AStar::calculateHeuristic(const GridCell& a, const GridCell& b) const
{
  // Euclidean distance (유클리드 거리)
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

// [수정됨] Safety Margin 적용: 장애물 주변 'Safety margin'칸 이내면 갈 수 없는 곳으로 판단
bool AStar::isValid(const GridCell& cell) const
{
  // 1. 기본 맵 범위 체크
  if (cell.x < 0 || cell.x >= map_width_ || cell.y < 0 || cell.y >= map_height_) {
    return false;
  }

  // 2. Safety Margin 체크
  for (int dy = -SAFETY_MARGIN; dy <= SAFETY_MARGIN; ++dy) {
    for (int dx = -SAFETY_MARGIN; dx <= SAFETY_MARGIN; ++dx) {
      int nx = cell.x + dx;
      int ny = cell.y + dy;

      // 주변 좌표가 맵 범위 안인지 확인
      if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
        // 주변에 장애물이 하나라도 있으면 false (벽에 너무 가까움)
        if (map_[ny][nx] != 0) { 
          return false; 
        }
      }
    }
  }

  return true; // 안전함
}

std::vector<GridCell> AStar::getNeighbors(const GridCell& cell) const
{
  std::vector<GridCell> neighbors;
  
  // 8-connected grid (상하좌우 + 대각선)
  std::vector<std::pair<int, int>> directions = {
    {0, 1},   // up
    {0, -1},  // down
    {1, 0},   // right
    {-1, 0},  // left
    {1, 1},   // up-right
    {1, -1},  // down-right
    {-1, 1},  // up-left
    {-1, -1}  // down-left
  };
  
  for (const auto& dir : directions) {
    GridCell neighbor = {cell.x + dir.first, cell.y + dir.second};
    // isValid 함수가 이제 마진까지 체크해줍니다.
    if (isValid(neighbor)) {
      neighbors.push_back(neighbor);
    }
  }
  
  return neighbors;
}

std::vector<GridCell> AStar::reconstructPath(
  const std::unordered_map<GridCell, GridCell, GridCellHash>& came_from,
  const GridCell& start,
  const GridCell& goal) const
{
  std::vector<GridCell> path;
  GridCell current = goal;
  
  while (!(current == start)) {
    path.push_back(current);
    auto it = came_from.find(current);
    if (it == came_from.end()) {
      break;
    }
    current = it->second;
  }
  
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  
  return path;
}

// [추가됨] Path Smoothing 함수 (이동 평균 필터)
std::vector<GridCell> AStar::smoothPath(const std::vector<GridCell>& path)
{
  if (path.size() < 3) return path; // 점이 너무 적으면 스무딩 불가

  std::vector<GridCell> smoothed_path = path;
  
  // 설정된 횟수만큼 반복해서 부드럽게 만듦
  for(int iter=0; iter < SMOOTHING_ITERATIONS; iter++) {
      for (size_t i = 1; i < smoothed_path.size() - 1; ++i) {
          // 이전 점(i-1), 현재 점(i), 다음 점(i+1)의 평균으로 이동
          double new_x = (1.0 - SMOOTH_WEIGHT) * smoothed_path[i].x + 
                         SMOOTH_WEIGHT * 0.5 * (smoothed_path[i-1].x + smoothed_path[i+1].x);
          double new_y = (1.0 - SMOOTH_WEIGHT) * smoothed_path[i].y + 
                         SMOOTH_WEIGHT * 0.5 * (smoothed_path[i-1].y + smoothed_path[i+1].y);
          
          smoothed_path[i].x = static_cast<int>(new_x);
          smoothed_path[i].y = static_cast<int>(new_y);
      }
  }
  
  return smoothed_path;
}

std::vector<GridCell> AStar::findPath(const GridCell& start, const GridCell& goal)
{
  std::vector<GridCell> empty_path;
  
  // Check if start and goal are valid
  if (!isValid(start)) {
    std::cerr << "Start position is invalid or occupied (Check Safety Margin)!" << std::endl;
    return empty_path;
  }
  
  if (!isValid(goal)) {
    std::cerr << "Goal position is invalid or occupied!" << std::endl;
    return empty_path;
  }
  
  // Priority queue for open set
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
  
  // Track visited nodes
  std::unordered_map<GridCell, bool, GridCellHash> closed_set;
  
  // Track g_cost for each node
  std::unordered_map<GridCell, double, GridCellHash> g_score;
  
  // Track parent of each node
  std::unordered_map<GridCell, GridCell, GridCellHash> came_from;
  
  // Initialize start node
  Node start_node;
  start_node.cell = start;
  start_node.g_cost = 0.0;
  start_node.h_cost = calculateHeuristic(start, goal);
  start_node.f_cost = start_node.g_cost + start_node.h_cost;
  start_node.parent = start;
  
  open_set.push(start_node);
  g_score[start] = 0.0;
  
  while (!open_set.empty()) {
    // Get node with lowest f_cost
    Node current = open_set.top();
    open_set.pop();
    
    // Check if we reached the goal
    if (current.cell == goal) {
      // [수정됨] 경로 재구성 후 스무딩 적용하여 반환
      std::vector<GridCell> raw_path = reconstructPath(came_from, start, goal);
      return smoothPath(raw_path); 
    }
    
    // Skip if already processed
    if (closed_set[current.cell]) {
      continue;
    }
    
    closed_set[current.cell] = true;
    
    // Check all neighbors
    std::vector<GridCell> neighbors = getNeighbors(current.cell);
    
    for (const auto& neighbor : neighbors) {
      // Skip if already processed
      if (closed_set[neighbor]) {
        continue;
      }
      
      // Calculate tentative g_cost
      double dx = static_cast<double>(neighbor.x - current.cell.x);
      double dy = static_cast<double>(neighbor.y - current.cell.y);
      double movement_cost = std::sqrt(dx * dx + dy * dy);
      double tentative_g = current.g_cost + movement_cost;
      
      // Check if this path is better
      auto it = g_score.find(neighbor);
      if (it == g_score.end() || tentative_g < it->second) {
        // This path is better, record it
        came_from[neighbor] = current.cell;
        g_score[neighbor] = tentative_g;
        
        Node neighbor_node;
        neighbor_node.cell = neighbor;
        neighbor_node.g_cost = tentative_g;
        neighbor_node.h_cost = calculateHeuristic(neighbor, goal);
        neighbor_node.f_cost = neighbor_node.g_cost + neighbor_node.h_cost;
        neighbor_node.parent = current.cell;
        
        open_set.push(neighbor_node);
      }
    }
  }
  
  std::cerr << "No path found!" << std::endl;
  return empty_path;
}

}  // namespace astar_planner