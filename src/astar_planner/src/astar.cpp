#include "astar_planner/astar.hpp"
#include <iostream>
#include <limits>
#include <cmath> 
#include <algorithm> // reverse 사용을 위해 필요

namespace astar_planner
{

// Parameters

const double SAFETY_MARGIN = 4;        // 장애물로부터 5칸 띄우기 (약 0.5m)
const int SMOOTHING_ITERATIONS = 5; // 경로를 5번 문질러서 부드럽게 만들기
const double SMOOTH_WEIGHT = 0.5;   // 스무딩 강도 (0.0 ~ 1.0)
const int samples_per_segment_ = 3; //

// [HARD] 로봇이 물리적으로 충돌하는 거리 (로봇 반지름 약 0.2m -> 3~4칸)
// 이 안으로는 절대 경로가 생기지 않습니다.
const int HARD_MARGIN = 3; 

// [SOFT] 로봇이 피하고 싶은 거리 (여유 공간, 약 0.4m -> 6~7칸)
// 이 영역은 갈 수는 있지만, 페널티를 받습니다. -> 코너를 넓게 돌게 만듦
const int SOFT_MARGIN = 10; 

// 벽 근처를 지나갈 때 부과할 벌점 (이 값이 클수록 벽에서 더 멀리 떨어지려 함)
const double PROXIMITY_PENALTY = 30.0;

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

    // [추가] 맵 크기만큼 cost_map_ 초기화 및 계산
    cost_map_.assign(map_height_, std::vector<double>(map_width_, 0.0));
    buildCostMap();
  }
}

//cost에 따른 MAP 비용 지도 생성
void AStar::buildCostMap()
{
  for (int y = 0; y < map_height_; ++y) {
    for (int x = 0; x < map_width_; ++x) {
      
      // 장애물인 경우 (HARD_MARGIN)
      if (map_[y][x] != 0) {
         cost_map_[y][x] = 9999.0; // 절대 불가
         continue;
      }

      // 장애물이 아닌 경우, 주변에 장애물이 있는지 검사 (SOFT_MARGIN)
      double max_penalty = 0.0;
      
      for (int dy = -SOFT_MARGIN; dy <= SOFT_MARGIN; ++dy) {
        for (int dx = -SOFT_MARGIN; dx <= SOFT_MARGIN; ++dx) {
          int nx = x + dx;
          int ny = y + dy;

          if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
            if (map_[ny][nx] != 0) { // 주변에 장애물이 있다!
              double dist = std::sqrt(dx*dx + dy*dy);
              
              // HARD_MARGIN 안쪽이면 그냥 벽이나 다름없음
              if (dist <= HARD_MARGIN) {
                 max_penalty = std::max(max_penalty, 9999.0);
              } 
              // SOFT_MARGIN 안쪽이면 거리에 따른 페널티
              else if (dist <= SOFT_MARGIN) {
                double penalty = PROXIMITY_PENALTY * (1.0 - (dist / SOFT_MARGIN));
                if (penalty > max_penalty) max_penalty = penalty;
              }
            }
          }
        }
      }
      // 계산된 최대 페널티를 저장
      cost_map_[y][x] = max_penalty;
    }
  }
}

double AStar::calculateHeuristic(const GridCell& a, const GridCell& b) const
{
  // Euclidean distance (유클리드 거리)
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

// 함수 추가 : SOFT MARGIN and HARD MARGIN 
// astar.cpp 내부 (AStar 클래스 멤버 함수로 추가하거나, 내부 헬퍼 함수로 사용)

// 특정 셀이 장애물과 얼마나 가까운지 체크하여 벌점 반환
double AStar::getPenalty(int x, int y) const
{
  // 범위 체크
  if (x < 0 || x >= map_width_ || y < 0 || y >= map_height_) return 9999.0;
  
  // 미리 계산해둔 값 리턴 (O(1))
  return cost_map_[y][x];
}

/*
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
}*/

//11.27 수정 소프트 마진 하드 마진
bool AStar::isValid(const GridCell& cell) const
{
  // 1. 맵 범위 체크
  if (cell.x < 0 || cell.x >= map_width_ || cell.y < 0 || cell.y >= map_height_) {
    return false;
  }

  // 2. Hard Margin (물리적 충돌) 체크
  // 로봇이 실제로 끼이는 거리만 체크합니다.
  for (int dy = -HARD_MARGIN; dy <= HARD_MARGIN; ++dy) {
    for (int dx = -HARD_MARGIN; dx <= HARD_MARGIN; ++dx) {
      int nx = cell.x + dx;
      int ny = cell.y + dy;

      if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
        if (map_[ny][nx] != 0) { 
          return false; // 여기는 절대 못 감 (좁은 문이라도 로봇보다 좁으면 못 감)
        }
      }
    }
  }
  return true; 
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
/*
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
}*/
std::vector<GridCell> AStar::smoothPath(const std::vector<GridCell>& path)
{
  if (path.size() < 4 || samples_per_segment_ <= 1) {
    return path;
  }

  std::vector<GridCell> out;
  out.reserve(path.size() * samples_per_segment_);

  // 인덱스 넘어갈 때 양 끝 점으로 클램프
  auto getP = [&](int idx) {
    if (idx < 0) idx = 0;
    if (idx >= static_cast<int>(path.size())) idx = static_cast<int>(path.size()) - 1;
    return path[static_cast<std::size_t>(idx)];
  };

  // 네 점 단위로 스플라인 세그먼트 생성
  for (int i = 0; i < static_cast<int>(path.size()) - 3; ++i) {
    GridCell p0 = getP(i);
    GridCell p1 = getP(i + 1);
    GridCell p2 = getP(i + 2);
    GridCell p3 = getP(i + 3);

    for (int k = 0; k < samples_per_segment_; ++k) {
      double t  = static_cast<double>(k) / static_cast<double>(samples_per_segment_);
      double t2 = t * t;
      double t3 = t2 * t;

      // cubic B-spline basis
      double b0 = (-t3 + 3.0 * t2 - 3.0 * t + 1.0) / 6.0;
      double b1 = ( 3.0 * t3 - 6.0 * t2 + 4.0) / 6.0;
      double b2 = (-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) / 6.0;
      double b3 = (        t3                 ) / 6.0;

      double x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;
      double y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;

      GridCell gc;
      gc.x = static_cast<int>(std::round(x));
      gc.y = static_cast<int>(std::round(y));

      // 맵 범위 체크
      gc.x = std::max(0, std::min(map_width_  - 1, gc.x));
      gc.y = std::max(0, std::min(map_height_ - 1, gc.y));

      // [수정] 스무딩 된 점이 안전한지 체크
      // ==========================================
      if (!isValid(gc)) {
          // 만약 곡선 점이 장애물(또는 Safety Margin) 영역이라면,
          // 곡선 점을 버리고 안전한 원본 경로(p1, p2)를 따라가게 강제해야 함.
          // 여기서는 간단히 이 점을 건너뜁니다. (너무 위험하면 스무딩 취소)
          continue; 
      }

      // 중복 제거 후 push
      if (out.empty() || !(out.back() == gc)) {
        out.push_back(gc);
      }
    }
  }

  // 원본 마지막 점 보장
  out.push_back(path.back());

  // 추가 중복 제거 (선택)
  std::vector<GridCell> filtered;
  filtered.reserve(out.size());
  GridCell prev = out.front();
  filtered.push_back(prev);

  for (std::size_t i = 1; i < out.size(); ++i) {
    if (!(out[i] == prev)) {
      filtered.push_back(out[i]);
      prev = out[i];
    }
  }

  return filtered;
}



std::vector<GridCell> AStar::findPath(const GridCell& start, const GridCell& goal)
{
  std::vector<GridCell> empty_path;
  
  // Check if start and goal are valid
  /*if (!isValid(start)) {
    std::cerr << "Start position is invalid or occupied (Check Safety Margin)!" << std::endl;
    return empty_path;
  }*/
  
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
      double penalty_cost = getPenalty(neighbor.x, neighbor.y); //수정 페널티 함수 코스트에 추가
      double tentative_g = current.g_cost + movement_cost + penalty_cost; //페널티 코스트에 추가
      
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