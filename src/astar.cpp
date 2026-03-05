#include "frontier_explorer/astar.hpp"
#include "frontier_explorer/grid_utils.hpp"
#include <queue>
#include <limits>
#include <algorithm>

namespace fe {

std::vector<GridPose> astar(const nav_msgs::msg::OccupancyGrid& map,
                            const GridPose& start,
                            const GridPose& goal,
                            const std::vector<uint8_t>& obsMaskInflated) {
  int W = (int)map.info.width;
  int H = (int)map.info.height;

  auto inside = [&](int x,int y){ return (0<=x && x<W && 0<=y && y<H); };
  if (!inside(start.x,start.y) || !inside(goal.x,goal.y)) return {};
  if (obsMaskInflated[IDX(start.x,start.y,W)] || obsMaskInflated[IDX(goal.x,goal.y,W)]) return {};

  auto h = [&](int x,int y){
    int dx = std::abs(x - goal.x);
    int dy = std::abs(y - goal.y);
    int mn = std::min(dx, dy);
    int mx = std::max(dx, dy);
    return 14*mn + 10*(mx - mn);
  };

  struct Node { int f,g,x,y; };
  struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };

  std::priority_queue<Node, std::vector<Node>, Cmp> pq;
  std::vector<int> came(W*H, -1);
  std::vector<int> gscore(W*H, std::numeric_limits<int>::max());

  int sid = IDX(start.x,start.y,W);
  gscore[sid] = 0;
  pq.push({h(start.x,start.y), 0, start.x, start.y});

  while(!pq.empty()){
    Node cur = pq.top(); pq.pop();
    int id = IDX(cur.x,cur.y,W);
    if (cur.g != gscore[id]) continue;

    if (cur.x == goal.x && cur.y == goal.y) {
      std::vector<GridPose> path;
      int cid = id;
      while (cid != -1) {
        int px = cid % W;
        int py = cid / W;
        path.push_back({px,py});
        cid = came[cid];
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (int k=0;k<8;k++){
      int nx = cur.x + dx8[k];
      int ny = cur.y + dy8[k];
      if (!inside(nx,ny)) continue;

      int nid = IDX(nx,ny,W);
      if (obsMaskInflated[nid]) continue;

      bool diagonal = (dx8[k] != 0 && dy8[k] != 0);
      int step_cost = diagonal ? 14 : 10;

      if (diagonal) {
        int n1 = IDX(cur.x + dx8[k], cur.y, W);
        int n2 = IDX(cur.x, cur.y + dy8[k], W);
        if (obsMaskInflated[n1] || obsMaskInflated[n2]) continue;
      }

      int ng = cur.g + step_cost; // ✅ 너 코드 반영
      if (ng < gscore[nid]) {
        gscore[nid] = ng;
        came[nid] = id;
        pq.push({ng + h(nx,ny), ng, nx, ny});
      }
    }
  }
  return {};
}

std::vector<uint8_t> buildReachableMaskFromStart(const nav_msgs::msg::OccupancyGrid& map,
                                                 const GridPose& start,
                                                 const std::vector<uint8_t>& blockedMask) {
  int W = (int)map.info.width;
  int H = (int)map.info.height;

  std::vector<uint8_t> reachable(W*H, 0);
  if (!inBounds(map, start.x, start.y)) return reachable;
  if (blockedMask[IDX(start.x, start.y, W)]) return reachable;

  std::queue<GridPose> q;
  q.push(start);
  reachable[IDX(start.x, start.y, W)] = 1;

  while(!q.empty()){
    GridPose c = q.front(); q.pop();
    for(int k=0;k<4;k++){
      int nx = c.x + dx4[k];
      int ny = c.y + dy4[k];
      if (!inBounds(map, nx, ny)) continue;
      int id = IDX(nx, ny, W);
      if (reachable[id]) continue;
      if (blockedMask[id]) continue;
      reachable[id] = 1;
      q.push({nx, ny});
    }
  }
  return reachable;
}

void filterFrontiersByReachable(const nav_msgs::msg::OccupancyGrid& map,
                                std::vector<GridPose>& frontiers,
                                const std::vector<uint8_t>& reachable) {
  int W = (int)map.info.width;
  frontiers.erase(
    std::remove_if(frontiers.begin(), frontiers.end(),
      [&](const GridPose& f){
        return !reachable[IDX(f.x, f.y, W)];
      }),
    frontiers.end()
  );
}

bool pathTailTooCloseToRawObstacle(const nav_msgs::msg::OccupancyGrid& map,
                                   const std::vector<GridPose>& p,
                                   const std::vector<uint8_t>& obsMaskRaw,
                                   double path_clearance_m,
                                   int /*obstacle_threshold*/) {
  if (p.empty()) return true;
  int W = (int)map.info.width;

  int clearance_cells = (int)std::ceil(path_clearance_m / map.info.resolution);
  int tail = std::max(6, (int)(p.size() * 0.2));
  int start_i = std::max(0, (int)p.size() - tail);

  for (int i=start_i; i<(int)p.size(); ++i) {
    const auto& pt = p[i];
    for (int dy=-clearance_cells; dy<=clearance_cells; ++dy) {
      for (int dx=-clearance_cells; dx<=clearance_cells; ++dx) {
        int nx = pt.x + dx;
        int ny = pt.y + dy;
        if (!inBounds(map, nx, ny)) continue;
        if (obsMaskRaw[IDX(nx, ny, W)]) return true;
      }
    }
  }
  return false;
}

} // namespace fe
