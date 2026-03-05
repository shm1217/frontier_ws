#include "frontier_explorer/dbscan.hpp"
#include <queue>
#include <cmath>

namespace fe {

double distMeters(const nav_msgs::msg::OccupancyGrid& map, const GridPose& a, const GridPose& b) {
  double dx = (a.x - b.x) * map.info.resolution;
  double dy = (a.y - b.y) * map.info.resolution;
  return std::hypot(dx, dy);
}

static std::vector<int> regionQuery(const nav_msgs::msg::OccupancyGrid& map,
                                    const std::vector<GridPose>& pts,
                                    int idx, double eps_m) {
  std::vector<int> neighbors;
  neighbors.reserve(64);
  for (int j = 0; j < (int)pts.size(); ++j) {
    if (j == idx) continue;
    if (distMeters(map, pts[idx], pts[j]) <= eps_m) neighbors.push_back(j);
  }
  return neighbors;
}

std::vector<int> dbscanCluster(const nav_msgs::msg::OccupancyGrid& map,
                               const std::vector<GridPose>& pts,
                               double eps_m, int min_pts) {
  const int N = (int)pts.size();
  std::vector<int> labels(N, -2); // -2 unvisited, -1 noise, >=0 cluster id
  int cluster_id = 0;

  for (int i = 0; i < N; ++i) {
    if (labels[i] != -2) continue;

    auto neighbors = regionQuery(map, pts, i, eps_m);
    if ((int)neighbors.size() + 1 < min_pts) { labels[i] = -1; continue; }

    labels[i] = cluster_id;
    std::queue<int> q;
    for (int nb : neighbors) q.push(nb);

    while (!q.empty()) {
      int p = q.front(); q.pop();

      if (labels[p] == -1) labels[p] = cluster_id;
      if (labels[p] != -2) continue;

      labels[p] = cluster_id;
      auto nbs2 = regionQuery(map, pts, p, eps_m);
      if ((int)nbs2.size() + 1 >= min_pts) {
        for (int nb2 : nbs2) {
          if (labels[nb2] == -2 || labels[nb2] == -1) q.push(nb2);
        }
      }
    }

    cluster_id++;
  }

  return labels;
}

std::vector<GridPose> computeClusterRepresentatives(const std::vector<GridPose>& pts,
                                                    const std::vector<int>& labels) {
  int max_id = -1;
  for (int l : labels) if (l > max_id) max_id = l;
  if (max_id < 0) return {};

  std::vector<std::vector<int>> clusters(max_id + 1);
  for (int i = 0; i < (int)pts.size(); ++i) {
    if (labels[i] >= 0) clusters[labels[i]].push_back(i);
  }

  std::vector<GridPose> reps;
  reps.reserve(clusters.size());

  for (const auto& idxs : clusters) {
    if (idxs.empty()) continue;

    double sx = 0.0, sy = 0.0;
    for (int id : idxs) { sx += pts[id].x; sy += pts[id].y; }
    double cx = sx / idxs.size();
    double cy = sy / idxs.size();

    int best = idxs[0];
    double best_d2 = 1e18;
    for (int id : idxs) {
      double dx = pts[id].x - cx;
      double dy = pts[id].y - cy;
      double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; best = id; }
    }

    reps.push_back(pts[best]);
  }

  return reps;
}

} // namespace fe
