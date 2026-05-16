// astar_planner.cpp
// 3D A* on OctoMap with pre-computed inflated occupancy grid.
// Inflation is done once before search — O(1) collision lookup per node.

#include <uav_planner_interface/global_planner_base.hpp>
#include <octomap/octomap.h>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <chrono>

namespace uav_planning {

struct AStarNode {
  int idx;       // flat index into grid
  float g, f;
  bool operator>(const AStarNode& o) const { return f > o.f; }
};

class AStarPlanner : public GlobalPlannerBase
{
public:
  void initialize(rclcpp::Node::SharedPtr node, const PlannerConfig& cfg) override
  {
    node_   = node;
    cfg_    = cfg;
    logger_ = node->get_logger();
    RCLCPP_INFO(logger_, "[AStarPlanner] init — clearance: %.2fm",
                cfg.robot_radius + cfg.inflation_radius);
  }

  std::string name() const override { return "AStarPlanner"; }

  nav_msgs::msg::Path computePath(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal,
      const octomap::OcTree& octree,
      const FeedbackCb& feedback_cb) override
  {
    nav_msgs::msg::Path empty;
    empty.header = start.header;

    double res = octree.getResolution();
    double inflate = cfg_.robot_radius + cfg_.inflation_radius;

    // ── 1. Get map bounds ─────────────────────────────────────────────
    double xmin, ymin, zmin, xmax, ymax, zmax;
    octree.getMetricMin(xmin, ymin, zmin);
    octree.getMetricMax(xmax, ymax, zmax);

    // Pad bounds slightly
    xmin -= res; ymin -= res; zmin -= res;
    xmax += res; ymax += res; zmax += res;

    int nx = static_cast<int>((xmax - xmin) / res) + 1;
    int ny = static_cast<int>((ymax - ymin) / res) + 1;
    int nz = static_cast<int>((zmax - zmin) / res) + 1;

    RCLCPP_INFO(logger_,
        "[A*] Grid: %d×%d×%d  bounds x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]",
        nx, ny, nz, xmin, xmax, ymin, ymax, zmin, zmax);

    auto toIdx = [&](int ix, int iy, int iz) {
      return ix + nx * (iy + ny * iz);
    };
    auto inBounds = [&](int ix, int iy, int iz) {
      return ix>=0 && ix<nx && iy>=0 && iy<ny && iz>=0 && iz<nz;
    };
    auto worldToGrid = [&](double x, double y, double z,
                            int& ix, int& iy, int& iz) {
      ix = static_cast<int>((x - xmin) / res);
      iy = static_cast<int>((y - ymin) / res);
      iz = static_cast<int>((z - zmin) / res);
    };

    // ── 2. Pre-compute inflated occupancy grid ────────────────────────
    // Mark voxels occupied in octree, then inflate by robot radius
    int total = nx * ny * nz;
    std::vector<uint8_t> occ(total, 0);  // 0=free/unknown, 1=occupied, 2=inflated

    // Pass 1: mark raw occupied
    for (auto it = octree.begin_leafs(); it != octree.end_leafs(); ++it) {
      if (octree.isNodeOccupied(*it)) {
        int ix, iy, iz;
        worldToGrid(it.getX(), it.getY(), it.getZ(), ix, iy, iz);
        if (inBounds(ix, iy, iz))
          occ[toIdx(ix, iy, iz)] = 1;
      }
    }

    // Pass 2: inflate — mark sphere of radius inflate around each occupied voxel
    int inf_steps = static_cast<int>(std::ceil(inflate / res));
    std::vector<uint8_t> occ_inflated = occ;
    for (int iz = 0; iz < nz; ++iz)
    for (int iy = 0; iy < ny; ++iy)
    for (int ix = 0; ix < nx; ++ix) {
      if (occ[toIdx(ix, iy, iz)] != 1) continue;
      for (int dz=-inf_steps; dz<=inf_steps; ++dz)
      for (int dy=-inf_steps; dy<=inf_steps; ++dy)
      for (int dx=-inf_steps; dx<=inf_steps; ++dx) {
        float d = std::sqrt(dx*dx+dy*dy+dz*dz)*static_cast<float>(res);
        if (d > inflate) continue;
        int nx2=ix+dx, ny2=iy+dy, nz2=iz+dz;
        if (inBounds(nx2, ny2, nz2))
          occ_inflated[toIdx(nx2, ny2, nz2)] = 2;
      }
    }

    RCLCPP_INFO(logger_, "[A*] Occupancy grid built (%d voxels)", total);

    // ── 3. Snap start/goal to grid ────────────────────────────────────
    int sx, sy, sz, gx, gy, gz;
    worldToGrid(start.pose.position.x, start.pose.position.y,
                start.pose.position.z, sx, sy, sz);
    worldToGrid(goal.pose.position.x,  goal.pose.position.y,
                goal.pose.position.z,  gx, gy, gz);

    // Clamp to bounds
    sx=std::clamp(sx,0,nx-1); sy=std::clamp(sy,0,ny-1); sz=std::clamp(sz,0,nz-1);
    gx=std::clamp(gx,0,nx-1); gy=std::clamp(gy,0,ny-1); gz=std::clamp(gz,0,nz-1);

    int s_idx = toIdx(sx, sy, sz);
    int g_idx = toIdx(gx, gy, gz);

    RCLCPP_INFO(logger_,
        "[A*] Start grid: (%d,%d,%d)  Goal grid: (%d,%d,%d)",
        sx,sy,sz, gx,gy,gz);

    if (occ_inflated[g_idx] > 0) {
      RCLCPP_WARN(logger_, "[A*] Goal is in occupied/inflated space — aborting");
      return empty;
    }

    // ── 4. A* search on flat grid ─────────────────────────────────────
    auto heuristic = [&](int idx) {
      int iz = idx / (nx*ny);
      int iy = (idx % (nx*ny)) / nx;
      int ix = idx % nx;
      float dx = static_cast<float>(ix-gx)*static_cast<float>(res);
      float dy = static_cast<float>(iy-gy)*static_cast<float>(res);
      float dz = static_cast<float>(iz-gz)*static_cast<float>(res);
      return std::sqrt(dx*dx+dy*dy+dz*dz);
    };

    std::priority_queue<AStarNode, std::vector<AStarNode>,
                        std::greater<AStarNode>> open;
    std::vector<float> g_score(total, std::numeric_limits<float>::infinity());
    std::vector<int>   came_from(total, -1);

    g_score[s_idx] = 0.0f;
    open.push({s_idx, 0.0f, heuristic(s_idx)});

    // 26-connected neighbourhood
    const std::array<std::array<int,3>, 26> nbrs = {{
      {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
      {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0},
      {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
      {0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1},
      {1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},
      {-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1}
    }};

    auto t0 = std::chrono::steady_clock::now();
    int  iter = 0;

    while (!open.empty()) {
      auto cur = open.top(); open.pop();
      ++iter;

      if (iter % 5000 == 0) {
        float elapsed = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - t0).count();
        feedback_cb(std::min(elapsed / static_cast<float>(cfg_.max_planning_time), 0.99f));
        if (elapsed > cfg_.max_planning_time) {
          RCLCPP_WARN(logger_, "[A*] Timeout after %d iterations", iter);
          return empty;
        }
      }

      if (cur.idx == g_idx) {
        RCLCPP_INFO(logger_, "[A*] Path found in %d iterations (%.2fs)",
            iter, std::chrono::duration<float>(
                std::chrono::steady_clock::now()-t0).count());
        return reconstructPath(cur.idx, came_from, s_idx,
                               nx, ny, xmin, ymin, zmin, res, start.header);
      }

      int iz = cur.idx / (nx*ny);
      int iy = (cur.idx % (nx*ny)) / nx;
      int ix = cur.idx % nx;

      for (auto& d : nbrs) {
        int nx2=ix+d[0], ny2=iy+d[1], nz2=iz+d[2];
        if (!inBounds(nx2,ny2,nz2)) continue;
        int nb_idx = toIdx(nx2,ny2,nz2);
        if (occ_inflated[nb_idx] > 0) continue;

        float step = static_cast<float>(res) *
            std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
        float tg = g_score[cur.idx] + step;

        if (tg < g_score[nb_idx]) {
          g_score[nb_idx]  = tg;
          came_from[nb_idx] = cur.idx;
          open.push({nb_idx, tg, tg + heuristic(nb_idx)});
        }
      }
    }

    RCLCPP_WARN(logger_, "[A*] No path found after %d iterations", iter);
    return empty;
  }

private:
  nav_msgs::msg::Path reconstructPath(
      int goal_idx,
      const std::vector<int>& came_from,
      int start_idx,
      int nx, int ny,
      double xmin, double ymin, double zmin, double res,
      const std_msgs::msg::Header& header) const
  {
    nav_msgs::msg::Path path;
    path.header = header;

    std::vector<int> indices;
    int cur = goal_idx;
    while (cur != start_idx && cur != -1) {
      indices.push_back(cur);
      cur = came_from[cur];
    }
    indices.push_back(start_idx);
    std::reverse(indices.begin(), indices.end());

    for (int idx : indices) {
      int iz = idx / (nx*ny);
      int iy = (idx % (nx*ny)) / nx;
      int ix = idx % nx;
      geometry_msgs::msg::PoseStamped ps;
      ps.header = header;
      ps.pose.position.x = xmin + (ix + 0.5) * res;
      ps.pose.position.y = ymin + (iy + 0.5) * res;
      ps.pose.position.z = zmin + (iz + 0.5) * res;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    return path;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger          logger_{rclcpp::get_logger("AStarPlanner")};
  PlannerConfig           cfg_;
};

}  // namespace uav_planning

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uav_planning::AStarPlanner, uav_planning::GlobalPlannerBase)