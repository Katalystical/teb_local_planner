#include <gtest/gtest.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/g2o_types/edge_via_point.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {

class InspectablePlanner : public teb_local_planner::TebOptimalPlanner
{
public:
  using TebOptimalPlanner::TebOptimalPlanner;
  using TebOptimalPlanner::buildGraph;
  using TebOptimalPlanner::clearGraph;
};

class InspectablePlannerROS : public teb_local_planner::TebLocalPlannerROS
{
public:
  using TebLocalPlannerROS::transformGlobalPlan;
  using TebLocalPlannerROS::estimateLocalGoalOrientation;
  using TebLocalPlannerROS::adjustLocalGoalOrientation;
};

TEST(OrderedPlan, LimitsHorizonAtTurnAndKeepsCursorOnNextPass)
{
  InspectablePlannerROS wrapper;
  costmap_2d::Costmap2D costmap(100, 100, 0.1, -3.0, -3.0);
  tf2_ros::Buffer tf;
  tf.setUsingDedicatedThread(true);
  geometry_msgs::TransformStamped identity;
  identity.header.frame_id = "map";
  identity.child_frame_id = "test_frame";
  identity.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf.setTransform(identity, "test", true));
  std::vector<geometry_msgs::PoseStamped> path(6);
  const double points[][2] = {{0.0, 0.0}, {0.1, 0.0}, {0.2, 0.0},
                              {0.2, 0.1}, {0.1, 0.1}, {0.0, 0.1}};
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    path[i].header.frame_id = "map";
    path[i].pose.position.x = points[i][0];
    path[i].pose.position.y = points[i][1];
    path[i].pose.orientation.w = 1.0;
  }
  geometry_msgs::PoseStamped robot = path.front();
  std::vector<geometry_msgs::PoseStamped> local;
  int goal = -1;
  int start = -1;
  ASSERT_TRUE(wrapper.transformGlobalPlan(tf, path, robot, costmap, "map", 1.2,
                                          local, &goal, nullptr, 0, 1.0, &start));
  EXPECT_EQ(0, start);
  EXPECT_EQ(2, goal);
  ASSERT_EQ(3u, local.size());

  robot = path[4];
  ASSERT_TRUE(wrapper.transformGlobalPlan(tf, path, robot, costmap, "map", 1.2,
                                          local, &goal, nullptr, 3, 1.0, &start));
  EXPECT_EQ(4, start);
  EXPECT_EQ(5, goal);
}

TEST(OrderedPlan, AdvancesPastCutCornerWithoutJumpingToNearbyPass)
{
  InspectablePlannerROS wrapper;
  costmap_2d::Costmap2D costmap(100, 100, 0.1, -3.0, -3.0);
  tf2_ros::Buffer tf;
  tf.setUsingDedicatedThread(true);
  geometry_msgs::TransformStamped identity;
  identity.header.frame_id = "map";
  identity.child_frame_id = "test_frame";
  identity.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf.setTransform(identity, "test", true));

  const auto make_path = [](const std::vector<std::pair<double, double>>& points) {
    std::vector<geometry_msgs::PoseStamped> path;
    for (const auto& point : points)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = point.first;
      pose.pose.position.y = point.second;
      pose.pose.orientation.w = 1.0;
      path.push_back(pose);
    }
    return path;
  };
  std::vector<geometry_msgs::PoseStamped> local;
  int start = -1;
  auto path = make_path({{0.089913, 0.043768}, {0.138284, 0.067314},
                         {0.050267, 0.114780}, {-0.037750, 0.162247}});
  geometry_msgs::PoseStamped robot = path.front();
  robot.pose.position.x = -0.004;
  robot.pose.position.y = 0.170;
  ASSERT_TRUE(wrapper.transformGlobalPlan(tf, path, robot, costmap, "map", 1.2,
                                          local, nullptr, nullptr, -1, 0.0, &start));
  EXPECT_EQ(0, start);
  costmap_2d::Costmap2D small_costmap(2, 2, 0.1, -1.0, -1.0);
  ASSERT_TRUE(wrapper.transformGlobalPlan(tf, path, robot, small_costmap, "map", 1.2,
                                          local, nullptr, nullptr, 0, 1.0, &start));
  EXPECT_EQ(2, start);
  EXPECT_FALSE(local.empty());

  path = make_path({{0.0, 0.0}, {0.1, 0.0}, {0.2, 0.0},
                    {0.2, 0.1}, {0.1, 0.1}, {0.0, 0.1}});
  robot.pose.position.x = 0.1;
  robot.pose.position.y = 0.09;
  start = 1;
  for (int i = 0; i < 10; ++i)
  {
    ASSERT_TRUE(wrapper.transformGlobalPlan(tf, path, robot, costmap, "map", 1.2,
                                            local, nullptr, nullptr, start, 1.0, &start));
    EXPECT_EQ(1, start);
  }
}

struct RecordedTightTurn
{
  // Recorded waypoints 895-924, translated to waypoint 899.
  const double points[30][2] = {
    {0.101480, -0.384271}, {0.062941, -0.291996}, {0.033159, -0.197002},
    {0.012663, -0.099125}, {0.000000, 0.000000}, {-0.011423, 0.099345},
    {-0.022879, 0.198686}, {-0.048826, 0.236995}, {-0.104108, 0.153665},
    {-0.159953, 0.070715}, {-0.216501, -0.011761}, {-0.273188, -0.094142},
    {-0.332398, -0.174717}, {-0.371345, -0.265808}, {-0.397285, -0.362385},
    {-0.421210, -0.459475}, {-0.444625, -0.556695}, {-0.472021, -0.652792},
    {-0.504659, -0.747248}, {-0.543647, -0.839334}, {-0.590247, -0.927719},
    {-0.639123, -1.014959}, {-0.687359, -1.102546}, {-0.736462, -1.189604},
    {-0.788846, -1.274785}, {-0.840173, -1.360607}, {-0.900307, -1.440223},
    {-0.965229, -1.516185}, {-1.036160, -1.586674}, {-1.105226, -1.658988}
  };
  std::vector<geometry_msgs::PoseStamped> path;
  costmap_2d::Costmap2D costmap{60, 60, 0.1, -3.240282, -3.050060};
  std::vector<geometry_msgs::Point> footprint;

  RecordedTightTurn()
  {
    for (std::size_t i = 0; i < 30; ++i)
    {
      const std::size_t next = std::min(i + 1, std::size_t(29));
      const std::size_t prev = next == i ? i - 1 : i;
      const double yaw = std::atan2(points[next][1] - points[prev][1],
                                    points[next][0] - points[prev][0]);
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = points[i][0];
      pose.pose.position.y = points[i][1];
      pose.pose.orientation.z = std::sin(yaw / 2.0);
      pose.pose.orientation.w = std::cos(yaw / 2.0);
      path.push_back(pose);
    }

    // Lethal cells near the turn from the recorded 0.1 m local costmap.
    const char* rows[] = {
      "...###.......####", "...##........####", "..###.......#####",
      ".###........#####", "###.........#####", "###.........#####",
      "##..........#####", "#............####", "#............####",
      ".............####", ".............####", ".............####",
      "..............###", "...............##", "................#",
      ".................", "................."
    };
    for (unsigned int row = 0; row < 17; ++row)
      for (unsigned int col = 0; col < 17; ++col)
        if (rows[row][col] == '#')
          costmap.setCost(23 + col, 39 - row, costmap_2d::LETHAL_OBSTACLE);

    const double vertices[][2] = {{-0.27, -0.22}, {-0.27, 0.22},
                                   {0.37, 0.22}, {0.37, -0.22}};
    for (const auto& vertex : vertices)
    {
      geometry_msgs::Point point;
      point.x = vertex[0];
      point.y = vertex[1];
      footprint.push_back(point);
    }
  }
};

TEST(OrderedPlan, RecordedTightTurnHasInfeasibleWaypointAndFeasibleStoppedPose)
{
  RecordedTightTurn turn;
  base_local_planner::CostmapModel model(turn.costmap);
  EXPECT_LT(model.footprintCost(turn.points[7][0], turn.points[7][1], -2.16,
                                turn.footprint, 0.0, 0.0), 0.0);
  EXPECT_GE(model.footprintCost(-0.256382, -0.008460, -2.074,
                                turn.footprint, 0.0, 0.0), 0.0);
}

TEST(OrderedPlan, AdjustsInfeasibleGoalHeading)
{
  RecordedTightTurn turn;
  base_local_planner::CostmapModel model(turn.costmap);
  InspectablePlannerROS wrapper;
  const auto& position = turn.path[7].pose.position;
  double yaw = -2.16;
  ASSERT_TRUE(wrapper.adjustLocalGoalOrientation(model, turn.footprint, position, yaw, 0.0, 0.0));
  EXPECT_LE(std::abs(g2o::normalize_theta(yaw + 2.16)), M_PI / 2.0);
  EXPECT_GE(model.footprintCost(position.x, position.y, yaw, turn.footprint, 0.0, 0.0), 0.0);
  const double feasible_yaw = yaw;
  EXPECT_FALSE(wrapper.adjustLocalGoalOrientation(model, turn.footprint, position, yaw, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(feasible_yaw, yaw);

  for (unsigned int y = 0; y < 60; ++y)
    for (unsigned int x = 0; x < 60; ++x)
      turn.costmap.setCost(x, y, costmap_2d::LETHAL_OBSTACLE);
  yaw = -2.16;
  EXPECT_FALSE(wrapper.adjustLocalGoalOrientation(model, turn.footprint, position, yaw, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(-2.16, yaw);
}

void expectRecordedTightTurnTraversal(bool with_boundary, bool shrink_horizon, bool from_incoming = false)
{
  RecordedTightTurn turn;
  if (!with_boundary)
    for (unsigned int y = 0; y < 60; ++y)
      for (unsigned int x = 0; x < 60; ++x)
        turn.costmap.setCost(x, y, costmap_2d::FREE_SPACE);
  base_local_planner::CostmapModel model(turn.costmap);
  InspectablePlannerROS wrapper;
  tf2_ros::Buffer tf;
  tf.setUsingDedicatedThread(true);
  geometry_msgs::TransformStamped identity;
  identity.header.frame_id = "map";
  identity.child_frame_id = "test_frame";
  identity.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf.setTransform(identity, "test", true));

  teb_local_planner::TebConfig config;
  config.robot.max_vel_x = 0.3;
  config.robot.max_vel_x_backwards = 0.2;
  config.robot.max_vel_theta = 1.0;
  config.robot.acc_lim_theta = 0.8;
  config.trajectory.via_points_ordered = true;
  config.optim.no_inner_iterations = 3;
  config.optim.no_outer_iterations = 2;
  config.optim.weight_viapoint = 5.0;
  config.optim.weight_obstacle = 80.0;
  config.optim.weight_kinematics_forward_drive = 100.0;
  config.obstacles.min_obstacle_dist = 0.25;
  config.hcp.max_number_classes = 2;
  config.hcp.simple_exploration = true;
  config.hcp.enable_multithreading = false;
  teb_local_planner::Point2dContainer footprint;
  footprint.emplace_back(-0.25, -0.2);
  footprint.emplace_back(-0.25, 0.2);
  footprint.emplace_back(0.35, 0.2);
  footprint.emplace_back(0.35, -0.2);
  config.robot_model.reset(new teb_local_planner::PolygonRobotFootprint(footprint));
  teb_local_planner::ObstContainer obstacles;
  if (with_boundary)
    for (unsigned int y = 24; y < 39; ++y)
      for (unsigned int x = 24; x < 39; ++x)
      {
        if (turn.costmap.getCost(x, y) != costmap_2d::LETHAL_OBSTACLE || (x + y) % 2 != 0)
          continue;
        if (turn.costmap.getCost(x - 1, y) != costmap_2d::FREE_SPACE &&
            turn.costmap.getCost(x + 1, y) != costmap_2d::FREE_SPACE &&
            turn.costmap.getCost(x, y - 1) != costmap_2d::FREE_SPACE &&
            turn.costmap.getCost(x, y + 1) != costmap_2d::FREE_SPACE)
          continue;
        double wx, wy;
        turn.costmap.mapToWorld(x, y, wx, wy);
        obstacles.emplace_back(new teb_local_planner::PointObstacle(wx, wy));
      }
  teb_local_planner::ViaPointContainer vias;
  teb_local_planner::HomotopyClassPlanner planner(config, &obstacles, {}, &vias);
  teb_local_planner::PoseSE2 robot(from_incoming ? turn.points[0][0] : -0.256382,
                                  from_incoming ? turn.points[0][1] : -0.008460,
                                  from_incoming ? 1.966 : -2.074);
  geometry_msgs::Twist velocity;
  int cursor = from_incoming ? 0 : 4;
  int last_progress_cycle = 0;
  int planning_failures = 0;
  int infeasible = 0;
  int invalid_commands = 0;
  int consecutive_failures = 0;
  int backup_until = 0;
  bool cursor_timed_out = false;
  bool reached_outgoing_path = false;
  double closest_to_turn = std::numeric_limits<double>::infinity();

  // A simplified 20 Hz replay: sampled boundary and shrink-horizon backup, no ROS converter or oscillation recovery.
  for (int cycle = 0; cycle < 1200 && !reached_outgoing_path; ++cycle)
  {
    closest_to_turn = std::min(closest_to_turn, std::hypot(robot.x() - turn.points[7][0],
                                                          robot.y() - turn.points[7][1]));
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = robot.x();
    pose.pose.position.y = robot.y();
    pose.pose.orientation.z = std::sin(robot.theta() / 2.0);
    pose.pose.orientation.w = std::cos(robot.theta() / 2.0);
    std::vector<geometry_msgs::PoseStamped> local;
    geometry_msgs::TransformStamped transform;
    int goal = 0;
    int next_cursor = cursor;
    ASSERT_TRUE(wrapper.transformGlobalPlan(tf, turn.path, pose, turn.costmap, "map", 1.2,
                                            local, &goal, &transform, cursor, 1.0, &next_cursor));
    ASSERT_GE(next_cursor, cursor);
    if (next_cursor > cursor)
      last_progress_cycle = cycle;
    cursor = next_cursor;
    if (cycle - last_progress_cycle >= 600)
    {
      cursor_timed_out = true;
      break;
    }
    reached_outgoing_path = cursor >= 14 &&
        std::hypot(robot.x() - turn.points[14][0], robot.y() - turn.points[14][1]) < 0.2;
    if (reached_outgoing_path)
      break;
    ASSERT_GE(local.size(), 2u);
    if (shrink_horizon && cycle < backup_until && goal + 1 < static_cast<int>(turn.path.size()))
    {
      int reduction = static_cast<int>(local.size()) / 2;
      if (consecutive_failures > 9)
        reduction /= 2;
      const int new_goal = static_cast<int>(local.size()) - reduction - 1;
      if (reduction > 0 && new_goal > 0 && goal >= reduction)
      {
        goal -= reduction;
        local.resize(new_goal + 1);
      }
    }
    vias.clear();
    std::size_t previous = 0;
    for (std::size_t i = 1; i < local.size(); ++i)
    {
      const auto& a = local[previous].pose.position;
      const auto& b = local[i].pose.position;
      if (std::hypot(b.x - a.x, b.y - a.y) >= 0.25)
      {
        vias.emplace_back(b.x, b.y);
        previous = i;
      }
    }
    double yaw = wrapper.estimateLocalGoalOrientation(turn.path, local.back(), goal, transform);
    if (goal + 1 < static_cast<int>(turn.path.size()))
      wrapper.adjustLocalGoalOrientation(model, turn.footprint, local.back().pose.position, yaw, 0.0, 0.0);
    local.back().pose.orientation.z = std::sin(yaw / 2.0);
    local.back().pose.orientation.w = std::cos(yaw / 2.0);
    local.front() = pose;
    double vx = 0.0, vy = 0.0, omega = 0.0;
    bool failed = true;
    if (!planner.plan(local, &velocity))
    {
      ++planning_failures;
      planner.clearPlanner();
    }
    else if (!planner.isTrajectoryFeasible(&model, turn.footprint, 0.0, 0.0, 5, 1.0))
    {
      ++infeasible;
      planner.clearPlanner();
    }
    else if (!planner.getVelocityCommand(vx, vy, omega, 1))
    {
      ++invalid_commands;
      planner.clearPlanner();
      vx = omega = 0.0;
    }
    else
      failed = false;
    if (failed)
    {
      ++consecutive_failures;
      backup_until = cycle + 200;
    }
    else
      consecutive_failures = 0;
    ASSERT_TRUE(std::isfinite(vx) && std::isfinite(omega));
    velocity.linear.x = std::max(-0.2, std::min(0.3, vx));
    velocity.angular.z = std::max(-1.0, std::min(1.0, omega));
    robot.x() += velocity.linear.x * std::cos(robot.theta()) * 0.05;
    robot.y() += velocity.linear.x * std::sin(robot.theta()) * 0.05;
    robot.theta() = g2o::normalize_theta(robot.theta() + velocity.angular.z * 0.05);
    ASSERT_GE(model.footprintCost(robot.x(), robot.y(), robot.theta(), turn.footprint, 0.0, 0.0), 0.0);
  }
  EXPECT_TRUE(reached_outgoing_path) << "Cursor " << cursor + 895 << ", cursor timeout " << cursor_timed_out
                                    << ", infeasible cycles " << infeasible << ", planning failures "
                                    << planning_failures << ", invalid commands " << invalid_commands
                                    << ", closest approach to turn " << closest_to_turn;
  EXPECT_LT(closest_to_turn, 0.2);
}

TEST(OrderedPlan, TraversesRecordedTightTurnWithoutBoundary)
{
  expectRecordedTightTurnTraversal(false, true);
}

TEST(OrderedPlan, TraversesRecordedTightTurnNearBoundary)
{
  expectRecordedTightTurnTraversal(true, false);
}

TEST(OrderedPlan, TraversesRecordedTightTurnWithShrinkHorizon)
{
  expectRecordedTightTurnTraversal(true, true);
}

TEST(OrderedPlan, ApproachesRecordedTightTurnFromIncomingPath)
{
  expectRecordedTightTurnTraversal(true, true, true);
}

TEST(OrderedViaPoints, DoNotWrapBackToFirstPoseWhenTrajectoryRunsOut)
{
  teb_local_planner::TebConfig config;
  config.trajectory.via_points_ordered = true;
  config.optim.weight_viapoint = 5.0;
  teb_local_planner::ViaPointContainer via_points;
  for (int i = 1; i <= 8; ++i)
    via_points.emplace_back(0.1 * i, 0.0);

  teb_local_planner::ObstContainer obstacles;
  InspectablePlanner planner(config, &obstacles, teb_local_planner::TebVisualizationPtr(), &via_points);
  planner.teb().addPose(teb_local_planner::PoseSE2(0.0, 0.0, 0.0));
  for (int i = 1; i <= 4; ++i)
    planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(0.1 * i, 0.0, 0.0), 0.3);

  ASSERT_TRUE(planner.buildGraph());
  int first_pose_edges = 0;
  int via_edges = 0;
  for (const auto* edge : planner.optimizer()->edges())
  {
    const auto* via_edge = dynamic_cast<const teb_local_planner::EdgeViaPoint*>(edge);
    if (!via_edge)
      continue;
    ++via_edges;
    if (via_edge->vertex(0) == planner.teb().PoseVertex(1))
      ++first_pose_edges;
  }
  EXPECT_EQ(1, first_pose_edges);
  EXPECT_EQ(2, via_edges);
  planner.clearGraph();
}

TEST(OrderedPlan, FollowsNarrowHairpinWithoutReturningToPreviousPass)
{
  teb_local_planner::TebConfig config;
  config.robot.max_vel_x = 0.3;
  config.robot.max_vel_x_backwards = 0.2;
  config.robot.max_vel_theta = 1.0;
  config.robot.acc_lim_theta = 0.8;
  config.trajectory.via_points_ordered = true;
  config.optim.no_inner_iterations = 3;
  config.optim.no_outer_iterations = 2;
  config.optim.weight_viapoint = 5.0;
  config.optim.weight_kinematics_forward_drive = 100.0;
  config.hcp.max_number_classes = 2;
  config.hcp.simple_exploration = true;
  config.hcp.enable_multithreading = false;

  std::vector<geometry_msgs::PoseStamped> path;
  const auto append = [&](double x, double y, double yaw) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path.push_back(pose);
  };
  for (int i = 0; i <= 20; ++i) append(i * 0.1, 0.0, 0.0);
  append(2.06, 0.02, 0.7);
  append(2.08, 0.07, M_PI / 2.0);
  append(2.06, 0.12, 2.4);
  for (int i = 20; i >= 0; --i) append(i * 0.1, 0.14, M_PI);

  teb_local_planner::ObstContainer obstacles;
  teb_local_planner::ViaPointContainer vias;
  teb_local_planner::HomotopyClassPlanner planner(config, &obstacles, {}, &vias);
  InspectablePlannerROS wrapper;
  costmap_2d::Costmap2D costmap(100, 100, 0.1, -3.0, -3.0);
  tf2_ros::Buffer tf;
  tf.setUsingDedicatedThread(true);
  geometry_msgs::TransformStamped identity;
  identity.header.frame_id = "map";
  identity.child_frame_id = "test_frame";
  identity.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf.setTransform(identity, "test", true));
  teb_local_planner::PoseSE2 robot(1.4, 0.0, 0.0);
  geometry_msgs::Twist velocity;
  int cursor = 0;
  double furthest_x = robot.x();
  for (int cycle = 0; cycle < 70 && cursor < 34; ++cycle)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = robot.x();
    pose.pose.position.y = robot.y();
    pose.pose.orientation.z = std::sin(robot.theta() / 2.0);
    pose.pose.orientation.w = std::cos(robot.theta() / 2.0);
    std::vector<geometry_msgs::PoseStamped> local;
    geometry_msgs::TransformStamped transform;
    int goal_idx = 0;
    int next_cursor = cursor;
    ASSERT_TRUE(wrapper.transformGlobalPlan(tf, path, pose, costmap, "map", 1.2,
                                            local, &goal_idx, &transform, cursor, 1.0, &next_cursor));
    ASSERT_GE(next_cursor, cursor);
    cursor = next_cursor;
    ASSERT_GE(local.size(), 2u);
    vias.clear();
    std::size_t previous = 0;
    for (std::size_t i = 1; i < local.size(); ++i)
    {
      const auto& a = local[previous].pose.position;
      const auto& b = local[i].pose.position;
      if (std::hypot(b.x - a.x, b.y - a.y) >= 0.25)
      {
        vias.emplace_back(b.x, b.y);
        previous = i;
      }
    }
    const double goal_yaw = wrapper.estimateLocalGoalOrientation(path, local.back(), goal_idx, transform);
    local.back().pose.orientation.z = std::sin(goal_yaw / 2.0);
    local.back().pose.orientation.w = std::cos(goal_yaw / 2.0);
    local.front() = pose;
    ASSERT_TRUE(planner.plan(local, &velocity));
    double vx, vy, omega;
    ASSERT_TRUE(planner.getVelocityCommand(vx, vy, omega, 1));
    vx = std::max(-0.2, std::min(0.3, vx));
    omega = std::max(-1.0, std::min(1.0, omega));
    velocity.linear.x = vx;
    velocity.angular.z = omega;
    robot.x() += vx * std::cos(robot.theta()) * 0.2;
    robot.y() += vx * std::sin(robot.theta()) * 0.2;
    robot.theta() = g2o::normalize_theta(robot.theta() + omega * 0.2);
    furthest_x = std::max(furthest_x, robot.x());
  }
  EXPECT_GE(furthest_x, 2.0);
  EXPECT_GE(cursor, 34);
}

// Integrates commands at 20 Hz; this does not exercise ROS backup modes or
// costmap feasibility checks, and planner time remains wall-clock time.
void expectRecordedOutlineTransition(bool with_boundary)
{
  // Recorded outline-loop transition, translated to the first connector point.
  const double points[][2] = {
    {-1.135996, -0.397276}, {-1.046083, -0.353508}, {-0.956170, -0.309740},
    {-0.866257, -0.265972}, {-0.776344, -0.222204}, {-0.686431, -0.178436},
    {-0.596517, -0.134668}, {-0.506604, -0.090900}, {-0.416691, -0.047132},
    {-0.326778, -0.003364}, { 0.000000,  0.000000}, { 0.089913,  0.043768},
    { 0.138284,  0.067314}, { 0.050267,  0.114780}, {-0.037750,  0.162247},
    {-0.125766,  0.209713}, {-0.112381,  0.252656}, {-0.020487,  0.292097},
    { 0.071407,  0.331537}, { 0.163300,  0.370977}, { 0.255194,  0.410418},
    { 0.347088,  0.449858}, { 0.438981,  0.489298}, { 0.530875,  0.528739},
    { 0.622769,  0.568179}, { 0.714663,  0.607620}, { 0.778343,  0.659545},
    { 0.744477,  0.753632}, { 0.709818,  0.847433}, { 0.680466,  0.942744},
    { 0.660326,  1.040694}, { 0.643458,  1.139254}, { 0.631872,  1.238386},
    { 0.628182,  1.338318}, { 0.625253,  1.438275}, { 0.623507,  1.538257},
    { 0.618435,  1.638076}
  };
  std::vector<geometry_msgs::PoseStamped> path;
  for (std::size_t i = 0; i < sizeof(points) / sizeof(points[0]); ++i)
  {
    const std::size_t next = std::min(i + 1, sizeof(points) / sizeof(points[0]) - 1);
    const std::size_t prev = next == i ? i - 1 : i;
    const double yaw = std::atan2(points[next][1] - points[prev][1],
                                  points[next][0] - points[prev][0]);
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = points[i][0];
    pose.pose.position.y = points[i][1];
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path.push_back(pose);
  }

  teb_local_planner::TebConfig config;
  config.robot.max_vel_x = 0.3;
  config.robot.max_vel_x_backwards = 0.2;
  config.robot.max_vel_theta = 1.0;
  config.robot.acc_lim_theta = 0.8;
  config.trajectory.via_points_ordered = true;
  config.optim.no_inner_iterations = 3;
  config.optim.no_outer_iterations = 2;
  config.optim.weight_viapoint = 5.0;
  config.optim.weight_obstacle = 80.0;
  config.optim.weight_kinematics_forward_drive = 100.0;
  config.obstacles.min_obstacle_dist = 0.25;
  config.hcp.max_number_classes = 2;
  config.hcp.simple_exploration = true;
  config.hcp.enable_multithreading = false;
  teb_local_planner::Point2dContainer footprint;
  footprint.emplace_back(-0.25, -0.2);
  footprint.emplace_back(-0.25, 0.2);
  footprint.emplace_back(0.35, 0.2);
  footprint.emplace_back(0.35, -0.2);
  config.robot_model.reset(new teb_local_planner::PolygonRobotFootprint(footprint));
  teb_local_planner::Point2dContainer padded_footprint;
  padded_footprint.emplace_back(-0.27, -0.22);
  padded_footprint.emplace_back(-0.27, 0.22);
  padded_footprint.emplace_back(0.37, 0.22);
  padded_footprint.emplace_back(0.37, -0.22);
  teb_local_planner::PolygonRobotFootprint collision_footprint(padded_footprint);

  // Approximate the nearby mapped boundary; these are not converter polygons.
  const double boundary[][2] = {
    {-0.789, -0.848}, {-0.489, -0.698}, {-0.189, -0.548},
    {0.111, -0.398}, {0.411, -0.248}, {0.711, -0.098},
    {0.861, 0.202}, {1.011, 0.302}
  };
  teb_local_planner::ObstContainer obstacles;
  if (with_boundary)
    for (std::size_t i = 1; i < sizeof(boundary) / sizeof(boundary[0]); ++i)
      obstacles.emplace_back(new teb_local_planner::LineObstacle(
          boundary[i-1][0], boundary[i-1][1], boundary[i][0], boundary[i][1]));
  teb_local_planner::ViaPointContainer vias;
  teb_local_planner::HomotopyClassPlanner planner(config, &obstacles, {}, &vias);
  InspectablePlannerROS wrapper;
  costmap_2d::Costmap2D costmap(100, 100, 0.1, -3.0, -3.0);
  tf2_ros::Buffer tf;
  tf.setUsingDedicatedThread(true);
  geometry_msgs::TransformStamped identity;
  identity.header.frame_id = "map";
  identity.child_frame_id = "test_frame";
  identity.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf.setTransform(identity, "test", true));

  teb_local_planner::PoseSE2 robot(points[5][0], points[5][1], 0.453);
  geometry_msgs::Twist velocity;
  int cursor = 0;
  int last_progress_cycle = 0;
  int invalid_commands = 0;
  int planning_failures = 0;
  double closest_to_turn = std::numeric_limits<double>::infinity();
  bool reached_next_pass = false;
  bool progress_timed_out = false;
  for (int cycle = 0; cycle < 1200 && !reached_next_pass; ++cycle)
  {
    for (const auto& obstacle : obstacles)
      ASSERT_GT(collision_footprint.calculateDistance(robot, obstacle.get()), 0.0)
          << "Padded footprint intersects the approximate boundary at cycle " << cycle;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = robot.x();
    pose.pose.position.y = robot.y();
    pose.pose.orientation.z = std::sin(robot.theta() / 2.0);
    pose.pose.orientation.w = std::cos(robot.theta() / 2.0);
    closest_to_turn = std::min(closest_to_turn, std::hypot(robot.x() - points[12][0],
                                                           robot.y() - points[12][1]));
    std::vector<geometry_msgs::PoseStamped> local;
    geometry_msgs::TransformStamped transform;
    int goal_idx = 0;
    int next_cursor = cursor;
    ASSERT_TRUE(wrapper.transformGlobalPlan(tf, path, pose, costmap, "map", 1.2,
                                            local, &goal_idx, &transform, cursor, 1.0, &next_cursor));
    ASSERT_GE(next_cursor, cursor);
    if (next_cursor > cursor)
      last_progress_cycle = cycle;
    cursor = next_cursor;
    if (cycle - last_progress_cycle >= 600)
    {
      progress_timed_out = true;
      break;
    }
    if (cursor > 16)
      ASSERT_LT(closest_to_turn, 0.3) << "The cursor skipped the turn";
    reached_next_pass = cursor >= 24 && std::hypot(robot.x() - points[24][0],
                                                   robot.y() - points[24][1]) < 0.25;
    if (reached_next_pass)
      break;
    ASSERT_GE(local.size(), 2u);
    vias.clear();
    std::size_t previous = 0;
    for (std::size_t i = 1; i < local.size(); ++i)
    {
      const auto& a = local[previous].pose.position;
      const auto& b = local[i].pose.position;
      if (std::hypot(b.x - a.x, b.y - a.y) >= 0.25)
      {
        vias.emplace_back(b.x, b.y);
        previous = i;
      }
    }
    const double goal_yaw = wrapper.estimateLocalGoalOrientation(path, local.back(), goal_idx, transform);
    local.back().pose.orientation.z = std::sin(goal_yaw / 2.0);
    local.back().pose.orientation.w = std::cos(goal_yaw / 2.0);
    local.front() = pose;
    double vx = 0, vy = 0, omega = 0;
    if (!planner.plan(local, &velocity))
    {
      ++planning_failures;
      planner.clearPlanner();
    }
    else if (!planner.getVelocityCommand(vx, vy, omega, 1))
    {
      ++invalid_commands;
      planner.clearPlanner();
      vx = omega = 0;
    }
    ASSERT_TRUE(std::isfinite(vx) && std::isfinite(vy) && std::isfinite(omega));
    vx = std::max(-0.2, std::min(0.3, vx));
    omega = std::max(-1.0, std::min(1.0, omega));
    velocity.linear.x = vx;
    velocity.angular.z = omega;
    robot.x() += vx * std::cos(robot.theta()) * 0.05;
    robot.y() += vx * std::sin(robot.theta()) * 0.05;
    robot.theta() = g2o::normalize_theta(robot.theta() + omega * 0.05);
  }
  EXPECT_TRUE(reached_next_pass) << "Stalled at cursor " << cursor
                                << ", pose (" << robot.x() << ", " << robot.y()
                                << ", " << robot.theta() << "), progress timeout: "
                                << progress_timed_out;
  EXPECT_EQ(0, planning_failures);
  EXPECT_EQ(0, invalid_commands);
}

TEST(OrderedPlan, TraversesRecordedOutlineTransitionWithoutBoundary)
{
  expectRecordedOutlineTransition(false);
}

TEST(OrderedPlan, TraversesRecordedOutlineTransitionNearBoundary)
{
  expectRecordedOutlineTransition(true);
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ordered_plan_test", ros::init_options::AnonymousName);
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
