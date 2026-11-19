#include <gtest/gtest.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace {

class InspectablePlannerROS : public teb_local_planner::TebLocalPlannerROS
{
public:
  using TebLocalPlannerROS::adjustLocalGoalOrientation;
  using TebLocalPlannerROS::estimateLocalGoalOrientation;
  using TebLocalPlannerROS::transformGlobalPlan;
};

TEST(RecordedLoopStall, RecordedPoseSelectsIntermediateGoalBehindRobot)
{
  const double points[][2] = {{0.000000, 0.000000}, {-0.093494, -0.035353},
                              {-0.184750, -0.074103}, {-0.220161, -0.159441},
                              {-0.189179, -0.253526}, {-0.106188, -0.288077}};
  std::vector<geometry_msgs::PoseStamped> path;
  for (const auto& point : points)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.orientation.w = 1.0;
    path.push_back(pose);
  }
  geometry_msgs::PoseStamped robot = path.front();
  robot.pose.position.x = 0.070409;
  robot.pose.position.y = -0.128204;
  constexpr double yaw = 0.475182;
  robot.pose.orientation.z = std::sin(yaw / 2.0);
  robot.pose.orientation.w = std::cos(yaw / 2.0);
  costmap_2d::Costmap2D costmap(60, 60, 0.1, -2.879915, -3.053526);
  tf2_ros::Buffer tf;
  tf.setUsingDedicatedThread(true);
  geometry_msgs::TransformStamped identity;
  identity.header.frame_id = "map";
  identity.child_frame_id = "test_frame";
  identity.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf.setTransform(identity, "test", true));
  InspectablePlannerROS wrapper;
  std::vector<geometry_msgs::PoseStamped> local;
  int cursor = -1;
  int goal = -1;
  ASSERT_TRUE(wrapper.transformGlobalPlan(tf, path, robot, costmap, "map", 1.2,
                                          local, &goal, nullptr, 0, 1.0, &cursor));
  ASSERT_EQ(0, cursor);
  ASSERT_EQ(3, goal);
  const auto& target = local.back().pose.position;
  EXPECT_LT((target.x - robot.pose.position.x) * std::cos(yaw) +
            (target.y - robot.pose.position.y) * std::sin(yaw), 0.0);
}

// Documents the unresolved stall; enable when a fix consistently traverses
// the recorded turn without compromising ordered progress or feasibility.
TEST(RecordedLoopStall, DISABLED_ReachesOutgoingPassWithoutColliding)
{
  // Recorded action path 1169-1199, relative to waypoint 1172.
  const double points[][2] = {
    {0.272619, 0.124967}, {0.182819, 0.080968}, {0.091967, 0.039216},
    {0.000000, 0.000000}, {-0.093494, -0.035353}, {-0.184750, -0.074103},
    {-0.220161, -0.159441}, {-0.189179, -0.253526}, {-0.106188, -0.288077},
    {-0.007555, -0.271700}, {0.088621, -0.245536}, {0.182076, -0.209979},
    {0.275498, -0.174323}, {0.369114, -0.139171}, {0.461940, -0.102041},
    {0.552686, -0.060132}, {0.640969, -0.013189}, {0.728002, 0.036057},
    {0.815130, 0.085135}, {0.901134, 0.136158}, {0.986753, 0.187824},
    {1.035819, 0.259119}, {1.006020, 0.354560}, {0.973501, 0.449125},
    {0.956453, 0.547652}, {0.943135, 0.646742}, {0.922856, 0.744565},
    {0.892185, 0.839714}, {0.860265, 0.934482}, {0.829433, 1.029609},
    {0.795409, 1.123604}
  };
  constexpr std::size_t count = sizeof(points) / sizeof(points[0]);
  std::vector<geometry_msgs::PoseStamped> path;
  for (std::size_t i = 0; i < count; ++i)
  {
    const std::size_t next = std::min(i + 1, count - 1);
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

  // Lethal cells reconstructed from the bag's local costmap and updates.
  const char* rows[] = {
    "..###########..........", "..###########..........", "..###########..........",
    "..##########...........", "..##########...........", "..#########............",
    "..######...............", "..####.................", ".####..................",
    ".####..................", "####...................", "####...................",
    "####...................", "####.................##", "#####..............####",
    "#####............######", "######........#########", "########...############",
    "#######################", "#######################", "#######################"
  };
  costmap_2d::Costmap2D costmap(60, 60, 0.1, -2.879915, -3.053526);
  for (unsigned int row = 0; row < 21; ++row)
    for (unsigned int col = 0; col < 23; ++col)
      if (rows[row][col] == '#')
        costmap.setCost(18 + col, 40 - row, costmap_2d::LETHAL_OBSTACLE);
  base_local_planner::CostmapModel costmap_model(costmap);

  // Nearby converter polygons captured at the stalled pose.
  const std::vector<std::vector<std::pair<double, double>>> polygons = {
    {{-1.330,-1.004},{-0.830,-1.304},{-0.630,-1.104},{-1.030,-0.704}},
    {{-1.330,-0.204},{-1.230,-0.504},{-0.830,-0.704},{-0.930,-0.104},{-1.030,-0.004}},
    {{-0.930,-0.004},{-0.730,-0.604},{-0.530,-0.604},{-0.530,0.296},{-0.830,0.396}},
    {{-0.830,0.496},{-0.530,0.396},{-0.230,0.496},{-0.630,0.896},{-0.830,0.896}},
    {{-0.830,0.996},{-0.230,0.696},{-0.330,1.196},{-0.730,1.296}},
    {{-0.730,-1.404},{-0.230,-1.804},{-0.130,-1.704},{-0.230,-1.204},{-0.530,-1.104}},
    {{-0.730,-0.804},{-0.630,-1.004},{-0.030,-1.304},{-0.130,-0.804}},
    {{-0.630,1.396},{-0.130,0.796},{0.070,0.796},{-0.030,1.396},{-0.230,1.496}},
    {{-0.230,0.596},{0.070,0.596},{0.170,1.196}},
    {{-0.030,-1.404},{0.370,-1.804},{0.570,-1.504},{0.170,-1.104},{-0.030,-1.104}},
    {{-0.030,-1.004},{0.570,-1.304},{0.470,-0.804},{0.070,-0.704}},
    {{0.370,-0.704},{0.770,-1.204},{0.870,-0.604},{0.670,-0.504}},
    {{0.870,-0.504},{0.970,-1.104},{1.170,-1.104},{1.270,-1.004},{1.170,-0.304}}
  };
  teb_local_planner::ObstContainer obstacles;
  for (const auto& vertices : polygons)
  {
    auto obstacle = boost::make_shared<teb_local_planner::PolygonObstacle>();
    for (const auto& vertex : vertices)
      obstacle->pushBackVertex(vertex.first, vertex.second);
    obstacle->finalizePolygon();
    obstacles.push_back(obstacle);
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
  std::vector<geometry_msgs::Point> padded_footprint(4);
  padded_footprint[0].x = -0.27; padded_footprint[0].y = -0.22;
  padded_footprint[1].x = -0.27; padded_footprint[1].y = 0.22;
  padded_footprint[2].x = 0.37;  padded_footprint[2].y = 0.22;
  padded_footprint[3].x = 0.37;  padded_footprint[3].y = -0.22;
  teb_local_planner::ViaPointContainer vias;
  teb_local_planner::HomotopyClassPlanner planner(config, &obstacles, {}, &vias);
  InspectablePlannerROS wrapper;
  tf2_ros::Buffer tf;
  tf.setUsingDedicatedThread(true);
  geometry_msgs::TransformStamped identity;
  identity.header.frame_id = "map";
  identity.child_frame_id = "test_frame";
  identity.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf.setTransform(identity, "test", true));

  // Recorded trajectory start and odometry at 15:32:15.
  teb_local_planner::PoseSE2 robot(0.070409, -0.128204, 0.475182);
  geometry_msgs::Twist velocity;
  velocity.linear.x = -0.056126;
  velocity.angular.z = -0.146412;
  int cursor = 3;
  int last_progress_cycle = 0;
  int infeasible = 0;
  bool reached_outgoing_path = false;
  double closest_to_turn = std::numeric_limits<double>::infinity();

  // Integrate at 20 Hz with static recorded obstacles, without ROS recovery or converter updates.
  for (int cycle = 0; cycle < 1200 && !reached_outgoing_path; ++cycle)
  {
    closest_to_turn = std::min(closest_to_turn, std::hypot(robot.x() - points[6][0],
                                                          robot.y() - points[6][1]));
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
    ASSERT_TRUE(wrapper.transformGlobalPlan(tf, path, pose, costmap, "map", 1.2,
                                            local, &goal, &transform, cursor, 1.0, &next_cursor));
    ASSERT_GE(next_cursor, cursor);
    if (next_cursor > cursor)
      last_progress_cycle = cycle;
    cursor = next_cursor;
    if (cycle - last_progress_cycle >= 600)
      break;
    reached_outgoing_path = cursor >= 14 &&
        std::hypot(robot.x() - points[14][0], robot.y() - points[14][1]) < 0.25;
    if (reached_outgoing_path)
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
    double goal_yaw = wrapper.estimateLocalGoalOrientation(path, local.back(), goal, transform);
    if (goal + 1 < static_cast<int>(path.size()))
      wrapper.adjustLocalGoalOrientation(costmap_model, padded_footprint, local.back().pose.position,
                                         goal_yaw, 0.0, 0.0);
    local.back().pose.orientation.z = std::sin(goal_yaw / 2.0);
    local.back().pose.orientation.w = std::cos(goal_yaw / 2.0);
    local.front() = pose;
    double vx = 0.0, vy = 0.0, omega = 0.0;
    if (!planner.plan(local, &velocity) ||
        !planner.isTrajectoryFeasible(&costmap_model, padded_footprint, 0.0, 0.0, 5, 1.0) ||
        !planner.getVelocityCommand(vx, vy, omega, 1))
    {
      ++infeasible;
      planner.clearPlanner();
      vx = omega = 0.0;
    }
    ASSERT_TRUE(std::isfinite(vx) && std::isfinite(omega));
    velocity.linear.x = std::max(-0.2, std::min(0.3, vx));
    velocity.angular.z = std::max(-1.0, std::min(1.0, omega));
    robot.x() += velocity.linear.x * std::cos(robot.theta()) * 0.05;
    robot.y() += velocity.linear.x * std::sin(robot.theta()) * 0.05;
    robot.theta() = g2o::normalize_theta(robot.theta() + velocity.angular.z * 0.05);
    ASSERT_GE(costmap_model.footprintCost(robot.x(), robot.y(), robot.theta(), padded_footprint, 0.0, 0.0), 0.0);
  }
  EXPECT_TRUE(reached_outgoing_path) << "Cursor " << cursor + 1169 << ", infeasible cycles " << infeasible
                                    << ", closest approach to turn " << closest_to_turn;
  EXPECT_LT(closest_to_turn, 0.2);
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recorded_loop_stall_test", ros::init_options::AnonymousName);
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
