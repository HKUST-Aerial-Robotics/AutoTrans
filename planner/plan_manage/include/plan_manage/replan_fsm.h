#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <plan_manage/planning_visualization.h>
#include <quadrotor_msgs/PolynomialTraj.h>

#include <fstream>
#include <iostream>
using std::vector;

namespace payload_planner
{

  class ReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      FIX_TARGET = 2,
      PRESET_TARGET = 3
    };

    /* planning utils */
    PlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_, wpt_id_;
    int goal_num_;
    double goalpoints_[50][3];
    double planning_horizen_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    int last_end_id_;
    double replan_trajectory_time_;

    std::vector<Eigen::Vector3d> wps_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_local_traj_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_jerk_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                                    // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_, local_target_acc_;               // local target state
    int current_wp_;

    bool flag_escape_emergency_;
    bool flag_relan_astar_;

    GlobalTrajData frontend_traj_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_;
    ros::Publisher replan_pub_, poly_traj_pub_, data_disp_pub_;

    ros::Publisher reached_pub_, start_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init);  // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos); // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init);

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, ReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    // void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void polyTraj2ROSMsg(quadrotor_msgs::PolynomialTraj &msg);
    bool planNextWaypoint(const Eigen::Vector3d next_wp);
    bool mondifyInCollisionFinalGoal();
    void readGivenWpsAndPlan();

  public:
    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace payload_planner

#endif