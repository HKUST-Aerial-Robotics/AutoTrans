

#include <plan_manage/replan_fsm.h>

namespace payload_planner
{
  void ReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    flag_escape_emergency_ = true;
    flag_relan_astar_ = false;
    have_local_traj_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0);

    have_trigger_ = !flag_realworld_experiment_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    nh.param("fsm/goal_num", goal_num_, -1);
    for (int i = 0; i < goal_num_; i++)
    {
      nh.param("fsm/target" + to_string(i) + "_x", goalpoints_[i][0], -1.0);
      nh.param("fsm/target" + to_string(i) + "_y", goalpoints_[i][1], -1.0);
      nh.param("fsm/target" + to_string(i) + "_z", goalpoints_[i][2], -1.0);
    }
    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new PlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &ReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &ReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &ReplanFSM::odometryCallback, this);

    poly_traj_pub_ = nh.advertise<quadrotor_msgs::PolynomialTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &ReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::FIX_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &ReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &ReplanFSM::triggerCallback, this);
    }
  }

  void ReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    if (!have_odom_)
    {
      ROS_ERROR("Reject formation flight!");
      return;
    }
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_]);
  }

  bool ReplanFSM::mondifyInCollisionFinalGoal()
  {
    if (planner_manager_->grid_map_->getInflateOccupancy(end_pt_))
    {
      Eigen::Vector3d orig_goal = end_pt_;
      double t_step = planner_manager_->grid_map_->getResolution() / planner_manager_->pp_.max_vel_;
      for (double t = planner_manager_->traj_.global_traj.duration; t > 0; t -= t_step)
      {
        Eigen::Vector3d pt = planner_manager_->traj_.global_traj.traj.getPos(t);
        if (!planner_manager_->grid_map_->getInflateOccupancy(pt))
        {
          if (planNextWaypoint(pt)) // final_goal_=pt inside if success
          {
            ROS_INFO("Current in-collision waypoint (%.3f, %.3f %.3f) has been modified to (%.3f, %.3f %.3f)",
                     orig_goal(0), orig_goal(1), orig_goal(2), end_pt_(0), end_pt_(1), end_pt_(2));
            return true;
          }
        }

        if (t <= t_step)
        {
          ROS_ERROR("Can't find any collision-free point on global traj.");
        }
      }
    }

    return false;
  }

  void ReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 300)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      bool success = planFromGlobalTraj(1);
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
      }
      else
      {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      bool success;
      if (flag_relan_astar_)
        success = planFromLocalTraj(true);
      else
        success = planFromLocalTraj(false);

      if (success)
      {
        flag_relan_astar_ = false;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        flag_relan_astar_ = true;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);

      Eigen::Vector3d pos = info->traj.getPos(t_cur);

      if (mondifyInCollisionFinalGoal()) // case 1: find that current goal is in obstacles
      {
        // pass
      }
      else if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
               (wpt_id_ < waypoint_num_ - 1) &&
               (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          // if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          // {
          //   wpt_id_ = 0;
          //   planNextWaypoint(wps_[wpt_id_]);
          // }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
          // return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;

      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  bool ReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET)
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  void ReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    double t_2_3 = info->duration * 2 / 3;
    double t_temp;
    bool occ = false;
    for (double t = t_cur; t < info->duration; t += time_step)
    {
      // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
      if (t_cur < t_2_3 && t >= t_2_3)
        break;

      if (planner_manager_->ploy_traj_opt_->checkCollision(info->traj, t))
      {
        ROS_WARN("drone is too close to the obstacle at relative time %f!", t / info->duration);
        t_temp = t;
        occ = true;
        break;
      }
    }

    if (occ)
    {
      /* Handle the collided case immediately */
      ROS_INFO("Try to replan a safe trajectory");
      if (planFromLocalTraj(false))
      {
        ROS_INFO("Plan success when detect collision.");
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }
      else
      {
        if (t_temp - t_cur < emergency_time_) // 1.0s of emergency time
        {
          ROS_WARN("Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }
        else
        {
          ROS_WARN("current traj in collision, replan.");
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }

  void ReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    readGivenWpsAndPlan();
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void ReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    have_trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      end_pt_ << msg->pose.position.x, msg->pose.position.y, waypoints_[0][2];
    }
    else if (target_type_ == TARGET_TYPE::FIX_TARGET)
    {
      end_pt_ << waypoints_[0][0], waypoints_[0][1], waypoints_[0][2];
    }

    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);

    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);
    have_local_traj_ = false; // reset
    if (success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void ReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void ReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void ReplanFSM::printFSMExecState()
  {
    static string state_str[] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    static int last_printed_state = -1, dot_nums = 0;

    if (exec_state_ != last_printed_state)
      dot_nums = 0;
    else
      dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    // if (!have_trigger_)
    // {
    //   cout << ", waiting for trigger";
    // }

    // cout << string(dot_nums, '.') << endl;

    fflush(stdout);
  }

  std::pair<int, ReplanFSM::FSM_EXEC_STATE> ReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void ReplanFSM::polyTraj2ROSMsg(quadrotor_msgs::PolynomialTraj &msg)
  {

    auto data = &planner_manager_->traj_.local_traj;

    msg.trajectory_id = data->traj_id;
    msg.header.stamp = ros::Time(data->start_time);
    msg.action = msg.ACTION_ADD;
    int piece_num = data->traj.getPieceNum();

    for (int i = 0; i < piece_num; ++i)
    {
      quadrotor_msgs::PolynomialMatrix piece;
      piece.num_dim = data->traj.getPiece(i).getDim();
      piece.num_order = data->traj.getPiece(i).getOrder();
      piece.duration = data->traj.getPiece(i).getDuration();
      auto cMat = data->traj.getPiece(i).getCoeffMat();
      piece.data.assign(cMat.data(), cMat.data() + cMat.rows() * cMat.cols());
      msg.trajectory.emplace_back(piece);
    }
  }

  void ReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps;
    if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      wps.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        wps[i](0) = waypoints_[i][0];
        wps[i](1) = waypoints_[i][1];
        wps[i](2) = waypoints_[i][2];
      }
      end_pt_ = wps.back();
      for (size_t i = 0; i < (size_t)waypoint_num_; i++)
      {
        visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
        ros::Duration(0.001).sleep();
      }
    }
    else
      return;

    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(),
                                                             Eigen::Vector3d::Zero(), wps,
                                                             Eigen::Vector3d::Zero(),
                                                             Eigen::Vector3d::Zero());

    if (success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  bool ReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/)
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    start_jerk_.setZero();

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true))
      {
        return true;
      }
    }
    return false;
  }

  bool ReplanFSM::planFromLocalTraj(bool flag_use_poly_init)
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);
    start_jerk_ = info->traj.getJer(t_cur);

    bool success = callReboundReplan(flag_use_poly_init);

    if (!success)
      return false;

    return true;
  }

  bool ReplanFSM::callReboundReplan(bool flag_use_poly_init)
  {

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_, local_target_acc_);

    Eigen::Vector3d desired_start_pt, desired_start_vel, desired_start_acc, desired_start_jerk;
    double desired_start_time;
    if (have_local_traj_)
    {
      desired_start_time = ros::Time::now().toSec() + replan_trajectory_time_;
      desired_start_pt =
          planner_manager_->traj_.local_traj.traj.getPos(desired_start_time - planner_manager_->traj_.local_traj.start_time);
      desired_start_vel =
          planner_manager_->traj_.local_traj.traj.getVel(desired_start_time - planner_manager_->traj_.local_traj.start_time);
      desired_start_acc =
          planner_manager_->traj_.local_traj.traj.getAcc(desired_start_time - planner_manager_->traj_.local_traj.start_time);
      desired_start_jerk =
          planner_manager_->traj_.local_traj.traj.getJer(desired_start_time - planner_manager_->traj_.local_traj.start_time);
    }
    else
    {
      desired_start_time = ros::Time::now().toSec();
      desired_start_pt = start_pt_;
      desired_start_vel = start_vel_;
      desired_start_acc = start_acc_;
      desired_start_jerk = start_jerk_;
    }
    bool plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc, desired_start_jerk,
        desired_start_time, local_target_pt_, local_target_vel_, local_target_acc_,
        (have_new_target_ || flag_use_poly_init), have_local_traj_);

    have_new_target_ = false;

    if (plan_success)
    {
      quadrotor_msgs::PolynomialTraj msg;
      const auto &dynamic_param = planner_manager_->ploy_traj_opt_;
      visualization_->visualizeDoubleball(planner_manager_->traj_.local_traj.traj, 5, dynamic_param->L_length_, dynamic_param->grav_, dynamic_param->payload_size_, dynamic_param->drone_size_);
      polyTraj2ROSMsg(msg);
      poly_traj_pub_.publish(msg);
      // broadcast_ploytraj_pub_.publish(msg);
      have_local_traj_ = true;
    }

    return plan_success;
  }

  bool ReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    planner_manager_->EmergencyStop(stop_pos);

    quadrotor_msgs::PolynomialTraj msg;
    msg.action = quadrotor_msgs::PolynomialTraj::ACTION_ABORT;
    poly_traj_pub_.publish(msg);

    return true;
  }

} // namespace payload_planner
