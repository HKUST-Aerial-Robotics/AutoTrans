// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h"

namespace payload_planner
{

  // SECTION interfaces for setup and query

  PlannerManager::PlannerManager() {}

  PlannerManager::~PlannerManager()
  {
    std::cout << "des manager" << std::endl;
  }

  void PlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;
  }

  bool PlannerManager::computeInitReferenceState(const Eigen::Vector3d &start_pt,
                                                    const Eigen::Vector3d &start_vel,
                                                    const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &start_jerk,
                                                    const Eigen::Vector3d &local_target_pt,
                                                    const Eigen::Vector3d &local_target_vel,
                                                    const Eigen::Vector3d &local_target_acc,
                                                    const double &ts,
                                                    poly_traj::MinSnapOpt &initMJO,
                                                    const bool flag_polyInit)
  {
    static bool flag_first_call = true;

    /*** case 1: use A* initialization ***/
    if (flag_first_call || flag_polyInit)
    {
      flag_first_call = false;
      /* basic params */
      Eigen::Matrix<double,3,4> headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      // poly_traj::Trajectory traj;
      vector<Eigen::Vector3d> simple_path;

      headState.col(0) = start_pt; headState.col(1) = start_vel;headState.col(2) = start_acc;headState.col(3) = start_jerk;
      tailState.col(0) = local_target_pt; tailState.col(1) = local_target_vel;tailState.col(2) = local_target_acc;tailState.col(3) = Eigen::Vector3d::Zero();

      /* step 1: A* search and generate init traj */
      Eigen::MatrixXd ctl_points;

      int status = ploy_traj_opt_->astarWithMinTraj(headState, tailState, simple_path, ctl_points, initMJO);
      if (status == KinodynamicAstar::NO_PATH || status == KinodynamicAstar::TOO_SHORT)
      {
        return false;
      }

      // show the init simple_path
      vector<vector<Eigen::Vector3d>> path_view;
      path_view.push_back(simple_path);
      visualization_->displayAStarList(path_view, 0);

      // show the init traj for debug
      std::vector<Eigen::Vector3d> point_set;
      for (int i = 0; i < ctl_points.cols(); ++i)
        point_set.push_back(ctl_points.col(i));
      visualization_->displayInitPathListDebug(point_set, 0.2, 0);
    }

    /*** case 2: initialize from previous optimal trajectory ***/
    else
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix<double, 3,4> headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc, start_jerk;
      tailState << local_target_pt, local_target_vel,local_target_acc, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88");
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  void PlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel,Eigen::Vector3d &local_target_acc)
  {
    double t;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 40.0 / pp_.max_vel_;
    // double dist_min = 9999, dist_min_t = 0.0;
    double sum_dist = 0;
    Eigen::Vector3d last_pt = start_pt;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
      double dist = (pos_t - last_pt).norm();
      last_pt = pos_t;
      sum_dist += dist;

      if (sum_dist >= planning_horizen)
      {
        cout << "pose " << pos_t.transpose() <<endl;
        cout << "start_pt " << start_pt.transpose() <<endl;
        cout << "t " << t <<endl;
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
      local_target_acc = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
      local_target_acc = traj_.global_traj.traj.getAcc(t - traj_.global_traj.global_start_time);
    }
  }

  bool PlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,const Eigen::Vector3d &start_jerk,
      const double trajectory_start_time, const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const Eigen::Vector3d &local_target_acc,
      const bool flag_polyInit, const bool have_local_traj)
  {
    static int count = 0;

    printf("\033[47;30m\n[replan %d]==============================================\033[0m\n", count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT ***/
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

   
    poly_traj::MinSnapOpt initMJO;
    if (!computeInitReferenceState(start_pt, start_vel, start_acc, start_jerk,
                                   local_target_pt, local_target_vel,local_target_acc,
                                   ts, initMJO, flag_polyInit))
    {
      return false;
    }


    Eigen::MatrixXd cstr_pts = initMJO.getInitConstrainPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    ploy_traj_opt_->setControlPoints(cstr_pts);

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    poly_traj::Trajectory<7> initTraj = initMJO.getTraj();
    int PN = initTraj.getPieceNum();
    Eigen::MatrixXd all_pos = initTraj.getPositions();
    Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
    Eigen::Matrix<double, 3, 4> headState, tailState;
    headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0),initTraj.getJuncJerk(0);
    tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN),initTraj.getJuncJerk(PN);
    flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(headState, tailState,
                                                            innerPts, initTraj.getDurations(),
                                                            cstr_pts);
 
    t_opt = ros::Time::now() - t_start;

    if (!flag_success)
    {
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
      return false;
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt).toSec()
         << "\033[0m,init:" << t_init.toSec()
         << ",optimize:" << t_opt.toSec()
         << ",avg_time=" << sum_time / count_success
         << ",count_success= " << count_success << endl;
    average_plan_time_ = sum_time / count_success;

    if (have_local_traj)
    {
      // double delta_replan_time = trajectory_start_time - ros::Time::now().toSec();
      // if (delta_replan_time > 0)
      //   ros::Duration(delta_replan_time).sleep();
      traj_.setLocalTraj(ploy_traj_opt_->getMinJerkOptPtr()->getTraj(), trajectory_start_time);
    }
    else
    {
      traj_.setLocalTraj(ploy_traj_opt_->getMinJerkOptPtr()->getTraj(), ros::Time::now().toSec()); // todo time
    }

    visualization_->displayOptimalList(cstr_pts, 0);

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool PlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 4> headState, tailState;
    headState << stop_pos, ZERO, ZERO,ZERO;
    tailState = headState;
    poly_traj::MinSnapOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    traj_.setLocalTraj(stopMJO.getTraj(), ros::Time::now().toSec());

    return true;
  }

  bool PlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    poly_traj::MinSnapOpt globalMJO;
    Eigen::Matrix<double, 3, 4> headState, tailState;
    headState << start_pos, start_vel, start_acc, Eigen::Vector3d::Zero();
    tailState << waypoints.back(), end_vel, end_acc,Eigen::Vector3d::Zero();
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {
      innerPts.resize(3, waypoints.size() - 1);
      for (size_t i = 0; i < waypoints.size() - 1; i++)
        innerPts.col(i) = waypoints[i];
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }
    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_/1.0;
    Eigen::VectorXd time_vec(waypoints.size());
    for (int j = 0; j < 5; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalMJO.generate(innerPts, time_vec);

      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 5)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      des_vel /= 1.5;
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }


} // namespace payload_planner
