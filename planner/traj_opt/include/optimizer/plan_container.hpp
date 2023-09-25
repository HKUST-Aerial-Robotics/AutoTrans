#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <optimizer/poly_traj_utils.hpp>

using std::vector;

namespace payload_planner
{

  struct GlobalTrajData
  {
    poly_traj::Trajectory<7> traj;
    double global_start_time; // world time
    double duration;

    /* Global traj time.
       The corresponding global trajectory time of the current local target.
       Used in local target selection process */
    double glb_t_of_lc_tgt;
    /* Global traj time.
       The corresponding global trajectory time of the last local target.
       Used in initial-path-from-last-optimal-trajectory generation process */
    double last_glb_t_of_lc_tgt;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct LocalTrajData
  {
    poly_traj::Trajectory<7> traj;
    int drone_id; // A negative value indicates no received trajectories.
    int traj_id;
    double duration;
    double start_time; // world time
    double end_time;   // world time
    Eigen::Vector3d start_pos;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class TrajContainer
  {
  public:
    GlobalTrajData global_traj;
    LocalTrajData local_traj;

    TrajContainer()
    {
      local_traj.traj_id = 0;
    }
    ~TrajContainer() {}

    void setGlobalTraj(const poly_traj::Trajectory<7> &trajectory, const double &world_time)
    {
      global_traj.traj.clear(); //Must clear in Ubuntu 18.04
      global_traj.traj = trajectory;
      global_traj.duration = trajectory.getTotalDuration();
      global_traj.global_start_time = world_time;
      global_traj.glb_t_of_lc_tgt = world_time;
      global_traj.last_glb_t_of_lc_tgt = -1.0;

      local_traj.drone_id = -1;
      local_traj.duration = 0.0;
      local_traj.traj_id = 0;
    }

    void setLocalTraj(const poly_traj::Trajectory<7> &trajectory, const double &world_time, const int drone_id = -1)
    {
      local_traj.drone_id = drone_id;
      local_traj.traj_id++;
      local_traj.duration = trajectory.getTotalDuration();
      local_traj.start_pos = trajectory.getJuncPos(0);
      local_traj.start_time = world_time;
      local_traj.traj.clear(); //Must clear in Ubuntu 18.04
      local_traj.traj = trajectory;
    }
  };

  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_, max_acc_;    // physical limits
    double polyTraj_piece_length; // distance between adjacient Spinle control points
    double planning_horizen_;

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

} // namespace payload_planner

#endif