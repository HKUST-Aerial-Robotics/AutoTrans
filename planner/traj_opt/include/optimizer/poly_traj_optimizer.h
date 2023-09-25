#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "optimizer/lbfgs.hpp"
#include <optimizer/plan_container.hpp>
#include "poly_traj_utils.hpp"
#include <fstream>
#include "flatness.hpp"
namespace payload_planner
{

  class ConstrainPoints
  {
  public:
    int cp_size; // deformation points
    Eigen::MatrixXd points;

    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      points.resize(3, size_set);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {
  public:
    double payload_size_;
    double drone_size_;
    double L_length_;
    double safe_margin_;
    double sample_ball_distance_;
    double grav_;

  private:
    GridMap::Ptr grid_map_;
    KinodynamicAstar::Ptr kino_a_star_;
    poly_traj::MinSnapOpt jerkOpt_;
    ConstrainPoints cps_;

    int cps_num_prePiece_; // number of distinctive constrain points each piece
    int variable_num_;     // optimization variables
    int piece_num_;        // poly traj piece numbers
    int iter_num_;         // iteration of the solver

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_;       // obstacle weight
    double wei_feas_;      // feasibility weight
    double wei_quad_feas_; // Multicopter feasibility weight
    double wei_sqrvar_;    // squared variance weight
    double wei_time_;      // time weight

    double max_vel_, max_acc_; // dynamic limits
    double thrust_mean_, thrust_sqr_radi_;
    double cos_theta_max_;

    double mass_payload_, mass_quad_;

    double t_now_;

    flatness::PayloadFlatnessMap flat;

  public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    /* set variables */
    void setParam(ros::NodeHandle &nh);
    void setEnvironment(const GridMap::Ptr &map);
    void setControlPoints(const Eigen::MatrixXd &points);

    /* helper functions */
    inline ConstrainPoints getControlPoints() { return cps_; }
    inline const ConstrainPoints *getControlPointsPtr(void) { return &cps_; }
    inline const poly_traj::MinSnapOpt *getMinJerkOptPtr(void) { return &jerkOpt_; }
    inline int get_cps_num_prePiece_() { return cps_num_prePiece_; };
    bool checkCollision(poly_traj::Trajectory<7> &traj, double t);

    /* main planning API */
    bool OptimizeTrajectory_lbfgs(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                                  const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                                  Eigen::MatrixXd &optimal_points);

    int astarWithMinTraj(const Eigen::Matrix<double, 3, 4> &iniState,
                         const Eigen::Matrix<double, 3, 4> &finState,
                         std::vector<Eigen::Vector3d> &simple_path,
                         Eigen::MatrixXd &ctl_points,
                         poly_traj::MinSnapOpt &frontendMJ);

  private:
    /* callbacks by the L-BFGS optimizer */
    // static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionCallback(void *instance,
                                       const Eigen::VectorXd &x,
                                       Eigen::VectorXd &g);

    static int earlyExitCallback(void *instance, const Eigen::VectorXd &x,
                                 const Eigen::VectorXd &g,
                                 const double fx,
                                 const double step,
                                 const int k,
                                 const int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);
    bool smoothedL1(const double &x, const double &mu, double &f, double &df);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    bool obstacleGradCostfordrone(const int i_dp,
                                  const Eigen::Vector3d &pos,
                                  const Eigen::Vector3d &acc,
                                  Eigen::Vector3d &gradp,
                                  Eigen::Vector3d &gradacc,
                                  double &costp);
    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    bool feasibilityGradCostDroneDynamic(const Eigen::Vector3d &a,
                                         const Eigen::Vector3d &j,
                                         const Eigen::Vector3d &s,
                                         Eigen::Vector3d &grada,
                                         Eigen::Vector3d &gradj,
                                         Eigen::Vector3d &grads,
                                         double &costd);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    bool checkCollision(void);

    inline void normalizeFDF(const Eigen::Vector3d &x,
                             Eigen::Vector3d &xNor,
                             Eigen::Matrix3d &G);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace payload_planner
#endif