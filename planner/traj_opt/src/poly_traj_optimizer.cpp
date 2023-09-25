#include "optimizer/poly_traj_optimizer.h"
// using namespace std;

namespace payload_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points)
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();

    jerkOpt_.reset(iniState, finState, piece_num_);

    double final_cost;
    variable_num_ = 4 * (piece_num_ - 1) + 1;

    ros::Time t1, t2;

    Eigen::VectorXd q(variable_num_);

    Eigen::Map<Eigen::MatrixXd> pt(q.data(), 3, piece_num_ - 1);
    pt = initInnerPts;
    Eigen::Map<Eigen::VectorXd> Vt(q.data() + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = 1.0e-6;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.max_linesearch = 256;
    lbfgs_params.past = 5;

    /* trick : for real-time optimization */
    lbfgs_params.max_iterations = 1000; // 200 for replan

    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;
    /* ---------- optimize ---------- */

    t1 = ros::Time::now();

    int result = lbfgs::lbfgs_optimize(
        q,
        final_cost,
        PolyTrajOptimizer::costFunctionCallback,
        NULL,
        PolyTrajOptimizer::earlyExitCallback,
        this,
        lbfgs_params);

    // test collision
    bool occ = false;
    occ = checkCollision();

    t2 = ros::Time::now();
    double time_ms = (t2 - t1).toSec() * 1000;

    printf("\033[32miter=%d,time(ms)=%5.3f, \n\033[0m", iter_num_, time_ms);
    // if ((result != lbfgs::LBFGS_CONVERGENCE) && (result != lbfgs::LBFGS_STOP))
    // {
    //   ROS_ERROR("The optimization result is : %s", lbfgs::lbfgs_strerror(result));
    // }
    // else
    // {
    //   ROS_INFO("The optimization result is : %s", lbfgs::lbfgs_strerror(result));
    // }
    optimal_points = cps_.points;

    if (occ)
      return false;
    else
      return true;
  }

  bool PolyTrajOptimizer::smoothedL1(const double &x,
                                     const double &mu,
                                     double &f,
                                     double &df)
  {
    if (x < 0.0)
    {
      f = 0;
      df = 0;
      return false;
    }
    else if (x > mu)
    {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    }
    else
    {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);

      // const double xdmu = x / mu;
      // f = 0.5*xdmu * x;
      // df = xdmu;
      return true;
    }
  }

  bool PolyTrajOptimizer::checkCollision(void)
  {
    // only check the 2/3 of trajectory
    bool occ = false;
    double dt = 0.01;
    poly_traj::Trajectory<7> traj = jerkOpt_.getTraj();
    double T_all = traj.getTotalDuration();
    int i_end = round(T_all / dt) * 3 / 5;
    double t = 0.05; // skip the start point

    for (int i = 5; i < i_end; i++)
    {
      occ = checkCollision(traj, t);
      t += dt;
      if (occ)
      {
        ROS_WARN("drone is too close to the obstacle at relative time %f!", t);
        break;
      }
    }
    return occ;
  }

  bool PolyTrajOptimizer::checkCollision(poly_traj::Trajectory<7> &traj, double t)
  {
    Eigen::Vector3d pos = traj.getPos(t);
    // if(grid_map_->getInflateOccupancy(pos))
    if (grid_map_->getDistance(pos) < payload_size_)
    {
      ROS_WARN_STREAM("payload distance  " << grid_map_->getDistance(pos) << "!");
      return true;
    }
    Eigen::Vector3d norm_negTp_ = traj.getAcc(t);
    norm_negTp_(2) += grav_;
    norm_negTp_.normalize();

    // check the points on line
    double check_length = L_length_ - drone_size_;
    for (double l_dis = payload_size_; l_dis <= check_length; l_dis += sample_ball_distance_)
    {
      Eigen::Vector3d pos_on_line = pos + l_dis * norm_negTp_; // the sample points on line

      // if(grid_map_->getInflateOccupancy(pos_on_line))
      if (grid_map_->getDistance(pos_on_line) < payload_size_)
      {
        ROS_WARN_STREAM("line distance  " << grid_map_->getDistance(pos_on_line) << " Line pos" << l_dis << "!");
        return true;
      }
    }
    // check the drone
    Eigen::Vector3d posQ = pos + L_length_ * norm_negTp_;
    if (grid_map_->getDistance(posQ) < drone_size_)
    {
      ROS_WARN_STREAM("drone distance  " << grid_map_->getDistance(posQ) << "!");
      return true;
    }
    return false;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *instance,
                                                 const Eigen::VectorXd &x,
                                                 Eigen::VectorXd &g)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(instance);

    Eigen::Map<const Eigen::MatrixXd> P(x.data(), 3, opt->piece_num_ - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x.data() + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(g.data(), 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(g.data() + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    opt->VirtualT2RealT(t, T);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd cost_terms(6);

    opt->jerkOpt_.generate(P, T);

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    opt->addPVAGradCost2CT(gradT, cost_terms, opt->cps_num_prePiece_); // Time int cost

    opt->jerkOpt_.getGrad2TP(gradT, gradP);

    // opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);
    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    opt->iter_num_ += 1;
    double costall = smoo_cost + cost_terms.sum() + time_cost;
    return costall;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const Eigen::VectorXd &x,
                                           const Eigen::VectorXd &g,
                                           const double fx,
                                           const double step,
                                           const int k,
                                           const int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {

    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer, snap, crakle;
    Eigen::Vector3d gradp, gradv, grada, gradj, grads;
    double costp, costv, costa, costd;
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4, beta5;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha;
    Eigen::Matrix<double, 8, 3> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc, gradViolaSc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt, gradViolaSt;
    double omg;
    int i_dp = 0;
    costs.setZero();

    // int innerLoop;
    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 8, 3> &c = jerkOpt_.get_b().block<8, 3>(i * 8, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s3 * s3;
        s7 = s6 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5, beta0(6) = s6, beta0(7) = s7;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4, beta1(6) = 6.0 * s5, beta1(7) = 7.0 * s6;
        beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3, beta2(6) = 30.0 * s4, beta2(7) = 42.0 * s5;
        beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2, beta3(6) = 120.0 * s3, beta3(7) = 210.0 * s4;
        beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1, beta4(6) = 360.0 * s2, beta4(7) = 840.0 * s3;
        beta5(0) = 0.0, beta5(1) = 0.0, beta5(2) = 0.0, beta5(3) = 0.0, beta5(4) = 0.0, beta4(5) = 120.0, beta5(6) = 720.0 * s1, beta5(7) = 2520.0 * s2;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        snap = c.transpose() * beta4;
        crakle = c.transpose() * beta5;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;
        // collision
        gradp.setZero();
        grada.setZero();
        costp = 0;
        if (obstacleGradCostfordrone(i_dp, pos, acc, gradp, grada, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<8, 3>(i * 8, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);

          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<8, 3>(i * 8, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);

          costs(0) += omg * step * costp;
        }

        grada.setZero();
        gradj.setZero();
        grads.setZero();
        costd = 0;

        if (feasibilityGradCostDroneDynamic(acc, jer, snap, grada, gradj, grads, costd))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<8, 3>(i * 8, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);

          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * snap;
          jerkOpt_.get_gdC().block<8, 3>(i * 8, 0) += omg * step * gradViolaJc;
          gdT(i) += omg * (costa / K + step * gradViolaJt);

          gradViolaSc = beta4 * grads.transpose();
          gradViolaSt = alpha * grads.transpose() * crakle;
          jerkOpt_.get_gdC().block<8, 3>(i * 8, 0) += omg * step * gradViolaSc;
          gdT(i) += omg * (costa / K + step * gradViolaSt);

          costs(1) += omg * step * costd;
        }

        // feasibility
        gradv.setZero();
        costv = 0;
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<8, 3>(i * 8, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(2) += omg * step * costv;
        }
        grada.setZero();
        costa = 0;
        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<8, 3>(i * 8, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(3) += omg * step * costa;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
      // t += jerkOpt_.get_T1()(i);
    }

    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s3 * s3;
        s7 = s6 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5, beta0(6) = s6, beta0(7) = s7;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4, beta1(6) = 6.0 * s5, beta1(7) = 7.0 * s6;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<8, 3>(i * 8, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<8, 3>(i * 8, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }
    costs(4) += var;
  }

  inline void PolyTrajOptimizer::normalizeFDF(const Eigen::Vector3d &x,
                                              Eigen::Vector3d &xNor,
                                              Eigen::Matrix3d &G)
  {
    const double eps = 1.0e-6;
    const double t_0 = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + eps; // Add eps to smooth the gradient
    const double t_1 = sqrt(t_0);
    const double inv_t_1 = 1.0 / t_1;
    xNor = x * inv_t_1;
    G = inv_t_1 * Eigen::Matrix3d::Identity() - 1.0 / (t_0)*inv_t_1 * x * x.transpose();
    return;
  }

  bool PolyTrajOptimizer::obstacleGradCostfordrone(const int i_dp,
                                                   const Eigen::Vector3d &pos,
                                                   const Eigen::Vector3d &acc,
                                                   Eigen::Vector3d &gradp,
                                                   Eigen::Vector3d &gradacc,
                                                   double &costp)
  {
    // if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)
    //   return false;

    bool ret = false;

    costp = 0;

    // compute the drone pos
    Eigen::Vector3d negTp = acc;
    negTp(2) += grav_;
    Eigen::Vector3d norm_negTp_;
    Eigen::Matrix3d grad_norm_negTp_;
    normalizeFDF(negTp, norm_negTp_, grad_norm_negTp_);

    double dist;
    double dist_err;
    double map_resolution = grid_map_->getResolution();
    double min_size = payload_size_ < drone_size_ ? payload_size_ : drone_size_;
    // use esdf check the payload pos
    dist = grid_map_->getDistance(pos);                              // coarse distance
    dist_err = payload_size_ + safe_margin_ + map_resolution - dist; // conservative checking
    if (dist_err > 0)
    {

      Eigen::Vector3d dist_grad;
      grid_map_->evaluateEDTWithGrad(pos, dist, dist_grad);
      dist_err = payload_size_ + safe_margin_ - dist; // refine distance
      if (dist_err > 0)
      {
        ret = true;
        double grad_l1, cost_p_dist;
        smoothedL1(dist_err, 0.05, cost_p_dist, grad_l1);
        costp += wei_obs_ * cost_p_dist;
        gradp += -wei_obs_ * grad_l1 * dist_grad;
      }
    }

    // check the points on line
    double check_length = L_length_ - drone_size_;
    int avg = (int)(check_length / sample_ball_distance_) + 1;
    double factor = 1.0 / (double)avg;
    int num_size = 0;
    for (double l_dis = payload_size_; l_dis <= check_length; l_dis += sample_ball_distance_)
    {
      Eigen::Vector3d pos_on_line = pos + l_dis * norm_negTp_; // the sample points on line

      dist = grid_map_->getDistance(pos_on_line);                 // coarse distance
      dist_err = min_size + safe_margin_ + map_resolution - dist; // conservative checking
      if (dist_err > 0)
      {

        Eigen::Vector3d dist_grad;
        grid_map_->evaluateEDTWithGrad(pos_on_line, dist, dist_grad);
        dist_err = min_size + safe_margin_ - dist; // refine distance
        if (dist_err > 0)
        {
          ret = true;

          double grad_l1, cost_p_dist;
          smoothedL1(dist_err, 0.05, cost_p_dist, grad_l1);
          costp += factor * wei_obs_ * cost_p_dist;
          Eigen::Vector3d curr_gradp = -wei_obs_ * factor * grad_l1 * dist_grad;
          gradp += curr_gradp;

          gradacc += (l_dis * curr_gradp.transpose() * grad_norm_negTp_).transpose();
        }
      }
      num_size++;
    }
    // check the drone
    Eigen::Vector3d posQ = pos + L_length_ * norm_negTp_;
    dist = grid_map_->getDistance(posQ);                           // coarse distance
    dist_err = drone_size_ + safe_margin_ + map_resolution - dist; // conservative checking
    if (dist_err > 0)
    {

      Eigen::Vector3d dist_grad;
      grid_map_->evaluateEDTWithGrad(posQ, dist, dist_grad);
      dist_err = drone_size_ + safe_margin_ - dist; // refine distance
      if (dist_err > 0)
      {
        ret = true;
        double grad_l1, cost_p_dist;
        smoothedL1(dist_err, 0.05, cost_p_dist, grad_l1);
        costp += wei_obs_ * cost_p_dist;
        Eigen::Vector3d curr_gradp = -wei_obs_ * grad_l1 * dist_grad;
        gradp += curr_gradp;

        gradacc += (L_length_ * curr_gradp.transpose() * grad_norm_negTp_).transpose();
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    double f = 0, df = 0;
    if (smoothedL1(vpen, 0.01, f, df))
    {
      gradv = wei_feas_ * 2.0 * df * v;
      costv = wei_feas_ * f;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    double f = 0, df = 0;
    if (smoothedL1(apen, 0.01, f, df))
    {
      grada = wei_feas_ * 2.0 * df * a;
      costa = wei_feas_ * f;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostDroneDynamic(const Eigen::Vector3d &a,
                                                          const Eigen::Vector3d &j,
                                                          const Eigen::Vector3d &s,
                                                          Eigen::Vector3d &grada,
                                                          Eigen::Vector3d &gradj,
                                                          Eigen::Vector3d &grads,
                                                          double &costd)
  {
    bool ret = false;
    double thr;
    Eigen::Vector3d zdir, zdir_grad = Eigen::Vector3d::Zero();
    flat.forward(a, j, s, thr, zdir);
    double violaThrust = (thr - thrust_mean_) * (thr - thrust_mean_) - thrust_sqr_radi_;
    double violaThrustPena, violaThrustPenaD;
    double gradThr = 0.0;
    if (smoothedL1(violaThrust, 0.01, violaThrustPena, violaThrustPenaD))
    {
      gradThr += wei_quad_feas_ * violaThrustPenaD * 2.0 * (thr - thrust_mean_);
      costd += wei_quad_feas_ * violaThrustPena;
      ret = true;
    }
    // cos_theta_max_ - e^w_z dot zdir
    double violaTilt = pow((cos_theta_max_ - zdir(2)), 3);
    double violaTiltPena, violaTiltPenaD;

    if (smoothedL1(violaTilt, 0.01, violaTiltPena, violaTiltPenaD))
    {
      zdir_grad += Eigen::Vector3d(0, 0, -1) * violaTiltPenaD * 3 * (cos_theta_max_ - zdir(2)) * (cos_theta_max_ - zdir(2));
      costd += wei_quad_feas_ * violaTiltPena;
      ret = true;
    }
    flat.backward(gradThr, zdir_grad, grada, gradj, grads);

    return ret;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps =
        ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean - dsqrmean * dsqrmean);
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1) - dsqrmean) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i) - dsqrmean) / N * dps.col(i));
      }
    }
    return;
  }

  int PolyTrajOptimizer::astarWithMinTraj(const Eigen::Matrix<double, 3, 4> &iniState,
                                          const Eigen::Matrix<double, 3, 4> &finState,
                                          vector<Eigen::Vector3d> &simple_path,
                                          Eigen::MatrixXd &ctl_points,
                                          poly_traj::MinSnapOpt &frontendMJ)
  {
    Eigen::Vector3d start_pos = iniState.col(0);
    Eigen::Vector3d end_pos = finState.col(0);
    Eigen::Vector3d start_vel = iniState.col(1);
    Eigen::Vector3d end_vel = finState.col(1);
    Eigen::Vector3d start_acc = iniState.col(2);
    // Eigen::Vector3d end_acc = finState.col(2);

    std::vector<double> t_list;
    /* astar search and get the simple path*/
    simple_path.clear();
    int status = kino_a_star_->KinoAstarSearchAndGetSimplePath(start_pos, start_vel, start_acc, end_pos, end_vel, simple_path, t_list);
    if (status == KinodynamicAstar::NO_PATH)
    {
      return status;
    }

    /* generate minimum snap trajectory based on the simple_path waypoints*/
    int piece_num = simple_path.size() - 1;
    Eigen::MatrixXd innerPts;

    if (piece_num > 1)
    {
      innerPts.resize(3, piece_num - 1);
      for (int i = 0; i < piece_num - 1; i++)
        innerPts.col(i) = simple_path[i + 1];
    }
    else
    {
      if ((simple_path[0] - simple_path[1]).norm() <= 0.1)  // Too short to ignore 
      {
        return KinodynamicAstar::TOO_SHORT;
      }
      ROS_WARN("simple path is too short");
      piece_num = 2;
      innerPts.resize(3, 1);
      innerPts.col(0) = (simple_path[0] + simple_path[1]) / 2.0;
      t_list.resize(2);
      t_list[0] = (simple_path[0] - innerPts.col(0)).norm() / max_vel_*2.5;
      t_list[1] = (simple_path[1] - innerPts.col(0)).norm() / max_vel_*2.5;
    }
    // ctl_points = innerPts;

    frontendMJ.reset(iniState, finState, piece_num);

    Eigen::Map<Eigen::VectorXd> time_vec(t_list.data(), t_list.size());
    time_vec = time_vec * 1.2;
    frontendMJ.generate(innerPts, time_vec);
    ctl_points = frontendMJ.getInitConstrainPoints(cps_num_prePiece_);
    return status;
  }

  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/constrain_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_quad_feasibility", wei_quad_feas_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);

    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);

    nh.param("optimization/payload_size", payload_size_, 0.1);
    nh.param("optimization/drone_size", drone_size_, 0.25);
    nh.param("optimization/L_length", L_length_, 0.6);
    nh.param("optimization/safe_margin", safe_margin_, 0.1);
    nh.param("optimization/sample_ball_distance", sample_ball_distance_, 0.1);
    nh.param("optimization/grav", grav_, 9.81);

    nh.param("optimization/mass_payload", mass_payload_, 0.1);
    nh.param("optimization/mass_quad", mass_quad_, 1.0);
    double max_theta;
    nh.param("optimization/max_theta", max_theta, 60.0);
    cos_theta_max_ = cos(max_theta * M_PI / 180.0);
    double max_thr, min_thr;
    nh.param("optimization/max_thrust", max_thr, 40.0);
    nh.param("optimization/min_thrust", min_thr, 0.5);

    thrust_mean_ = (max_thr + min_thr) / 2.0;
    thrust_sqr_radi_ = (max_thr - min_thr) * (max_thr - min_thr) / 4.0;
    flat.reset(mass_quad_, mass_payload_, grav_, L_length_);

    kino_a_star_.reset(new KinodynamicAstar);
    kino_a_star_->setParam(nh);
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    // should be called after setParam
    grid_map_ = map;
    kino_a_star_->setEnvironment(grid_map_);
    kino_a_star_->init();
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.resize_cp(points.cols());
    cps_.points = points;
  }

}