#ifndef FLATNESS_HPP
#define FLATNESS_HPP

#include <Eigen/Eigen>

#include <cmath>

namespace flatness
{
    class PayloadFlatnessMap
    {
    public:
        inline void reset(const double &quad_mass,
                          const double &load_mass,
                          const double &gravitational_acceleration,
                          const double &length)
        {
            massQ = quad_mass;
            massL = load_mass;
            grav = gravitational_acceleration;
            l_length = length;
        }

        void forward(const Eigen::Vector3d &load_acc,
                            const Eigen::Vector3d &load_jer,
                            const Eigen::Vector3d &load_snap,
                            double &thr, Eigen::Vector3d &z)
        {

            q0 = load_acc(0);
            q1 = load_acc(1);
            q2 = load_acc(2) + grav;

            qd0 = load_jer(0);
            qd1 = load_jer(1);
            qd2 = load_jer(2);

            qdd0 = load_snap(0);
            qdd1 = load_snap(1);
            qdd2 = load_snap(2);

            q_dot_q = q0 * q0 + q1 * q1 + q2 * q2;
            q_norm = sqrt(q_dot_q);

            q_norm_3 = q_dot_q * q_norm;
            q_norm_5 = q_dot_q * q_norm_3;
            q_dot_qd = q0 * qd0 + q1 * qd1 + q2 * qd2;
            q_dot_qdd = q0 * qdd0 + q1 * qdd1 + q2 * qdd2;
            qd_dot_qd = qd0 * qd0 + qd1 * qd1 + qd2 * qd2;

            sqr_q_dot_qd = q_dot_qd * q_dot_qd;

            qnormdd0 = (3 * (sqr_q_dot_qd)*q0 - q_dot_q * ((q_dot_qdd + qd_dot_qd) * q0 + 2 * q_dot_qd * qd0)) / q_norm_5 + qdd0 / q_norm;
            qnormdd1 = (3 * (sqr_q_dot_qd)*q1 - q_dot_q * ((q_dot_qdd + qd_dot_qd) * q1 + 2 * q_dot_qd * qd1)) / q_norm_5 + qdd1 / q_norm;
            qnormdd2 = (3 * (sqr_q_dot_qd)*q2 - q_dot_q * ((q_dot_qdd + qd_dot_qd) * q2 + 2 * q_dot_qd * qd2)) / q_norm_5 + qdd2 / q_norm;

            zu0 = (massQ + massL) * load_acc(0) + massQ * l_length * qnormdd0;
            zu1 = (massQ + massL) * load_acc(1) + massQ * l_length * qnormdd1;
            zu2 = (massQ + massL) * (load_acc(2) + grav) + massQ * l_length * qnormdd2;

            zu_sqr_norm = zu0 * zu0 + zu1 * zu1 + zu2 * zu2;
            zu_norm = sqrt(zu_sqr_norm);

            z0 = zu0 / zu_norm;
            z1 = zu1 / zu_norm;
            z2 = zu2 / zu_norm;

            z(0) = z0;
            z(1) = z1;
            z(2) = z2;
            thr = zu_norm;
            return;
        }

        void backward(const double &thr_grad,
                             const Eigen::Vector3d &z_grad,
                             Eigen::Vector3d &load_acc_grad,
                             Eigen::Vector3d &load_jer_grad,
                             Eigen::Vector3d &load_snap_grad) const
        {
            double z0b = z_grad(0);
            double z1b = z_grad(1);
            double z2b = z_grad(2);

            double q0b = 0.0;
            double q1b = 0.0;
            double q2b = 0.0;
            double qd0b = 0.0;
            double qd1b = 0.0;
            double qd2b = 0.0;
            double qdd0b = 0.0;
            double qdd1b = 0.0;
            double qdd2b = 0.0;
            double q_dot_qb = 0.0;
            double q_norm_3b = 0.0;
            double q_norm_5b = 0.0;
            double q_dot_qdb = 0.0;
            double q_dot_qddb = 0.0;
            double qd_dot_qdb = 0.0;
            double sqr_q_dot_qdb = 0.0;
            double qnormdd0b = 0.0;
            double qnormdd1b = 0.0;
            double qnormdd2b = 0.0;
            double zu0b = 0.0;
            double zu1b = 0.0;
            double zu2b = 0.0;
            double zu_sqr_normb = 0.0;
            double zu_normb = thr_grad;
            double temp;
            double tempb;
            double temp0;
            double tempb0;
            double q_normb;
            zu_sqr_normb = (zu_sqr_norm == 0.0 ? 0.0 : zu_normb / (2.0 * sqrt(zu_sqr_norm)));
            zu2b = z2b / zu_norm + 2 * zu2 * zu_sqr_normb;
            zu_normb = -(zu2 * (z2b) / (zu_norm * zu_norm));
            zu1b = z1b / zu_norm + 2 * zu1 * zu_sqr_normb;
            zu_normb = -(zu1 * (z1b) / (zu_norm * zu_norm));
            zu0b = z0b / zu_norm + 2 * zu0 * zu_sqr_normb;
            zu_normb = -(zu0 * (z0b) / (zu_norm * zu_norm));
            qnormdd2b = massQ * l_length * zu2b;
            qnormdd1b = massQ * l_length * zu1b;
            qnormdd0b = massQ * l_length * zu0b;
            temp = (q_dot_qdd + qd_dot_qd) * q2 + 2 * q_dot_qd * qd2;
            tempb0 = qnormdd2b / q_norm_5;
            sqr_q_dot_qdb = 3 * q2 * tempb0;
            q_dot_qb = -(temp * tempb0);
            tempb = -(q_dot_q * tempb0);
            q2b = 3 * sqr_q_dot_qd * tempb0 + (q_dot_qdd + qd_dot_qd) * tempb;
            q_norm_5b = -((3 * sqr_q_dot_qd * q2 - q_dot_q * temp) * tempb0 / q_norm_5);
            q_dot_qddb = q2 * tempb;
            qd_dot_qdb = q2 * tempb;
            q_dot_qdb = 2 * qd2 * tempb;
            qd2b = 2 * q_dot_qd * tempb;
            temp = (q_dot_qdd + qd_dot_qd) * q1 + 2 * q_dot_qd * qd1;
            tempb0 = qnormdd1b / q_norm_5;
            tempb = -(q_dot_q * tempb0);
            q1b = 3 * sqr_q_dot_qd * tempb0 + (q_dot_qdd + qd_dot_qd) * tempb;
            q_dot_qddb = q_dot_qddb + q1 * tempb;
            qd_dot_qdb = qd_dot_qdb + q1 * tempb;
            q_dot_qdb = q_dot_qdb + 2 * qd1 * tempb;
            qd1b = 2 * q_dot_qd * tempb;
            temp0 = (q_dot_qdd + qd_dot_qd) * q0 + 2 * q_dot_qd * qd0;
            tempb = qnormdd0b / q_norm_5;
            sqr_q_dot_qdb = sqr_q_dot_qdb + 3 * q1 * tempb0 + 3 * q0 * tempb;
            q_norm_5b = q_norm_5b - (3 * sqr_q_dot_qd * q1 - q_dot_q * temp) * tempb0 / q_norm_5 -
                        (3 * sqr_q_dot_qd * q0 - q_dot_q * temp0) * tempb / q_norm_5;
            q_norm_3b = q_dot_q * q_norm_5b;
            q_normb = q_dot_q * q_norm_3b - qdd2 * qnormdd2b / (q_norm * q_norm) - qdd1 * qnormdd1b / (q_norm * q_norm) - qdd0 * qnormdd0b / (q_norm * q_norm);
            if (q_dot_q == 0.0)
                q_dot_qb = q_dot_qb + q_norm_3 * q_norm_5b - temp * tempb0 - temp0 * tempb +
                           q_norm * q_norm_3b;
            else
                q_dot_qb = q_dot_qb + q_norm_3 * q_norm_5b - temp * tempb0 - temp0 * tempb +
                           q_norm * q_norm_3b + q_normb / (2.0 * sqrt(q_dot_q));
            tempb0 = -(q_dot_q * tempb);
            q_dot_qddb = q_dot_qddb + q0 * tempb0;
            qdd2b = qnormdd2b / q_norm + q2 * q_dot_qddb;
            qdd1b = qnormdd1b / q_norm + q1 * q_dot_qddb;
            qdd0b = qnormdd0b / q_norm + q0 * q_dot_qddb;
            qd_dot_qdb = qd_dot_qdb + q0 * tempb0;
            q_dot_qdb = q_dot_qdb + 2 * qd0 * tempb0 + 2 * q_dot_qd * sqr_q_dot_qdb;
            q0b = 3 * sqr_q_dot_qd * tempb + (q_dot_qdd + qd_dot_qd) * tempb0 + qdd0 * q_dot_qddb + qd0 * q_dot_qdb + 2 * q0 * q_dot_qb;
            double load_acc0b = (massQ + massL) * zu0b + q0b;
            qd0b = 2 * q_dot_qd * tempb0 + 2 * qd0 * qd_dot_qdb + q0 * q_dot_qdb;
            qd1b = qd1b + 2 * qd1 * qd_dot_qdb + q1 * q_dot_qdb;
            qd2b = qd2b + 2 * qd2 * qd_dot_qdb + q2 * q_dot_qdb;
            q1b = q1b + qdd1 * q_dot_qddb + qd1 * q_dot_qdb + 2 * q1 * q_dot_qb;
            double load_acc1b = (massQ + massL) * zu1b + q1b;
            q2b = q2b + qdd2 * q_dot_qddb + qd2 * q_dot_qdb + 2 * q2 * q_dot_qb;
            double load_acc2b = (massQ + massL) * zu2b + q2b;
            double load_snap2b = qdd2b;
            double load_snap1b = qdd1b;
            double load_snap0b = qdd0b;
            double load_jer2b = qd2b;
            double load_jer1b = qd1b;
            double load_jer0b = qd0b;

            load_acc_grad << load_acc0b, load_acc1b, load_acc2b;
            load_jer_grad << load_jer0b, load_jer1b, load_jer2b;
            load_snap_grad << load_snap0b, load_snap1b, load_snap2b;

            return;
        }

    private:
        // Parameters
        double massQ, massL, grav, eps, l_length;
        // forward params
        double q0 = 0, qd0 = 0, qdd0 = 0;
        double q1 = 0, qd1 = 0, qdd1 = 0;
        double q2 = 0, qd2 = 0, qdd2 = 0;
        double q_dot_q;
        double q_norm, q_norm_3, q_norm_5;
        double q_dot_qd;
        double q_dot_qdd;
        double qd_dot_qd;
        double sqr_q_dot_qd;
        double qnormdd0, qnormdd1, qnormdd2;
        double zu0;
        double zu1;
        double zu2;
        double zu_sqr_norm;
        double zu_norm;
        double z0;
        double z1;
        double z2;

    };
}

#endif
