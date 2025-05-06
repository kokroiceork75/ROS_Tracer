#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>

#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

#include <Eigen/Dense>
#include <qpOASES.hpp>

USING_NAMESPACE_QPOASES

// 定義 MPC 參數
const int N = 10;
const double DT = 0.1;
const double MAX_V = 0.8;
const double MAX_W = 1.5;
const double MIN_V = 0.0;
const double MIN_W = -1.5;

const double Q_X = 1.0;
const double Q_Y = 1.0;
const double Q_YAW = 3.0; // 降低 yaw 權重以減少目標點附近晃動
const double R_V = 0.1;
const double R_W = 0.45;
const double P_X = 1.0;
const double P_Y = 1.0;
const double P_YAW = 1.0; // 降低終端 yaw 權重

const double GOAL_TOLERANCE = 0.15;
const float WHEEL_DISTANCE = 0.3791;

class MPCLocalPlanner {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_amcl_;
    ros::Subscriber sub_goal_;
    ros::Publisher pub_cmd_vel_;
    ros::Timer control_timer_;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double current_v_ = 0.0;
    double current_w_ = 0.0;

    nav_msgs::Path global_path_;
    bool path_received_ = false;
    bool odom_received_ = false;
    bool amcl_received_ = false;
    geometry_msgs::Pose final_goal_pose_;
    bool final_goal_received_ = false;
    bool is_at_goal_ = false;

    SQProblem qp_solver_;
    int nV_;
    int nC_;
    real_t H_[(2*N) * (2*N)];
    real_t g_[2*N];
    real_t lb_[2*N];
    real_t ub_[2*N];

    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd P_;

    bool first_run_ = true;

public:
    MPCLocalPlanner() : nh_("~"), qp_solver_(2*N, 0) {
        sub_path_ = nh_.subscribe("/move_base/NavfnROS/plan", 1, &MPCLocalPlanner::pathCallback, this);
        sub_odom_ = nh_.subscribe("/odom", 1, &MPCLocalPlanner::odomCallback, this);
        sub_amcl_ = nh_.subscribe("/move_base/feedback", 1, &MPCLocalPlanner::amclFeedbackCallback, this);
        sub_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &MPCLocalPlanner::goalCallback, this);

        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_1", 1);
        control_timer_ = nh_.createTimer(ros::Duration(DT), &MPCLocalPlanner::controlLoop, this);

        nV_ = 2 * N;
        nC_ = 0;

        Options qp_options;
        qp_options.printLevel = PL_LOW;
        qp_solver_.setOptions(qp_options);

        Q_ = Eigen::MatrixXd::Zero(3, 3);
        Q_(0, 0) = Q_X;
        Q_(1, 1) = Q_Y;
        Q_(2, 2) = Q_YAW;

        R_ = Eigen::MatrixXd::Zero(2, 2);
        R_(0, 0) = R_V;
        R_(1, 1) = R_W;

        P_ = Eigen::MatrixXd::Zero(3, 3);
        P_(0, 0) = P_X;
        P_(1, 1) = P_Y;
        P_(2, 2) = P_YAW;

        const double MAX_WHEEL_V = 1.0;
        for (int i = 0; i < N; ++i) {
            lb_[2*i] = std::max(MIN_V, -MAX_WHEEL_V + (WHEEL_DISTANCE / 2) * MIN_W);
            ub_[2*i] = std::min(MAX_V, MAX_WHEEL_V - (WHEEL_DISTANCE / 2) * MIN_W);
            lb_[2*i+1] = std::max(MIN_W, (-MAX_WHEEL_V - MIN_V) / (WHEEL_DISTANCE / 2));
            ub_[2*i+1] = std::min(MAX_W, (MAX_WHEEL_V - MIN_V) / (WHEEL_DISTANCE / 2));
        }

        ROS_INFO("MPC Local Planner initialized. Prediction Horizon N=%d, Control Period DT=%.2f s", N, DT);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) {
            ROS_WARN("Received empty global path.");
            path_received_ = false;
            return;
        }
        global_path_ = *msg;
        path_received_ = true;
        if (is_at_goal_) {
            is_at_goal_ = false;
            ROS_INFO("New path received, resetting goal state.");
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_v_ = msg->twist.twist.linear.x;
        current_w_ = msg->twist.twist.angular.z;
        odom_received_ = true;
    }

    void amclFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
        current_x_ = msg->feedback.base_position.pose.position.x;
        current_y_ = msg->feedback.base_position.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->feedback.base_position.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        current_yaw_ = normalizeAngle(current_yaw_);
        ROS_DEBUG("Current yaw: %.2f rad", current_yaw_);
        amcl_received_ = true;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        final_goal_pose_ = msg->pose;
        final_goal_received_ = true;
        is_at_goal_ = false;
        path_received_ = false;
        global_path_.poses.clear();
        ROS_INFO("Received new goal: x=%.2f, y=%.2f, resetting planner state.", 
                 final_goal_pose_.position.x, final_goal_pose_.position.y);
    }

    void controlLoop(const ros::TimerEvent& event) {
        if (!path_received_ || !odom_received_ || !amcl_received_ || global_path_.poses.empty()) {
            ROS_DEBUG_THROTTLE(1.0, "Waiting for path, odometry velocity, and AMCL pose data...");
            publishStopCommand();
            return;
        }

        if (is_at_goal_) {
            ROS_DEBUG_THROTTLE(1.0, "At goal, maintaining stop state...");
            publishStopCommand();
            return;
        }

        double dist_to_goal = std::sqrt(std::pow(current_x_ - final_goal_pose_.position.x, 2) +
                                        std::pow(current_y_ - final_goal_pose_.position.y, 2));
        if (final_goal_received_ && dist_to_goal < GOAL_TOLERANCE) {
            ROS_INFO("Final goal reached! Stopping robot.");
            publishStopCommand();
            is_at_goal_ = true;
            final_goal_received_ = false;
            path_received_ = false;
            global_path_.poses.clear();
            return;
        }

        Eigen::MatrixXd ref_trajectory = generateReferenceTrajectory();
        if (ref_trajectory.rows() == 0) {
            ROS_WARN("Failed to generate reference trajectory.");
            publishStopCommand();
            return;
        }

        ROS_DEBUG("Reference trajectory first point: x=%.2f, y=%.2f, yaw=%.2f",
                  ref_trajectory(0, 0), ref_trajectory(1, 0), ref_trajectory(2, 0));
        double yaw_diff = normalizeAngleRelative(ref_trajectory(2, 0), current_yaw_);
        if (std::abs(yaw_diff) > M_PI * 0.9) {
            ROS_WARN("Detected 180-degree yaw error: ref_yaw=%.2f, current_yaw=%.2f, diff=%.2f",
                     ref_trajectory(2, 0), current_yaw_, yaw_diff);
        }

        Eigen::Vector3d current_state(current_x_, current_y_, current_yaw_);
        Eigen::Vector2d current_control(current_v_, current_w_);
        Eigen::MatrixXd A_d(3, 3);
        Eigen::MatrixXd B_d(3, 2);
        linearizeAndDiscretize(current_state, current_control, A_d, B_d);

        if (!buildQPMatrices(current_state, A_d, B_d, ref_trajectory)) {
            ROS_ERROR("Failed to build QP matrices!");
            publishStopCommand();
            return;
        }

        real_t qp_solution[nV_];
        int nWSR = 100;
        returnValue qp_status;
        if (first_run_) {
            qp_status = qp_solver_.init(H_, g_, nullptr, lb_, ub_, nullptr, nullptr, nWSR);
            if (qp_status == SUCCESSFUL_RETURN) {
                first_run_ = false;
            }
        } else {
            qp_status = qp_solver_.hotstart(H_, g_, nullptr, lb_, ub_, nullptr, nullptr, nWSR);
        }

        if (qp_status != SUCCESSFUL_RETURN) {
            ROS_ERROR("QP solver failed! Error code: %d", static_cast<int>(qp_status));
            publishStopCommand();
            if (qp_status == RET_INIT_FAILED) {
                ROS_WARN("QP Solver initialization failed, attempting to reset.");
                qp_solver_.reset();
                first_run_ = true;
            }
            return;
        }

        qp_solver_.getPrimalSolution(qp_solution);
        double optimal_v = qp_solution[0];
        double optimal_w = qp_solution[1];

        optimal_v = std::max(MIN_V, std::min(MAX_V, optimal_v));
        optimal_w = std::max(MIN_W, std::min(MAX_W, optimal_w));

        ROS_DEBUG("Control output: v=%.2f, w=%.2f", optimal_v, optimal_w);
        if (optimal_v < 0) {
            ROS_WARN("Negative linear velocity detected: v=%.2f", optimal_v);
        }
        if (std::abs(optimal_v) > 0.01 || std::abs(optimal_w) > 0.01) {
            ROS_DEBUG("Non-zero control near goal: dist_to_goal=%.2f, v=%.2f, w=%.2f", dist_to_goal, optimal_v, optimal_w);
        }

        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = optimal_v;
        cmd_vel_msg.angular.z = optimal_w;
        pub_cmd_vel_.publish(cmd_vel_msg);
    }

    double normalizeAngleRelative(double target_angle, double current_angle) {
        double diff = target_angle - current_angle;
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff <= -M_PI) diff += 2.0 * M_PI;
        if (std::abs(diff) > M_PI * 0.9) {
            ROS_WARN("Angle diff near 180 degrees: target=%.2f, current=%.2f, diff=%.2f",
                     target_angle, current_angle, diff);
        }
        return diff;
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle <= -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    Eigen::MatrixXd generateReferenceTrajectory() {
        Eigen::MatrixXd ref_traj(3, N + 1);
        if (global_path_.poses.empty()) {
            return Eigen::MatrixXd(0, 0);
        }

        double dist_to_goal = std::sqrt(std::pow(current_x_ - final_goal_pose_.position.x, 2) +
                                        std::pow(current_y_ - final_goal_pose_.position.y, 2));
        // 若接近目標，生成靜止參考軌跡
        if (dist_to_goal < GOAL_TOLERANCE) {
            for (int k = 0; k <= N; ++k) {
                ref_traj(0, k) = current_x_;
                ref_traj(1, k) = current_y_;
                ref_traj(2, k) = current_yaw_;
            }
            ROS_DEBUG("Generating static reference trajectory near goal: x=%.2f, y=%.2f, yaw=%.2f",
                      current_x_, current_y_, current_yaw_);
            return ref_traj;
        }

        double min_dist_sq = std::numeric_limits<double>::max();
        int closest_idx = 0;
        for (int i = 0; i < global_path_.poses.size(); ++i) {
            double dx = global_path_.poses[i].pose.position.x - current_x_;
            double dy = global_path_.poses[i].pose.position.y - current_y_;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }

        double lookahead_dist = std::max(0.1, std::min(current_v_ * DT * 2.0, 0.5));
        int start_idx = findLookaheadPoint(closest_idx, lookahead_dist);

        for (int k = 0; k <= N; ++k) {
            double target_dist = k * MAX_V * DT * 0.8;
            int ref_idx = findLookaheadPoint(start_idx, target_dist);

            ref_traj(0, k) = global_path_.poses[ref_idx].pose.position.x;
            ref_traj(1, k) = global_path_.poses[ref_idx].pose.position.y;

            double ref_yaw;
            if (ref_idx + 1 < global_path_.poses.size()) {
                double next_x = global_path_.poses[ref_idx + 1].pose.position.x;
                double next_y = global_path_.poses[ref_idx + 1].pose.position.y;
                ref_yaw = std::atan2(next_y - ref_traj(1, k), next_x - ref_traj(0, k)) + M_PI;
                ref_yaw = normalizeAngle(ref_yaw);
            } else {
                tf2::Quaternion q;
                tf2::fromMsg(global_path_.poses[ref_idx].pose.orientation, q);
                tf2::Matrix3x3 m(q);
                double r, p;
                m.getRPY(r, p, ref_yaw);
                ref_yaw = normalizeAngle(ref_yaw + M_PI);
            }
            ref_traj(2, k) = current_yaw_ + normalizeAngleRelative(ref_yaw, current_yaw_);
            ROS_DEBUG("Ref point %d: x=%.2f, y=%.2f, yaw=%.2f", k, ref_traj(0, k), ref_traj(1, k), ref_traj(2, k));
        }

        if (final_goal_received_) {
            double dist_last_ref_to_goal = std::sqrt(std::pow(ref_traj(0, N) - final_goal_pose_.position.x, 2) +
                                                    std::pow(ref_traj(1, N) - final_goal_pose_.position.y, 2));
            if (dist_last_ref_to_goal < N * MAX_V * DT * 0.5) {
                ref_traj(0, N) = final_goal_pose_.position.x;
                ref_traj(1, N) = final_goal_pose_.position.y;
                tf2::Quaternion q_goal;
                tf2::fromMsg(final_goal_pose_.orientation, q_goal);
                tf2::Matrix3x3 m_goal(q_goal);
                double r_goal, p_goal, y_goal;
                m_goal.getRPY(r_goal, p_goal, y_goal);
                y_goal = normalizeAngle(y_goal + M_PI);
                ref_traj(2, N) = current_yaw_ + normalizeAngleRelative(y_goal, current_yaw_);
                ROS_DEBUG("Final goal adjusted: x=%.2f, y=%.2f, yaw=%.2f",
                          ref_traj(0, N), ref_traj(1, N), ref_traj(2, N));
            }
        }

        return ref_traj;
    }

    int findLookaheadPoint(int start_index, double target_dist) {
        double accumulated_dist = 0.0;
        int current_idx = start_index;

        if (start_index >= global_path_.poses.size() - 1) {
            return global_path_.poses.size() - 1;
        }

        for (int i = start_index; i < global_path_.poses.size() - 1; ++i) {
            double x1 = global_path_.poses[i].pose.position.x;
            double y1 = global_path_.poses[i].pose.position.y;
            double x2 = global_path_.poses[i + 1].pose.position.x;
            double y2 = global_path_.poses[i + 1].pose.position.y;
            double segment_dist = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));

            double segment_yaw = std::atan2(y2 - y1, x2 - x1);
            double yaw_diff = std::abs(normalizeAngleRelative(segment_yaw, current_yaw_));
            if (yaw_diff > M_PI / 2) {
                continue;
            }

            if (accumulated_dist + segment_dist >= target_dist) {
                current_idx = (target_dist - accumulated_dist) < (accumulated_dist + segment_dist - target_dist) ? i : i + 1;
                break;
            }
            accumulated_dist += segment_dist;
            current_idx = i + 1;
        }
        ROS_DEBUG("Lookahead point index: %d, distance: %.2f", current_idx, accumulated_dist);
        return std::min((int)global_path_.poses.size() - 1, current_idx);
    }

    void linearizeAndDiscretize(const Eigen::Vector3d& state, const Eigen::Vector2d& control,
                                Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d) {
        double yaw = state(2);
        double v = control(0);
        double w = control(1);

        Eigen::Matrix3d A_c = Eigen::Matrix3d::Zero();
        A_c(0, 2) = -v * std::sin(yaw);
        A_c(1, 2) = v * std::cos(yaw);

        Eigen::MatrixXd B_c(3, 2);
        B_c.setZero();
        B_c(0, 0) = std::cos(yaw);
        B_c(1, 0) = std::sin(yaw);
        B_c(2, 1) = 1.0;

        A_d = Eigen::Matrix3d::Identity() + A_c * DT;
        B_d = B_c * DT;

        double v_left = v - (WHEEL_DISTANCE / 2) * w;
        double v_right = v + (WHEEL_DISTANCE / 2) * w;
        const double MAX_WHEEL_V = 1.0;
        if (std::abs(v_left) > MAX_WHEEL_V || std::abs(v_right) > MAX_WHEEL_V) {
            ROS_WARN("Wheel velocity constraint violated: v_left=%.2f, v_right=%.2f", v_left, v_right);
        }
    }

    bool buildQPMatrices(const Eigen::Vector3d& current_state,
                         const Eigen::MatrixXd& A_d, const Eigen::MatrixXd& B_d,
                         const Eigen::MatrixXd& ref_trajectory) {
        Eigen::MatrixXd H_qp = Eigen::MatrixXd::Zero(nV_, nV_);
        Eigen::VectorXd g_qp = Eigen::VectorXd::Zero(nV_);

        Eigen::MatrixXd G = Eigen::MatrixXd::Zero(3 * N, nV_);
        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(3 * N, 3);

        Eigen::MatrixXd A_pow_k = Eigen::MatrixXd::Identity(3, 3);
        for (int k = 0; k < N; ++k) {
            A_pow_k = A_d * A_pow_k;
            F.block(3 * k, 0, 3, 3) = A_pow_k;

            Eigen::MatrixXd A_pow_kj = Eigen::MatrixXd::Identity(3, 3);
            for (int j = k; j >= 0; --j) {
                if (j < k) {
                    A_pow_kj = A_d * A_pow_kj;
                }
                G.block(3 * k, 2 * j, 3, 2) = A_pow_kj * B_d;
            }
        }

        Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(3 * N, 3 * N);
        Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(nV_, nV_);
        for (int i = 0; i < N; ++i) {
            if (i < N - 1) {
                Q_bar.block(3 * i, 3 * i, 3, 3) = Q_;
            } else {
                Q_bar.block(3 * i, 3 * i, 3, 3) = P_;
            }
            R_bar.block(2 * i, 2 * i, 2, 2) = R_;
        }

        Eigen::VectorXd ref_vec(3 * N);
        for (int i = 0; i < N; ++i) {
            ref_vec.segment(3 * i, 2) = ref_trajectory.block(0, i + 1, 2, 1);
            double ref_yaw = ref_trajectory(2, i + 1);
            double yaw_diff = normalizeAngleRelative(ref_yaw, current_state(2));
            ref_vec(3 * i + 2) = current_state(2) + yaw_diff;
            ROS_DEBUG("Ref yaw %d: %.2f, diff: %.2f", i, ref_yaw, yaw_diff);
        }

        Eigen::VectorXd error_term = F * current_state - ref_vec;
        for (int i = 0; i < N; ++i) {
            error_term(3 * i + 2) = normalizeAngleRelative(ref_vec(3 * i + 2), (F * current_state)(3 * i + 2));
        }

        H_qp = 2.0 * (G.transpose() * Q_bar * G + R_bar);
        g_qp = 2.0 * G.transpose() * Q_bar * error_term;

        if (H_qp.rows() != nV_ || H_qp.cols() != nV_ || g_qp.size() != nV_) {
            ROS_ERROR("QP matrix dimension mismatch! H:(%ldx%ld), g:(%ldx1), nV=%d",
                      H_qp.rows(), H_qp.cols(), g_qp.size(), nV_);
            return false;
        }

        memcpy(H_, H_qp.data(), H_qp.size() * sizeof(real_t));
        memcpy(g_, g_qp.data(), g_qp.size() * sizeof(real_t));

        return true;
    }

    void publishStopCommand() {
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        pub_cmd_vel_.publish(stop_msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpc_local_planner");
    MPCLocalPlanner mpc_planner;
    ros::spin();
    return 0;
}