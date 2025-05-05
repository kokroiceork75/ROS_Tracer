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

#include <Eigen/Dense>
#include <qpOASES.hpp>

USING_NAMESPACE_QPOASES

// 定義 MPC 參數
const int N = 10;           // 預測時域長度
const double DT = 0.1;      // 控制週期（秒）
const double MAX_V = 0.8;   // 最大線速度（m/s）
const double MAX_W = 1.5;   // 最大角速度（rad/s）
const double MIN_V = 0.0;   // 最小線速度
const double MIN_W = -1.5;  // 最小角速度

const double Q_X = 1.0;     // X 位置誤差權重
const double Q_Y = 1.0;     // Y 位置誤差權重
const double Q_YAW = 0.5;   // Yaw 角度誤差權重
const double R_V = 0.1;     // 線速度控制權重
const double R_W = 0.05;    // 角速度控制權重
const double P_X = 1.0;     // 終端 X 誤差權重
const double P_Y = 1.0;     // 終端 Y 誤差權重
const double P_YAW = 0.5;   // 終端 Yaw 誤差權重

const double GOAL_TOLERANCE = 0.15; // 目標點容忍距離（m）
const float WHEEL_DISTANCE = 0.3791; // 兩輪間距（m）

class MPCLocalPlanner {
private:
    ros::NodeHandle nh_;         // ROS 節點句柄
    ros::Subscriber sub_path_;   // 訂閱全局路徑
    ros::Subscriber sub_odom_;   // 訂閱里程計，用於獲取速度
    ros::Subscriber sub_amcl_;   // 訂閱 AMCL 反饋，用於獲取姿態
    ros::Subscriber sub_goal_;   // 訂閱最終目標點
    ros::Publisher pub_cmd_vel_; // 發布速度指令
    ros::Timer control_timer_;   // 控制迴圈計時器

    // 機器人狀態變數
    double current_x_ = 0.0;     // 當前 X 座標
    double current_y_ = 0.0;     // 當前 Y 座標
    double current_yaw_ = 0.0;   // 當前 Yaw 角度（弧度）
    double current_v_ = 0.0;     // 當前線速度
    double current_w_ = 0.0;     // 當前角速度

    nav_msgs::Path global_path_; // 儲存全局路徑
    bool path_received_ = false;  // 是否收到路徑
    bool odom_received_ = false;  // 是否收到里程計速度
    bool amcl_received_ = false;  // 是否收到 AMCL 姿態
    geometry_msgs::Pose final_goal_pose_; // 最終目標姿態
    bool final_goal_received_ = false;    // 是否收到最終目標

    // MPC 和 qpOASES 相關變數
    SQProblem qp_solver_;        // QP 求解器
    int nV_;                     // QP 優化變數數量（2*N）
    int nC_;                     // QP 約束數量（目前為 0）
    real_t H_[(2*N) * (2*N)];    // QP Hessian 矩陣
    real_t g_[2*N];              // QP 梯度向量
    real_t lb_[2*N];             // 優化變數下界
    real_t ub_[2*N];             // 優化變數上界

    Eigen::MatrixXd Q_;          // 狀態權重矩陣
    Eigen::MatrixXd R_;          // 控制權重矩陣
    Eigen::MatrixXd P_;          // 終端狀態權重矩陣

    bool first_run_ = true;      // 首次運行標誌，用於 QP 初始化

public:
    // 建構函數：初始化 ROS 節點和 MPC 參數
    MPCLocalPlanner() : nh_("~"), qp_solver_(2*N, 0) {
        // 初始化 ROS 訂閱與發布
        sub_path_ = nh_.subscribe("/move_base/NavfnROS/plan", 1, &MPCLocalPlanner::pathCallback, this);
        sub_odom_ = nh_.subscribe("/odom", 1, &MPCLocalPlanner::odomCallback, this);
        sub_amcl_ = nh_.subscribe("/move_base/feedback", 1, &MPCLocalPlanner::amclFeedbackCallback, this);
        sub_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &MPCLocalPlanner::goalCallback, this);

        pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_1", 1);
        control_timer_ = nh_.createTimer(ros::Duration(DT), &MPCLocalPlanner::controlLoop, this);

        // 初始化 MPC 參數
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

        // 設定控制輸入上下界
        for (int i = 0; i < N; ++i) {
            lb_[2*i] = MIN_V;
            ub_[2*i] = MAX_V;
            lb_[2*i+1] = MIN_W;
            ub_[2*i+1] = MAX_W;
        }

        ROS_INFO("MPC Local Planner initialized. Prediction Horizon N=%d, Control Period DT=%.2f s", N, DT);
    }

    // 處理全局路徑回調
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) {
            ROS_WARN("Received empty global path.");
            path_received_ = false;
            return;
        }
        global_path_ = *msg;
        path_received_ = true;
    }

    // 處理里程計回調，僅更新速度
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_v_ = msg->twist.twist.linear.x;
        current_w_ = msg->twist.twist.angular.z;
        odom_received_ = true;
    }

    // 處理 AMCL 反饋回調，更新姿態
    void amclFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {
        current_x_ = msg->feedback.base_position.pose.position.x;
        current_y_ = msg->feedback.base_position.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->feedback.base_position.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        amcl_received_ = true;
    }

    // 處理最終目標回調
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        final_goal_pose_ = msg->pose;
        final_goal_received_ = true;
        ROS_INFO("Received final goal: x=%.2f, y=%.2f", final_goal_pose_.position.x, final_goal_pose_.position.y);
    }

    // 主控制迴圈
    void controlLoop(const ros::TimerEvent& event) {
        // 檢查是否收到路徑、速度和姿態數據
        if (!path_received_ || !odom_received_ || !amcl_received_ || global_path_.poses.empty()) {
            ROS_DEBUG_THROTTLE(1.0, "Waiting for path, odometry velocity, and AMCL pose data...");
            publishStopCommand();
            return;
        }

        // 生成參考軌跡
        Eigen::MatrixXd ref_trajectory = generateReferenceTrajectory();
        if (ref_trajectory.rows() == 0) {
            ROS_WARN("Failed to generate reference trajectory.");
            publishStopCommand();
            return;
        }

        // 檢查是否到達最終目標
        double dist_to_goal = std::sqrt(std::pow(current_x_ - final_goal_pose_.position.x, 2) +
                                        std::pow(current_y_ - final_goal_pose_.position.y, 2));
        if (final_goal_received_ && dist_to_goal < GOAL_TOLERANCE) {
            ROS_INFO("Final goal reached!");
            publishStopCommand();
            control_timer_.stop();
            return;
        }

        // 準備 MPC 計算
        Eigen::Vector3d current_state(current_x_, current_y_, current_yaw_);
        Eigen::Vector2d current_control(current_v_, current_w_);
        Eigen::MatrixXd A_d(3, 3);
        Eigen::MatrixXd B_d(3, 2);
        linearizeAndDiscretize(current_state, current_control, A_d, B_d);

        // 構建 QP 問題
        if (!buildQPMatrices(current_state, A_d, B_d, ref_trajectory)) {
            ROS_ERROR("Failed to build QP matrices!");
            publishStopCommand();
            return;
        }

        // 求解 QP 問題
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

        // 檢查 QP 求解結果
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

        // 獲取並應用最優控制
        qp_solver_.getPrimalSolution(qp_solution);
        double optimal_v = qp_solution[0];
        double optimal_w = qp_solution[1];

        optimal_v = std::max(MIN_V, std::min(MAX_V, optimal_v));
        optimal_w = std::max(MIN_W, std::min(MAX_W, optimal_w));

        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = optimal_v;
        cmd_vel_msg.angular.z = optimal_w;
        pub_cmd_vel_.publish(cmd_vel_msg);
    }

    // 規範化角度至 [-π, π]
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle <= -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // 生成參考軌跡
    Eigen::MatrixXd generateReferenceTrajectory() {
        Eigen::MatrixXd ref_traj(3, N + 1);
        if (global_path_.poses.empty()) {
            return Eigen::MatrixXd(0, 0);
        }

        // 尋找距離當前位置最近的路徑點
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

        // 計算前瞻距離並選擇起始點
        double lookahead_dist = std::max(0.1, current_v_ * DT * 1.5);
        int start_idx = findLookaheadPoint(closest_idx, lookahead_dist);

        // 生成 N+1 個參考點
        for (int k = 0; k <= N; ++k) {
            double target_dist = k * MAX_V * DT * 0.8;
            int ref_idx = findLookaheadPoint(start_idx, target_dist);

            ref_traj(0, k) = global_path_.poses[ref_idx].pose.position.x;
            ref_traj(1, k) = global_path_.poses[ref_idx].pose.position.y;

            double ref_yaw;
            if (ref_idx + 1 < global_path_.poses.size()) {
                double next_x = global_path_.poses[ref_idx + 1].pose.position.x;
                double next_y = global_path_.poses[ref_idx + 1].pose.position.y;
                ref_yaw = std::atan2(next_y - ref_traj(1, k), next_x - ref_traj(0, k));
            } else {
                tf2::Quaternion q;
                tf2::fromMsg(global_path_.poses[ref_idx].pose.orientation, q);
                tf2::Matrix3x3 m(q);
                double r, p;
                m.getRPY(r, p, ref_yaw);
            }
            ref_traj(2, k) = normalizeAngle(ref_yaw);
        }

        // 若接近最終目標，調整末端參考點
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
                ref_traj(2, N) = normalizeAngle(y_goal);
            }
        }

        return ref_traj;
    }

    // 尋找前瞻點索引
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

            if (accumulated_dist + segment_dist >= target_dist) {
                if ((target_dist - accumulated_dist) < (accumulated_dist + segment_dist - target_dist)) {
                    current_idx = i;
                } else {
                    current_idx = i + 1;
                }
                break;
            }
            accumulated_dist += segment_dist;
            current_idx = i + 1;
        }
        return std::min((int)global_path_.poses.size() - 1, current_idx);
    }

    // 線性化和離散化運動學模型
    void linearizeAndDiscretize(const Eigen::Vector3d& state, const Eigen::Vector2d& control,
                                Eigen::MatrixXd& A_d, Eigen::MatrixXd& B_d) {
        double yaw = state(2);
        double v = control(0);

        // 連續時間 A_c 矩陣
        Eigen::Matrix3d A_c = Eigen::Matrix3d::Zero();
        A_c(0, 2) = -v * std::sin(yaw);
        A_c(1, 2) = v * std::cos(yaw);

        // 連續時間 B_c 矩陣
        Eigen::MatrixXd B_c(3, 2);
        B_c(0, 0) = std::cos(yaw);
        B_c(1, 0) = std::sin(yaw);
        B_c(2, 1) = 1.0;
        B_c(0, 1) = 0.0;
        B_c(1, 1) = 0.0;
        B_c(2, 0) = 0.0;

        // 歐拉離散化
        A_d = Eigen::Matrix3d::Identity() + A_c * DT;
        B_d = B_c * DT;
    }

    // 構建 QP 問題的 Hessian 和梯度
    bool buildQPMatrices(const Eigen::Vector3d& current_state,
                         const Eigen::MatrixXd& A_d, const Eigen::MatrixXd& B_d,
                         const Eigen::MatrixXd& ref_trajectory) {
        Eigen::MatrixXd H_qp = Eigen::MatrixXd::Zero(nV_, nV_);
        Eigen::VectorXd g_qp = Eigen::VectorXd::Zero(nV_);

        // 構建狀態預測矩陣
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

        // 構建權重矩陣
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

        // 計算 QP 矩陣
        H_qp = 2.0 * (G.transpose() * Q_bar * G + R_bar);
        Eigen::VectorXd ref_vec(3 * N);
        for (int i = 0; i < N; ++i) {
            ref_vec.segment(3 * i, 3) = ref_trajectory.col(i + 1);
        }

        Eigen::VectorXd error_term = F * current_state - ref_vec;
        g_qp = 2.0 * G.transpose() * Q_bar * error_term;

        // 檢查矩陣維度
        if (H_qp.rows() != nV_ || H_qp.cols() != nV_ || g_qp.size() != nV_) {
            ROS_ERROR("QP matrix dimension mismatch! H:(%ldx%ld), g:(%ldx1), nV=%d",
                      H_qp.rows(), H_qp.cols(), g_qp.size(), nV_);
            return false;
        }

        // 複製數據至 qpOASES 格式
        memcpy(H_, H_qp.data(), H_qp.size() * sizeof(real_t));
        memcpy(g_, g_qp.data(), g_qp.size() * sizeof(real_t));

        return true;
    }

    // 發布停止指令
    void publishStopCommand() {
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        pub_cmd_vel_.publish(stop_msg);
    }
};

int main(int argc, char **argv) {
    // 初始化 ROS 節點
    ros::init(argc, argv, "mpc_local_planner");
    MPCLocalPlanner mpc_planner;
    ros::spin();
    return 0;
}