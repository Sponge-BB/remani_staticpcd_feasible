#include <plan_manage/remani_replan_fsm.h>

#include <cmath>
#include <limits>

namespace remani_planner
{
  REMANIReplanFSM::~REMANIReplanFSM(){}
  void REMANIReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    try_plan_after_emergency_ = false;
    flag_relan_astar_ = false;
    have_local_traj_ = false;
    replan_fail_time_ = 0;
    new_traj_fail_time_ = 0;

    /*  fsm param  */
    nh.param("fsm/target_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0);
    nh.param("fsm/replan_feedback_position_threshold", replan_feedback_position_threshold_, 0.08);
    nh.param("fsm/replan_feedback_yaw_threshold", replan_feedback_yaw_threshold_, 0.15);
    nh.param("fsm/replan_feedback_arm_threshold", replan_feedback_arm_threshold_, 0.10);
    nh.param("fsm/goal_position_tolerance", goal_position_tolerance_, 0.10);
    nh.param("fsm/goal_yaw_tolerance", goal_yaw_tolerance_, 0.15);
    nh.param("fsm/goal_arm_tolerance", goal_arm_tolerance_, 0.12);
    nh.param("fsm/actual_state_collision_check_interval",
             actual_state_collision_check_interval_, 0.05);
    nh.param("fsm/global_plan_dynamic_lookahead_time", global_plan_dynamic_lookahead_time_, 2.0);
    nh.param("fsm/global_plan_dynamic_check_interval", global_plan_dynamic_check_interval_, 0.1);
    nh.param("fsm/global_plan_dynamic_check_time_step", global_plan_dynamic_check_time_step_, 0.05);
    nh.param("fsm/global_plan_static_check_time_step", global_plan_static_check_time_step_, 0.1);
    nh.param("fsm/global_plan_dynamic_lookahead_distance", global_plan_dynamic_lookahead_distance_, 2.0);
    nh.param("fsm/global_plan_dynamic_far_sample_distance", global_plan_dynamic_far_sample_distance_, 0.1);
    nh.param("fsm/global_plan_dynamic_immediate_confirm_distance", global_plan_dynamic_immediate_confirm_distance_, 0.8);
    nh.param("fsm/global_plan_dynamic_confirm_frames", global_plan_dynamic_confirm_frames_, 2);
    nh.param("fsm/global_plan_dynamic_confirm_timeout", global_plan_dynamic_confirm_timeout_, 0.6);
    nh.param("fsm/global_plan_dynamic_clear_hold_time", global_plan_dynamic_clear_hold_time_, 0.4);
    nh.param("fsm/global_plan_dynamic_replan_hold_time", global_plan_dynamic_replan_hold_time_, 0.6);
    nh.param("fsm/global_plan_dynamic_same_obstacle_radius", global_plan_dynamic_same_obstacle_radius_, 0.45);
    nh.param("fsm/global_plan_dynamic_retry_interval", global_plan_dynamic_retry_interval_, 0.3);
    nh.param("fsm/global_plan_dynamic_slowdown_distance", global_plan_dynamic_slowdown_distance_, 2.0);
    nh.param("fsm/global_plan_dynamic_slowdown_min_speed", global_plan_dynamic_slowdown_min_speed_, 0.08);
    nh.param("fsm/global_plan_dynamic_slowdown_max_speed", global_plan_dynamic_slowdown_max_speed_, 0.20);
    nh.param("fsm/global_plan_dynamic_emergency_deceleration", global_plan_dynamic_emergency_deceleration_, 0.35);
    nh.param("fsm/global_plan_dynamic_emergency_reaction_time", global_plan_dynamic_emergency_reaction_time_, 0.15);
    nh.param("fsm/global_plan_dynamic_emergency_margin", global_plan_dynamic_emergency_margin_, 0.05);
    nh.param("fsm/global_plan_dynamic_emergency_min_distance", global_plan_dynamic_emergency_min_distance_, 0.15);
    nh.param("fsm/global_plan_dynamic_emergency_max_distance", global_plan_dynamic_emergency_max_distance_, 0.22);
    nh.param("fsm/controller_progress_timeout", controller_progress_timeout_, 0.5);
    nh.param("fsm/global_plan_max_failures", global_plan_max_failures_, 3);
    nh.param("fsm/global_plan_trial_times", global_plan_trial_times_, 3);
    nh.param("fsm/rear_goal_plan_max_failures", rear_goal_plan_max_failures_, 3);
    nh.param("fsm/rear_goal_plan_trial_times", rear_goal_plan_trial_times_, 3);
    nh.param("fsm/rear_goal_plan_distance", rear_goal_plan_distance_, 0.5);
    nh.param("fsm/rear_goal_plan_cos", rear_goal_plan_cos_, -0.2);
    nh.param("fsm/time_for_gripper", time_for_gripper_, -1.0);
    nh.param("fsm/global_plan", global_plan_, false);
    if(global_plan_) planning_horizen_ = 1.0e3;

    global_plan_dynamic_lookahead_time_ = std::max(0.0, global_plan_dynamic_lookahead_time_);
    global_plan_dynamic_check_interval_ = std::max(0.01, global_plan_dynamic_check_interval_);
    global_plan_dynamic_check_time_step_ = std::max(0.01, global_plan_dynamic_check_time_step_);
    global_plan_static_check_time_step_ = std::max(0.01, global_plan_static_check_time_step_);
    global_plan_dynamic_lookahead_distance_ = std::max(0.0, global_plan_dynamic_lookahead_distance_);
    global_plan_dynamic_far_sample_distance_ = std::max(0.02, global_plan_dynamic_far_sample_distance_);
    global_plan_dynamic_immediate_confirm_distance_ = std::max(0.0, global_plan_dynamic_immediate_confirm_distance_);
    global_plan_dynamic_confirm_frames_ = std::max(1, global_plan_dynamic_confirm_frames_);
    global_plan_dynamic_confirm_timeout_ = std::max(
        global_plan_dynamic_check_interval_, global_plan_dynamic_confirm_timeout_);
    global_plan_dynamic_clear_hold_time_ = std::max(0.0, global_plan_dynamic_clear_hold_time_);
    global_plan_dynamic_replan_hold_time_ = std::max(0.0, global_plan_dynamic_replan_hold_time_);
    global_plan_dynamic_same_obstacle_radius_ = std::max(0.0, global_plan_dynamic_same_obstacle_radius_);
    global_plan_dynamic_retry_interval_ = std::max(0.05, global_plan_dynamic_retry_interval_);
    global_plan_dynamic_slowdown_distance_ = std::max(0.0, global_plan_dynamic_slowdown_distance_);
    global_plan_dynamic_slowdown_min_speed_ = std::max(0.0, global_plan_dynamic_slowdown_min_speed_);
    global_plan_dynamic_slowdown_max_speed_ = std::max(global_plan_dynamic_slowdown_min_speed_,
                                                       global_plan_dynamic_slowdown_max_speed_);
    global_plan_dynamic_emergency_deceleration_ = std::max(0.05, global_plan_dynamic_emergency_deceleration_);
    global_plan_dynamic_emergency_reaction_time_ = std::max(0.0, global_plan_dynamic_emergency_reaction_time_);
    global_plan_dynamic_emergency_margin_ = std::max(0.0, global_plan_dynamic_emergency_margin_);
    global_plan_dynamic_emergency_min_distance_ = std::max(0.0, global_plan_dynamic_emergency_min_distance_);
    global_plan_dynamic_emergency_max_distance_ = std::max(global_plan_dynamic_emergency_min_distance_,
                                                           global_plan_dynamic_emergency_max_distance_);
    controller_progress_timeout_ = std::max(0.1, controller_progress_timeout_);
    rear_goal_plan_max_failures_ = std::max(1, rear_goal_plan_max_failures_);
    rear_goal_plan_trial_times_ = std::max(1, rear_goal_plan_trial_times_);
    rear_goal_plan_distance_ = std::max(0.0, rear_goal_plan_distance_);
    rear_goal_plan_cos_ = std::max(-1.0, std::min(1.0, rear_goal_plan_cos_));
    replan_feedback_position_threshold_ = std::max(0.0, replan_feedback_position_threshold_);
    replan_feedback_yaw_threshold_ = std::max(0.0, replan_feedback_yaw_threshold_);
    replan_feedback_arm_threshold_ = std::max(0.0, replan_feedback_arm_threshold_);
    goal_position_tolerance_ = std::max(0.01, goal_position_tolerance_);
    goal_yaw_tolerance_ = std::max(0.01, goal_yaw_tolerance_);
    goal_arm_tolerance_ = std::max(0.01, goal_arm_tolerance_);
    actual_state_collision_check_interval_ =
        std::max(0.01, actual_state_collision_check_interval_);
    last_global_plan_dynamic_check_time_ = ros::Time(0);
    last_actual_state_collision_check_time_ = ros::Time(0);
    last_dynamic_collision_time_ = ros::Time(0);
    last_dynamic_seen_time_ = ros::Time(0);
    dynamic_replan_hold_until_ = ros::Time(0);
    next_dynamic_replan_retry_time_ = ros::Time(0);
    last_dynamic_observation_seq_ = 0;
    dynamic_collision_confirm_count_ = 0;
    current_safety_speed_limit_ = std::numeric_limits<double>::quiet_NaN();
    last_dynamic_confirmation_pos_.setZero();
    last_dynamic_replan_collision_pos_.setZero();
    last_dynamic_replan_collision_type_ = -1;
    has_last_dynamic_confirmation_pos_ = false;
    has_last_dynamic_replan_collision_ = false;
    dynamic_safety_speed_limit_active_ = false;
    dynamic_replan_retry_pending_ = false;
    have_controller_progress_ = false;
    tracking_fault_active_ = false;
    controller_trajectory_start_time_ = 0.0;
    controller_trajectory_time_ = 0.0;
    controller_trajectory_scale_ = 1.0;
    controller_progress_stamp_ = ros::Time(0);

    nh.param("mm/mobile_base_dof", mobile_base_dim_, -1);
    nh.param("mm/manipulator_dof", manipulator_dim_, -1);
    nh.param("mm/mobile_base_non_singul_vel", mobile_base_non_singul_vel_, -1.0);

    // Default arm stand-by comes from fsm/init_state. If that parameter is
    // missing, fall back to the first joint_state message.
    default_arm_from_config_ = false;
    has_default_arm_ = false;
    default_arm_config_ = Eigen::VectorXd::Zero(manipulator_dim_);
    std::vector<double> init_state_cfg;
    if (nh.getParam("fsm/init_state", init_state_cfg) &&
        init_state_cfg.size() >= static_cast<size_t>(mobile_base_dim_ + manipulator_dim_)) {
      for (int i = 0; i < manipulator_dim_; ++i) {
        default_arm_config_(i) = init_state_cfg[mobile_base_dim_ + i] * M_PI / 180.0;
      }
      has_default_arm_ = true;
      default_arm_from_config_ = true;
      ROS_INFO_STREAM("[FSM] Arm stand-by pose from fsm/init_state: "
                      << default_arm_config_.transpose());
    } else {
      ROS_WARN("[FSM] fsm/init_state is missing or too short; arm stand-by will use first joint_state.");
    }
    nh.param("fsm/manual_restore_standby_after_goal", manual_restore_standby_after_goal_, true);
    nh.param("fsm/manual_restore_tolerance", manual_restore_tolerance_, 0.05);
    nh.param("fsm/manual_use_work_arm_waypoint", manual_use_work_arm_waypoint_, false);
    nh.param("fsm/manual_target_end_arm_standby", manual_target_end_arm_standby_, true);
    nh.param("fsm/manual_work_arm_waypoint_id", manual_work_arm_waypoint_id_, 0);
    manual_restore_pending_ = false;
    manual_restore_running_ = false;
    manual_restore_tolerance_ = std::max(0.0, manual_restore_tolerance_);
    

    traj_dim_ = mobile_base_dim_ + manipulator_dim_;

    mm_state_pos_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_vel_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_acc_ = Eigen::VectorXd::Zero(traj_dim_);

    gripper_flag_ = true;

    start_pos_.resize(traj_dim_);
    start_vel_.resize(traj_dim_);
    start_acc_.resize(traj_dim_);
    start_jer_.resize(traj_dim_);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);

    waypoints_.clear();
    waypoints_yaw_.clear();
    Eigen::VectorXd wp = Eigen::VectorXd::Zero(traj_dim_);
    double yaw_temp;
    bool gripper_close;
    for (int i = 0; i < waypoint_num_; i++){
      nh.param("fsm/waypoint" + to_string(i) + "_yaw", yaw_temp, -1.0);
      waypoints_yaw_.push_back(yaw_temp * M_PI / 180.0);

      nh.param("fsm/waypoint" + to_string(i) + "_gripper_close", gripper_close, true);
      waypoint_gripper_close_.push_back(gripper_close);

      std::vector<double> waypoints_temp;
      nh.getParam("fsm/waypoint" + to_string(i), waypoints_temp);
      for(unsigned int j = 0; j < waypoints_temp.size(); j++){
        wp(j) = waypoints_temp[j];
        if((int)j >= mobile_base_dim_) wp(j) = wp(j) * M_PI / 180.0;
      }
      waypoints_.push_back(wp);
    }
    
    init_time_list_.clear();
    opt_time_list_.clear();
    total_time_list_.clear();

    rcv_gripper_state_ = false;
    gripper_state_ = false;
    map_state_ = 0;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new MMPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &REMANIReplanFSM::mmCarOdomCallback, this);
    joint_state_sub_ = nh.subscribe("joint_state", 1, &REMANIReplanFSM::mmManiOdomCallback, this);
    gripper_state_sub_ = nh.subscribe("gripper_state", 1, &REMANIReplanFSM::gripperCallback, this);
    trajectory_progress_sub_ = nh.subscribe(
        "/mm_controller_node/trajectory_progress", 1,
        &REMANIReplanFSM::trajectoryProgressCallback, this);
    tracking_fault_sub_ = nh.subscribe(
        "/tracking_fault", 1, &REMANIReplanFSM::trackingFaultCallback, this);

    poly_traj_pub_ = nh.advertise<quadrotor_msgs::PolynomialTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    gripper_cmd_pub_ = nh.advertise<std_msgs::Bool>("gripper_cmd", 100);
    map_state_pub_ = nh.advertise<std_msgs::Int32>("/map_generator/map_state", 100);

    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);
    safety_speed_limit_pub_ = nh.advertise<std_msgs::Float64>("/planning/safety_speed_limit", 1, true);
    publishSafetySpeedLimit(-1.0);
    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &REMANIReplanFSM::waypointCallback, this);
    
  }

  void REMANIReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100){
      fsm_num = 0;
      // printFSMExecState();
    }

    switch (exec_state_){
    case INIT:
    {
      if (!have_odom_){
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return; // return;
      else{
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      if(try_plan_after_emergency_){
        std::cout << "emergency stop mm pos: " << mm_state_pos_.transpose() << std::endl;
        std::cout << "emergency stop mm vel: " << mm_state_vel_.transpose() << std::endl;
        std::cout << "emergency stop mm acc: " << mm_state_acc_.transpose() << std::endl;
        std::cout << "emergency stop mm yaw: " << mm_car_yaw_ << std::endl;
      }
      // std::cout << "gen new traj 1\n";
      have_local_traj_ = false;
      const bool rear_distant_goal = isRearDistantGoal();
      const int trial_times = rear_distant_goal
          ? rear_goal_plan_trial_times_
          : global_plan_trial_times_;
      const int max_failures = rear_distant_goal
          ? rear_goal_plan_max_failures_
          : global_plan_max_failures_;
      bool success = planFromGlobalTraj(trial_times);
      // std::cout << "gen new traj 2\n";
      if (success){
        new_traj_fail_time_ = 0;
        if (dynamic_safety_speed_limit_active_)
          clearDynamicSafetySpeedLimit(ros::Time::now(), true);
        else
          publishSafetySpeedLimit(-1.0);
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        try_plan_after_emergency_ = false;
      }
      else
      {
        ++new_traj_fail_time_;
        ROS_WARN("[FSM] Failed to generate new trajectory (%d/%d).",
                 new_traj_fail_time_, max_failures);
        if (new_traj_fail_time_ >= max_failures)
        {
          new_traj_fail_time_ = 0;
          have_target_ = false;
          have_new_target_ = false;
          have_local_traj_ = false;
          manual_restore_pending_ = false;
          manual_restore_running_ = false;
          ROS_ERROR("[FSM] Give up current target and return to WAIT_TARGET. Please send a new goal.");
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (dynamic_replan_retry_pending_ &&
          ros::Time::now() < next_dynamic_replan_retry_time_) {
        goto force_return;
      }

      if(planFromLocalTraj(flag_relan_astar_)){
        replan_fail_time_ = 0;
        flag_relan_astar_ = false;
        if (dynamic_replan_retry_pending_) {
          dynamic_replan_retry_pending_ = false;
          dynamic_replan_hold_until_ =
              ros::Time::now() + ros::Duration(global_plan_dynamic_replan_hold_time_);
        }
        if((ros::Time::now() - t_last_Astar_ ).toSec() > 1.0){
          std::cout << "cal front end next time" << std::endl;
          flag_relan_astar_ = true;
          t_last_Astar_ = ros::Time::now();
        }
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else{
        replan_fail_time_++;
        flag_relan_astar_ = true;
        t_last_Astar_ = ros::Time::now();
        if (dynamic_replan_retry_pending_) {
          next_dynamic_replan_retry_time_ =
              ros::Time::now() + ros::Duration(global_plan_dynamic_retry_interval_);
        }
        if(replan_fail_time_ >= 20){
          replan_fail_time_ = 0;
          dynamic_replan_retry_pending_ = false;
          ROS_ERROR("[FSM]:REPLAN fail over 20 times!!!");
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else{
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      break;
    }
    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
      const ros::Time now = ros::Time::now();
      double t_cur = getCurrentTrajectoryTime(*info, now);
      const bool trajectory_finished = t_cur >= info->duration - 1e-2;
      bool need_to_plan_next = trajectory_finished;
      bool need_to_gripper = trajectory_finished;
      t_cur = min(info->duration, t_cur);

      Eigen::VectorXd pos = info->getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);
      bool close_to_no_replan_thresh = ((end_pt_ - pos).head(2).norm() < no_replan_thresh_);
      const double actual_goal_position_error =
          (mm_state_pos_.head(2) - end_pt_.head(2)).norm();
      const double actual_goal_yaw_error =
          fabs(atan2(sin(mm_car_yaw_ - end_yaw_), cos(mm_car_yaw_ - end_yaw_)));
      const double actual_goal_arm_error =
          (mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_) -
           end_pt_.segment(mobile_base_dim_, manipulator_dim_)).cwiseAbs().maxCoeff();
      const bool actual_at_goal =
          actual_goal_position_error <= goal_position_tolerance_ &&
          actual_goal_yaw_error <= goal_yaw_tolerance_ &&
          actual_goal_arm_error <= goal_arm_tolerance_;

      if((target_type_ == TARGET_TYPE::PRESET_TARGET) && close_to_no_replan_thresh){
        if((wpt_id_ < waypoint_num_ - 1) && need_to_plan_next){
          ++wpt_id_;
          planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
          gripper_flag_ = true;
        }else if(need_to_gripper && gripper_flag_){
          ++map_state_;
          std_msgs::Bool gripper_cmd;
          gripper_cmd.data = waypoint_gripper_close_[wpt_id_];
          gripper_cmd_pub_.publish(gripper_cmd);
          std::string gripper_cmd_str = waypoint_gripper_close_[wpt_id_] ? "close gripper" : "open gripper";
          ROS_INFO(gripper_cmd_str.c_str());
          std_msgs::Int32 map_state;
          map_state.data = map_state_;
          gripper_flag_ = false;
        }
      }else if(target_type_ == TARGET_TYPE::MANUAL_TARGET && trajectory_finished &&
               touch_the_goal && actual_at_goal){
        const double arm_standby_error = has_default_arm_ ?
            (mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_) - default_arm_config_).norm() : 0.0;
        const bool need_manual_restore =
            manual_restore_standby_after_goal_ && has_default_arm_ && !manual_restore_running_ &&
            arm_standby_error > manual_restore_tolerance_;

        if(need_manual_restore){
          manual_restore_pending_ = false;
          manual_restore_running_ = true;

          Eigen::VectorXd restore_pt = end_pt_;
          restore_pt.segment(mobile_base_dim_, manipulator_dim_) = default_arm_config_;
          const double restore_yaw = end_yaw_;

          ROS_INFO("[FSM] Manual target reached. Planning arm restore to stand-by pose. arm_error=%.4f",
                   arm_standby_error);
          if(planNextWaypoint(restore_pt, restore_yaw)){
            goto force_return;
          }

          manual_restore_running_ = false;
          ROS_WARN("[FSM] Failed to plan arm restore trajectory. Mark manual target as finished.");
          have_target_ = false;
          have_trigger_ = false;
          std::cout << "reach goal\n";
          changeFSMExecState(WAIT_TARGET, "FSM");
          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }else{
          manual_restore_running_ = false;
          have_target_ = false;
          have_trigger_ = false;
          std::cout << "reach goal\n";
          changeFSMExecState(WAIT_TARGET, "FSM");
          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }
      }else if(trajectory_finished && touch_the_goal && actual_at_goal){
        if(target_type_ != TARGET_TYPE::PRESET_TARGET && wpt_id_ >= waypoint_num_ - 1){
          have_target_ = false;
          have_trigger_ = false;
          std::cout << "reach goal\n";
          changeFSMExecState(WAIT_TARGET, "FSM");
          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }
      }else if(trajectory_finished && touch_the_goal && !actual_at_goal){
        ROS_WARN_THROTTLE(
            1.0,
            "[FSM] Reference trajectory ended before the real robot reached the goal "
            "(base=%.3f m, yaw=%.3f rad, arm=%.3f rad); replan from feedback.",
            actual_goal_position_error, actual_goal_yaw_error, actual_goal_arm_error);
        have_local_traj_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "GOAL_FEEDBACK");
        goto force_return;
      }else if(!close_to_no_replan_thresh && t_cur > replan_thresh_ && (!global_plan_)){
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }
    case EMERGENCY_STOP:
    {
      if(flag_escape_emergency_){ // Avoiding repeated calls
        callEmergencyStop(mm_state_pos_, mm_car_yaw_, mm_car_singul_);
      }
      else{
        if(enable_fail_safe_ && mm_state_vel_.head(2).norm() < 0.1){
          try_plan_after_emergency_ = true;
          have_local_traj_ = false;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
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

  void REMANIReplanFSM::trajectoryProgressCallback(
      const geometry_msgs::Vector3Stamped::ConstPtr &msg)
  {
    controller_trajectory_start_time_ = msg->vector.x;
    controller_trajectory_time_ = std::max(0.0, msg->vector.y);
    controller_trajectory_scale_ =
        std::max(0.0, std::min(1.0, msg->vector.z));
    controller_progress_stamp_ = msg->header.stamp;
    have_controller_progress_ = controller_trajectory_start_time_ > 0.0;
  }

  void REMANIReplanFSM::trackingFaultCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    if (!msg->data) {
      tracking_fault_active_ = false;
      return;
    }
    if (tracking_fault_active_)
      return;

    tracking_fault_active_ = true;
    publishSafetySpeedLimit(0.0);
    if (have_target_ && exec_state_ != EMERGENCY_STOP) {
      ROS_ERROR("[FSM] Tracker reported excessive trajectory error; abort old trajectory and replan from odometry.");
      changeFSMExecState(EMERGENCY_STOP, "TRACKING");
    }
  }

  double REMANIReplanFSM::getCurrentTrajectoryTime(
      const SingulTrajData &traj, const ros::Time &now) const
  {
    const bool progress_matches =
        have_controller_progress_ &&
        std::fabs(controller_trajectory_start_time_ - traj.start_time) < 1e-3 &&
        !controller_progress_stamp_.isZero();
    if (progress_matches) {
      const double age = std::max(0.0, (now - controller_progress_stamp_).toSec());
      if (age <= controller_progress_timeout_) {
        return std::max(
            0.0, std::min(
                traj.duration,
                controller_trajectory_time_ + age * controller_trajectory_scale_));
      }
    }

    return std::max(
        0.0, std::min(traj.duration, now.toSec() - traj.start_time));
  }

  double REMANIReplanFSM::getCurrentTrajectoryScale(
      const SingulTrajData &traj, const ros::Time &now) const
  {
    const bool progress_matches =
        have_controller_progress_ &&
        std::fabs(controller_trajectory_start_time_ - traj.start_time) < 1e-3 &&
        !controller_progress_stamp_.isZero() &&
        (now - controller_progress_stamp_).toSec() <= controller_progress_timeout_;
    return progress_matches ? controller_trajectory_scale_ : 1.0;
  }

  void REMANIReplanFSM::publishSafetySpeedLimit(double speed_limit)
  {
    const double normalized_limit = speed_limit < 0.0 ? -1.0 : std::max(0.0, speed_limit);
    if (std::isfinite(current_safety_speed_limit_) &&
        std::fabs(current_safety_speed_limit_ - normalized_limit) < 1e-3)
      return;

    std_msgs::Float64 msg;
    msg.data = normalized_limit;
    safety_speed_limit_pub_.publish(msg);
    current_safety_speed_limit_ = normalized_limit;
  }

  void REMANIReplanFSM::applyDynamicSafetySpeedLimit(double speed_limit)
  {
    double limited_speed = std::max(0.0, speed_limit);
    if (dynamic_safety_speed_limit_active_ &&
        std::isfinite(current_safety_speed_limit_) &&
        current_safety_speed_limit_ >= 0.0) {
      limited_speed = std::min(limited_speed, current_safety_speed_limit_);
    }

    dynamic_safety_speed_limit_active_ = true;
    publishSafetySpeedLimit(limited_speed);
  }

  void REMANIReplanFSM::clearDynamicSafetySpeedLimit(const ros::Time &now, bool force)
  {
    if (!dynamic_safety_speed_limit_active_)
      return;
    if (!force &&
        (exec_state_ == EMERGENCY_STOP || last_dynamic_seen_time_.isZero() ||
         (now - last_dynamic_seen_time_).toSec() < global_plan_dynamic_clear_hold_time_)) {
      return;
    }

    dynamic_safety_speed_limit_active_ = false;
    if (force)
      dynamic_replan_retry_pending_ = false;
    dynamic_collision_confirm_count_ = 0;
    last_dynamic_collision_time_ = ros::Time(0);
    last_dynamic_seen_time_ = ros::Time(0);
    has_last_dynamic_confirmation_pos_ = false;
    has_last_dynamic_replan_collision_ = false;
    publishSafetySpeedLimit(-1.0);
  }

  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    auto map = planner_manager_->grid_map_;
    const ros::Time now = ros::Time::now();

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0){
      clearDynamicSafetySpeedLimit(now, true);
      publishSafetySpeedLimit(-1.0);
      return;
    }
    if (exec_state_ == EMERGENCY_STOP)
      return;
    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout()){
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      dynamic_safety_speed_limit_active_ = false;
      publishSafetySpeedLimit(0.0);
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      return;
    }
    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = getCurrentTrajectoryTime(*info, now);
    Eigen::VectorXd p_cur = info->getPos(t_cur);
    double t_temp = 0;
    bool occ = false;
    bool collision_is_dynamic = false;
    double dynamic_collision_path_distance = std::numeric_limits<double>::infinity();
    Eigen::Vector2d dynamic_collision_position = Eigen::Vector2d::Zero();
    const double base_speed = mm_state_vel_.head(2).norm();
    const double calculated_stop_distance =
        base_speed * base_speed / (2.0 * global_plan_dynamic_emergency_deceleration_) +
        base_speed * global_plan_dynamic_emergency_reaction_time_ +
        global_plan_dynamic_emergency_margin_;
    const double dynamic_emergency_distance = std::min(
        global_plan_dynamic_emergency_max_distance_,
        std::max(global_plan_dynamic_emergency_min_distance_, calculated_stop_distance));
    int coll_type;

    if (global_plan_) {
      const bool time_for_actual_state_check =
          last_actual_state_collision_check_time_.isZero() ||
          (now - last_actual_state_collision_check_time_).toSec() >=
              actual_state_collision_check_interval_;
      if (has_default_arm_ && time_for_actual_state_check){
        last_actual_state_collision_check_time_ = now;
        int actual_coll_type = -1;
        if (planner_manager_->mm_config_->checkcollision(
                Eigen::Vector3d(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_),
                mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_),
                false, actual_coll_type)){
          ROS_ERROR_THROTTLE(
              1.0,
              "[FSM] Real whole-body state is in collision (type=%d); stop before using nominal trajectory.",
              actual_coll_type);
          dynamic_replan_retry_pending_ = false;
          publishSafetySpeedLimit(0.0);
          changeFSMExecState(EMERGENCY_STOP, "ACTUAL_STATE");
          return;
        }
      }

      // Keep current-pose checking at 100 Hz for emergency reaction.
      if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t_cur, coll_type)){
        if(coll_type == 0){
          ROS_WARN_THROTTLE(2.0, "car collision at current time %f!", t_cur);
        }else if (coll_type == 1){
          ROS_WARN_THROTTLE(2.0, "mani collision at current time %f!", t_cur);
        }else if (coll_type == 2){
          ROS_WARN_THROTTLE(2.0, "car-mani collision at current time %f!", t_cur);
        }else if (coll_type == 3){
          ROS_WARN_THROTTLE(2.0, "mani-mani collision at current time %f!", t_cur);
        }
        occ = true;
        t_temp = t_cur;
      }

      // Scan the future trajectory against both the persistent PCD ESDF and
      // the live LiDAR layer. The latter is kept separate because its direct
      // occupancy query reacts before the ESDF update completes.
      const bool time_for_future_check = last_global_plan_dynamic_check_time_.isZero() ||
                                         (now - last_global_plan_dynamic_check_time_).toSec() >=
                                             global_plan_dynamic_check_interval_;
      if (!occ && global_plan_dynamic_lookahead_time_ > 0.0 && time_for_future_check){
        last_global_plan_dynamic_check_time_ = now;
        const std::uint64_t dynamic_observation_seq = map->getDynamicObservationSeq();
        const bool has_new_dynamic_observation =
            dynamic_observation_seq != last_dynamic_observation_seq_;
        const double time_check_end = std::min(info->duration, t_cur + global_plan_dynamic_lookahead_time_);
        double static_collision_time = std::numeric_limits<double>::infinity();
        double dynamic_collision_time = std::numeric_limits<double>::infinity();
        int static_coll_type = -1;
        int dynamic_coll_type = -1;

        for (double t_check = t_cur + global_plan_static_check_time_step_; t_check <= time_check_end;
             t_check += global_plan_static_check_time_step_){
          if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t_check, static_coll_type)){
            static_collision_time = t_check;
            break;
          }
        }

        Eigen::Vector2d previous_dynamic_position = info->getPos(t_cur).head(2);
        double scanned_path_distance = 0.0;
        double next_far_sample_distance = 0.0;
        for (double t_check = t_cur + global_plan_dynamic_check_time_step_; t_check <= info->duration;
             t_check += global_plan_dynamic_check_time_step_){
          const Eigen::Vector2d dynamic_position = info->getPos(t_check).head(2);
          scanned_path_distance += (dynamic_position - previous_dynamic_position).norm();
          previous_dynamic_position = dynamic_position;

          const bool within_time_horizon = t_check <= time_check_end;
          const bool far_sample_due = scanned_path_distance >= next_far_sample_distance;
          if (within_time_horizon || far_sample_due){
            if (!within_time_horizon)
              next_far_sample_distance = scanned_path_distance + global_plan_dynamic_far_sample_distance_;
            if (planner_manager_->ploy_traj_opt_->checkDynamicObstacleCollision(*info, t_check, dynamic_coll_type)){
              dynamic_collision_time = t_check;
              dynamic_collision_path_distance = scanned_path_distance;
              dynamic_collision_position = dynamic_position;
              break;
            }
          }

          if (!within_time_horizon && scanned_path_distance >= global_plan_dynamic_lookahead_distance_)
            break;
        }

        const bool has_static_collision = static_collision_time < std::numeric_limits<double>::infinity();
        const bool has_dynamic_collision = dynamic_collision_time < std::numeric_limits<double>::infinity();
        bool confirmed_dynamic_collision = false;
        if (has_dynamic_collision) {
          const int required_confirm_frames =
              dynamic_collision_path_distance <= global_plan_dynamic_immediate_confirm_distance_ ?
                  1 : global_plan_dynamic_confirm_frames_;

          if (has_new_dynamic_observation) {
            const bool confirmation_is_fresh =
                !last_dynamic_collision_time_.isZero() &&
                (now - last_dynamic_collision_time_).toSec() <= global_plan_dynamic_confirm_timeout_;
            const bool confirmation_is_same_obstacle =
                !has_last_dynamic_confirmation_pos_ ||
                (dynamic_collision_position - last_dynamic_confirmation_pos_).norm() <=
                    global_plan_dynamic_same_obstacle_radius_;

            if (confirmation_is_fresh && confirmation_is_same_obstacle)
              ++dynamic_collision_confirm_count_;
            else
              dynamic_collision_confirm_count_ = 1;

            last_dynamic_collision_time_ = now;
            last_dynamic_seen_time_ = now;
            last_dynamic_confirmation_pos_ = dynamic_collision_position;
            has_last_dynamic_confirmation_pos_ = true;
            last_dynamic_observation_seq_ = dynamic_observation_seq;
          }

          confirmed_dynamic_collision = dynamic_collision_path_distance <= dynamic_emergency_distance ||
                                        required_confirm_frames == 1 ||
                                        dynamic_collision_confirm_count_ >= required_confirm_frames;

          if (dynamic_collision_path_distance <= global_plan_dynamic_slowdown_distance_){
            const double slowdown_range = std::max(
                1e-3, global_plan_dynamic_slowdown_distance_ - dynamic_emergency_distance);
            const double slowdown_ratio = std::max(
                0.0, std::min(1.0,
                              (dynamic_collision_path_distance - dynamic_emergency_distance) / slowdown_range));
            const double speed_limit = global_plan_dynamic_slowdown_min_speed_ +
                slowdown_ratio * (global_plan_dynamic_slowdown_max_speed_ -
                                  global_plan_dynamic_slowdown_min_speed_);
            applyDynamicSafetySpeedLimit(
                dynamic_collision_path_distance <= dynamic_emergency_distance ? 0.0 : speed_limit);
          }

          if (!confirmed_dynamic_collision) {
            ROS_INFO_THROTTLE(
                1.0,
                "[FSM] Dynamic collision awaiting distinct map frames (%d/%d, map_seq=%lu).",
                dynamic_collision_confirm_count_, required_confirm_frames,
                static_cast<unsigned long>(dynamic_observation_seq));
          }
        } else {
          if (has_new_dynamic_observation) {
            last_dynamic_observation_seq_ = dynamic_observation_seq;
            if (!last_dynamic_collision_time_.isZero() &&
                (now - last_dynamic_collision_time_).toSec() > global_plan_dynamic_confirm_timeout_) {
              dynamic_collision_confirm_count_ = 0;
              last_dynamic_collision_time_ = ros::Time(0);
              has_last_dynamic_confirmation_pos_ = false;
            }
          }
          clearDynamicSafetySpeedLimit(now);
          if (dynamic_replan_retry_pending_ && !dynamic_safety_speed_limit_active_ &&
              exec_state_ == REPLAN_TRAJ && have_controller_progress_) {
            dynamic_replan_retry_pending_ = false;
            changeFSMExecState(EXEC_TRAJ, "SAFETY_CLEAR");
          }
        }

        if (has_static_collision || confirmed_dynamic_collision){
          const bool use_static_collision = has_static_collision &&
                                            (!confirmed_dynamic_collision || static_collision_time <= dynamic_collision_time);
          t_temp = use_static_collision ? static_collision_time : dynamic_collision_time;
          coll_type = use_static_collision ? static_coll_type : dynamic_coll_type;
          collision_is_dynamic = !use_static_collision;
          occ = true;
          if (use_static_collision){
            ROS_WARN_THROTTLE(1.0, "[FSM] Future map collision type=%d detected %.2f s ahead.",
                              coll_type, t_temp - t_cur);
          } else {
            ROS_WARN_THROTTLE(1.0,
                              "[FSM] Future dynamic collision type=%d detected %.2f m / %.2f s ahead.",
                              coll_type, dynamic_collision_path_distance, t_temp - t_cur);
          }
        }
      }
    } else {
      // Local reactive mode: scan the entire remaining trajectory for collision.
      for (double t = t_cur; t < info->duration; t += time_step){

        if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t, coll_type)){
          if(coll_type == 0){
            ROS_WARN_THROTTLE(2.0, "car collision at relative time %f!", t / info->duration);
          }else if (coll_type == 1){
            ROS_WARN_THROTTLE(2.0, "mani collision at relative time %f!", t / info->duration);
          }else if (coll_type == 2){
            ROS_WARN_THROTTLE(2.0, "car-mani collision at relative time %f!", t / info->duration);
          }else if (coll_type == 3){
            ROS_WARN_THROTTLE(2.0, "mani-mani collision at relative time %f!", t / info->duration);
          }

          t_temp = t;
          occ = true;
          break;
        }
      }
    }

    if (occ){
      /* Handle the collided case immediately */
      if (collision_is_dynamic && dynamic_collision_path_distance <= dynamic_emergency_distance){
        ROS_ERROR_THROTTLE(1.0,
                           "[FSM] Dynamic obstacle only %.2f m ahead (stop threshold %.2f m); stop before replanning.",
                           dynamic_collision_path_distance, dynamic_emergency_distance);
        publishSafetySpeedLimit(0.0);
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        return;
      }

      if (collision_is_dynamic && has_last_dynamic_replan_collision_ &&
          last_dynamic_replan_collision_type_ == coll_type &&
          (dynamic_collision_position - last_dynamic_replan_collision_pos_).norm() <=
              global_plan_dynamic_same_obstacle_radius_ &&
          now < dynamic_replan_hold_until_) {
        ROS_INFO_THROTTLE(
            1.0,
            "[FSM] Keep executing the new trajectory while the same dynamic event settles (%.2f s).",
            (dynamic_replan_hold_until_ - now).toSec());
        return;
      }

      ROS_INFO("Try to replan a safe trajectory");
      if (planFromLocalTraj(false, !collision_is_dynamic)){ // Make a chance
        dynamic_collision_confirm_count_ = 0;
        last_dynamic_collision_time_ = ros::Time(0);
        if (collision_is_dynamic) {
          last_dynamic_replan_collision_pos_ = dynamic_collision_position;
          last_dynamic_replan_collision_type_ = coll_type;
          has_last_dynamic_replan_collision_ = true;
          dynamic_replan_hold_until_ =
              ros::Time::now() + ros::Duration(global_plan_dynamic_replan_hold_time_);
          dynamic_replan_retry_pending_ = false;
        }
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }else{
        if (!collision_is_dynamic && t_temp - t_cur < emergency_time_){
          ROS_WARN_THROTTLE(2.0, "Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }else if (collision_is_dynamic){
          last_dynamic_replan_collision_pos_ = dynamic_collision_position;
          last_dynamic_replan_collision_type_ = coll_type;
          has_last_dynamic_replan_collision_ = true;
          dynamic_replan_retry_pending_ = true;
          next_dynamic_replan_retry_time_ =
              ros::Time::now() + ros::Duration(global_plan_dynamic_retry_interval_);
          dynamic_replan_hold_until_ = next_dynamic_replan_retry_time_;
          ROS_WARN("[FSM] Fast dynamic replan failed; keep the speed limit and retry after %.2f s.",
                   global_plan_dynamic_retry_interval_);
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }else{
          ROS_WARN("current traj in collision, replan.");
          if(planFromLocalTraj(true))
          {
            ROS_INFO("Plan success when detect collision.");
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }

  bool REMANIReplanFSM::planNextWaypoint(const Eigen::VectorXd next_wp, const double next_yaw)
  {
    std::vector<Eigen::VectorXd> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    bool success = planner_manager_->planGlobalTrajWaypoints(
        mm_state_pos_, mm_car_yaw_, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_),
        one_pt_wps, next_yaw, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_));

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;
      end_yaw_ = next_yaw;
      have_local_traj_ = false;
      start_singul_ = 0;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_container_.global_traj.duration / step_size_t);
      vector<Eigen::Vector2d> global_traj(i_end);
      for (int i = 0; i < i_end; i++){
        global_traj[i] = planner_manager_->traj_container_.global_traj.traj.getPos(i * step_size_t).head(mobile_base_dim_);
      }

      have_target_ = true;
      have_new_target_ = true;
      new_traj_fail_time_ = 0;

      if (exec_state_ != INIT)
      {
        // A new goal should interrupt an unreachable old goal immediately.
        // Waiting for EXEC_TRAJ here can deadlock while GEN_NEW_TRAJ is failing.
        if (exec_state_ == GEN_NEW_TRAJ)
          continously_called_times_ = 0;
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGoalPoint(end_pt_.head(2), Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalTraj(global_traj, 0.05, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
      have_target_ = false;
      have_new_target_ = false;
      have_local_traj_ = false;
      new_traj_fail_time_ = 0;
      if (exec_state_ != INIT)
        changeFSMExecState(WAIT_TARGET, "TRIG");
    }

    return success;
  }

  // manual waypoint
  void REMANIReplanFSM::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    clearDynamicSafetySpeedLimit(ros::Time::now(), true);
    last_dynamic_observation_seq_ = 0;
    dynamic_replan_retry_pending_ = false;
    dynamic_collision_confirm_count_ = 0;
    last_dynamic_collision_time_ = ros::Time(0);
    last_dynamic_seen_time_ = ros::Time(0);
    has_last_dynamic_confirmation_pos_ = false;
    has_last_dynamic_replan_collision_ = false;
    
    if (target_type_ == TARGET_TYPE::PRESET_TARGET){
      have_trigger_ = true;
      cout << "Triggered! traget type: " << target_type_ << endl;

      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      planner_manager_->global_start_time_ = ros::Time::now();
      planner_manager_->start_flag_ = true;
      start_pub_.publish(flag_msg);
      wpt_id_ = 0;
      planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
      return;
    }

    if(msg->pose.position.z < -0.1)
      return;
    cout << "Triggered! traget type: " << target_type_ << endl;
    // trigger_ = true;
    init_state_ = mm_state_pos_;
    end_pt_ = mm_state_pos_; // Start with current state (keeps car yaw)
    manual_restore_pending_ = false;
    manual_restore_running_ = false;

    if(target_type_ == TARGET_TYPE::MANUAL_TARGET){
      end_pt_(0) = msg->pose.position.x;
      end_pt_(1) = msg->pose.position.y;
      end_yaw_ = tf::getYaw(msg->pose.orientation);

      const bool valid_work_wp =
          manual_work_arm_waypoint_id_ >= 0 && manual_work_arm_waypoint_id_ < waypoint_num_;
      if (manual_use_work_arm_waypoint_ && valid_work_wp) {
        wpt_id_ = manual_work_arm_waypoint_id_;
        end_pt_.segment(mobile_base_dim_, manipulator_dim_) =
            waypoints_[manual_work_arm_waypoint_id_].segment(mobile_base_dim_, manipulator_dim_);
        manual_restore_pending_ = manual_restore_standby_after_goal_ && has_default_arm_ &&
            (end_pt_.segment(mobile_base_dim_, manipulator_dim_) - default_arm_config_).norm() > 1e-3;
        ROS_INFO("[FSM] Manual target uses arm pose from waypoint%d. Restore stand-by after goal: %s",
                 manual_work_arm_waypoint_id_, manual_restore_pending_ ? "true" : "false");
      } else {
        wpt_id_ = std::max(0, waypoint_num_ - 1);
        if(manual_target_end_arm_standby_ && has_default_arm_){
          end_pt_.segment(mobile_base_dim_, manipulator_dim_) = default_arm_config_;
          manual_restore_pending_ = false;
          ROS_INFO("[FSM] Manual target ends with configured stand-by arm pose.");
        }else{
          manual_restore_pending_ = manual_restore_standby_after_goal_ && has_default_arm_ &&
              (end_pt_.segment(mobile_base_dim_, manipulator_dim_) - default_arm_config_).norm() > 1e-3;
          ROS_INFO("[FSM] Manual target keeps current arm pose. Restore stand-by after goal: %s",
                   manual_restore_pending_ ? "true" : "false");
        }
      }
    }else{
      ROS_ERROR("wrong target type: %d", target_type_);
      return;
    }

    planNextWaypoint(end_pt_, end_yaw_);
  }

  void REMANIReplanFSM::mmCarOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // std::cout << "odom: " << mm_state_pos_.transpose() << "\n";
    mm_state_pos_(0) = msg->pose.pose.position.x;
    mm_state_pos_(1) = msg->pose.pose.position.y;
    mm_car_yaw_ = tf::getYaw(msg->pose.pose.orientation);

    mm_car_orient_.w() = msg->pose.pose.orientation.w;
    mm_car_orient_.x() = msg->pose.pose.orientation.x;
    mm_car_orient_.y() = msg->pose.pose.orientation.y;
    mm_car_orient_.z() = msg->pose.pose.orientation.z;

    const double body_vx = msg->twist.twist.linear.x;
    const double body_vy = msg->twist.twist.linear.y;
    mm_state_vel_(0) = cos(mm_car_yaw_) * body_vx - sin(mm_car_yaw_) * body_vy;
    mm_state_vel_(1) = sin(mm_car_yaw_) * body_vx + cos(mm_car_yaw_) * body_vy;
    if(mm_state_vel_.head(2).norm() < mobile_base_non_singul_vel_){
      mm_state_vel_(0) = mobile_base_non_singul_vel_ * cos(mm_car_yaw_);
      mm_state_vel_(1) = mobile_base_non_singul_vel_ * sin(mm_car_yaw_);
      mm_car_singul_ = 0;
    }else{
      Eigen::Vector2d car_head(cos(mm_car_yaw_), sin(mm_car_yaw_));
      mm_car_singul_ = car_head.dot(mm_state_vel_.head(2)) >= 0.0 ? 1 : -1;
    }

    mm_car_yaw_rate_ = msg->twist.twist.angular.z;

    have_odom_ = true;
  }

  void REMANIReplanFSM::mmManiOdomCallback(const sensor_msgs::JointStateConstPtr &msg){
    for(int i = 0; i < manipulator_dim_; ++i){
      if(i < msg->position.size()){
        mm_state_pos_(mobile_base_dim_ + i) = msg->position[i];
      }
      if(i < msg->velocity.size()){
        mm_state_vel_(mobile_base_dim_ + i) = msg->velocity[i];
      }else{
        mm_state_vel_(mobile_base_dim_ + i) = 0.0;
      }
      if(i < msg->effort.size()){
        mm_state_acc_(mobile_base_dim_ + i) = msg->effort[i];
      }else{
        mm_state_acc_(mobile_base_dim_ + i) = 0.0;
      }
    }
    // Fallback only: keep the configured stand-by pose if fsm/init_state exists.
    if (!has_default_arm_) {
      has_default_arm_ = true;
      default_arm_config_ = mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_);
      default_arm_from_config_ = false;
      ROS_INFO_STREAM("[FSM] Arm stand-by pose captured from first joint_state: "
                      << default_arm_config_.transpose());
    }
  }

  void REMANIReplanFSM::gripperCallback(const std_msgs::Bool::ConstPtr &msg){
    if(gripper_state_ != msg->data || (!rcv_gripper_state_)){
      rcv_gripper_state_ = true;
      gripper_state_ = msg->data;
      planner_manager_->mm_config_->setGripperPoint(gripper_state_);
    }
  }

  void REMANIReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void REMANIReplanFSM::printFSMExecState(){
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
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
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << string(dot_nums, '.') << endl;

    fflush(stdout);
  }

  std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> REMANIReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void REMANIReplanFSM::sendPolyTrajROSMsg(){
    auto data = &planner_manager_->traj_container_.singul_traj_data;
    
    for(unsigned int i = 0; i < data->singul_traj.size(); ++i){
      quadrotor_msgs::PolynomialTraj msg;
      msg.trajectory_id = data->singul_traj[i].traj_id;
      msg.header.stamp = ros::Time(data->start_time);
      msg.action = msg.ACTION_ADD;
      msg.singul = data->singul_traj[i].singul;
      int piece_num = data->singul_traj[i].traj.getPieceNum();
      for (int j = 0; j < piece_num; ++j)
      {
        quadrotor_msgs::PolynomialMatrix piece;
        piece.num_dim = data->singul_traj[i].traj.getPiece(j).getDim();
        piece.num_order = data->singul_traj[i].traj.getPiece(j).getDegree();
        piece.duration = data->singul_traj[i].traj.getPiece(j).getDuration();
        auto cMat = data->singul_traj[i].traj.getPiece(j).getCoeffMat();
        piece.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
        msg.trajectory.emplace_back(piece);
      }
      poly_traj_pub_.publish(msg);
    }

  }

  bool REMANIReplanFSM::isRearDistantGoal() const
  {
    if (end_pt_.size() < 2 || mm_state_pos_.size() < 2)
      return false;
    const Eigen::Vector2d offset =
        end_pt_.head(2) - mm_state_pos_.head(2);
    const double distance = offset.norm();
    if (distance < rear_goal_plan_distance_ || distance < 1e-6)
      return false;
    const Eigen::Vector2d heading(cos(mm_car_yaw_), sin(mm_car_yaw_));
    return heading.dot(offset / distance) <= rear_goal_plan_cos_;
  }

  bool REMANIReplanFSM::planFromGlobalTraj(const int trial_times /*= 1*/){
    start_pos_ = mm_state_pos_;
    start_vel_ = mm_state_vel_;
    start_acc_.setZero();
    start_jer_.setZero();
    start_yaw_ = mm_car_yaw_;
    start_singul_ = mm_car_singul_;
    bool flag_random_poly_init;
    if(timesOfConsecutiveStateCalls().first == 1) flag_random_poly_init = false;
    else flag_random_poly_init = true;
    for(int i = 0; i < trial_times; i++){
      if(callReboundReplan(true, flag_random_poly_init)){
        return true;
      }
    }
    return false;
  }

  bool REMANIReplanFSM::planFromLocalTraj(bool flag_use_poly_init, bool allow_random_retry){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    const ros::Time now = ros::Time::now();
    const double t_now = getCurrentTrajectoryTime(*info, now);
    const double current_time_scale =
        getCurrentTrajectoryScale(*info, now);
    const double expected_progress =
        replan_trajectory_time_ * current_time_scale;
    const double t_cur = std::min(info->duration, t_now + expected_progress);

    start_pos_     = info->getPos(t_cur);
    start_vel_    = info->getVel(t_cur) * current_time_scale;
    start_acc_    = info->getAcc(t_cur) *
                    (current_time_scale * current_time_scale);
    start_jer_   = info->getJer(t_cur) *
                   (current_time_scale * current_time_scale * current_time_scale);
    start_singul_ = info->getSingul(t_cur);
    if(start_vel_.head(2).norm() >= mobile_base_non_singul_vel_) start_yaw_ = atan2(start_singul_ * start_vel_(1), start_singul_ * start_vel_(0));
    else start_yaw_ = mm_car_yaw_;

    // The original handoff assumed that the physical base stayed exactly on the
    // old trajectory. Preserve the measured base residual at the switch time so
    // the next trajectory starts from the real robot's predicted location.
    const Eigen::VectorXd expected_now = info->getPos(t_now);
    const Eigen::Vector2d position_error = mm_state_pos_.head(2) - expected_now.head(2);
    const Eigen::VectorXd arm_position_error =
        mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_) -
        expected_now.segment(mobile_base_dim_, manipulator_dim_);
    const Eigen::VectorXd expected_now_vel = info->getVel(t_now);
    const int expected_now_singul = info->getSingul(t_now);
    const double expected_now_yaw = expected_now_vel.head(2).norm() >= mobile_base_non_singul_vel_
        ? atan2(expected_now_singul * expected_now_vel(1), expected_now_singul * expected_now_vel(0))
        : mm_car_yaw_;
    const double yaw_error = atan2(sin(mm_car_yaw_ - expected_now_yaw), cos(mm_car_yaw_ - expected_now_yaw));

    if (position_error.norm() > replan_feedback_position_threshold_ ||
        fabs(yaw_error) > replan_feedback_yaw_threshold_) {
      start_pos_.head(2) += position_error;
      start_yaw_ = atan2(sin(start_yaw_ + yaw_error), cos(start_yaw_ + yaw_error));
      ROS_WARN("[FSM] Replan handoff feedback: pos_error=%.3f m, yaw_error=%.3f rad, predicted_start=(%.3f, %.3f)",
               position_error.norm(), yaw_error, start_pos_(0), start_pos_(1));
    }
    if(arm_position_error.cwiseAbs().maxCoeff() > replan_feedback_arm_threshold_){
      start_pos_.segment(mobile_base_dim_, manipulator_dim_) += arm_position_error;
      start_vel_.segment(mobile_base_dim_, manipulator_dim_) =
          mm_state_vel_.segment(mobile_base_dim_, manipulator_dim_);
      start_acc_.segment(mobile_base_dim_, manipulator_dim_).setZero();
      start_jer_.segment(mobile_base_dim_, manipulator_dim_).setZero();
      ROS_WARN("[FSM] Replan handoff uses measured arm state (max error %.3f rad).",
               arm_position_error.cwiseAbs().maxCoeff());
    }

    bool success = callReboundReplan(flag_use_poly_init, false);
    if (!success && allow_random_retry){
      for (int i = 0; i < 1; i++){
        success = callReboundReplan(true, true);
        if (success)
          break;
      }
    }

    if (!success)
      return false;

    return true;
  }

  bool REMANIReplanFSM::validateStaticTrajectory(const SingulTrajData &traj, int &coll_type) const
  {
    for (double t = 0.0; t <= traj.duration; t += global_plan_static_check_time_step_)
    {
      if (planner_manager_->ploy_traj_opt_->checkCollision(traj, t, coll_type))
        return false;
    }
    return true;
  }

  bool REMANIReplanFSM::validateDynamicTrajectory(
      const SingulTrajData &traj, int &coll_type) const
  {
    const double check_step = std::max(0.05, global_plan_static_check_time_step_);
    for (double t = 0.0; t <= traj.duration; t += check_step)
    {
      if (planner_manager_->ploy_traj_opt_->checkDynamicObstacleCollision(
              traj, t, coll_type))
        return false;
    }
    if (traj.duration > 0.0 &&
        planner_manager_->ploy_traj_opt_->checkDynamicObstacleCollision(
            traj, traj.duration, coll_type))
      return false;
    return true;
  }

  bool REMANIReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj){
    const bool preserve_executing_trajectory =
        have_local_traj_ &&
        !planner_manager_->traj_container_.singul_traj_data.singul_traj.empty();
    const SingulTrajData executing_trajectory =
        planner_manager_->traj_container_.singul_traj_data;

    bool reach_horizon;
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pos_, start_yaw_, end_pt_, end_yaw_,
        local_target_pt_, local_target_vel_, local_target_acc_, reach_horizon);
    bool local_target_gripper;
    if(reach_horizon){
      local_target_gripper = gripper_state_;
    }else{
      local_target_gripper = waypoint_gripper_close_[wpt_id_];
    }
    local_target_acc_.setZero();
    double local_target_yaw = atan2(local_target_vel_(1), local_target_vel_(0)); // global traj is foreward, no need to take singul into account
    local_target_vel_.setZero();
    local_target_vel_.head(2) = mobile_base_non_singul_vel_ * Eigen::Vector2d(cos(local_target_yaw), sin(local_target_yaw));

    Eigen::VectorXd desired_start_pt, desired_start_vel, desired_start_acc, desired_start_jerk;
    int desired_start_singul;
    double desired_start_yaw;
    double desired_start_time;
    
    if(have_local_traj_)
    {
      desired_start_time = ros::Time::now().toSec() + replan_trajectory_time_;
      desired_start_pt = start_pos_;
      desired_start_vel = start_vel_;
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_singul = start_singul_;
      desired_start_acc = start_acc_;
      desired_start_jerk = start_jer_;
      desired_start_yaw = start_yaw_;
    }else{
      desired_start_time = ros::Time::now().toSec();
      desired_start_pt = start_pos_;
      desired_start_vel = start_vel_;
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_acc = start_acc_;
      desired_start_jerk = start_jer_;
      desired_start_yaw = start_yaw_;
      desired_start_singul = start_singul_;
    }
    // std::cout << "desired_start_singul: " << desired_start_singul << std::endl;
    double init_time, opt_time;
    
    bool plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc,desired_start_jerk, desired_start_yaw, desired_start_singul, gripper_state_,
        desired_start_time, local_target_pt_, local_target_vel_, local_target_acc_, local_target_yaw, local_target_gripper,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, have_local_traj_, init_time, opt_time);
    have_new_target_ = false;

    if (plan_success){
      int coll_type = -1;
      const SingulTrajData &candidate = planner_manager_->traj_container_.singul_traj_data;
      if (global_plan_ && !validateStaticTrajectory(candidate, coll_type)){
        ROS_WARN("[FSM] Rejecting replan candidate: static collision type=%d remains after optimization.", coll_type);
        if (preserve_executing_trajectory)
          planner_manager_->traj_container_.singul_traj_data = executing_trajectory;
        return false;
      }
      if (global_plan_ && dynamic_safety_speed_limit_active_ &&
          !validateDynamicTrajectory(candidate, coll_type)){
        ROS_WARN("[FSM] Rejecting replan candidate: live obstacle collision type=%d remains.", coll_type);
        if (preserve_executing_trajectory)
          planner_manager_->traj_container_.singul_traj_data = executing_trajectory;
        return false;
      }
      init_time_list_.push_back(init_time);
      opt_time_list_.push_back(opt_time);
      total_time_list_.push_back(init_time + opt_time);
      sendPolyTrajROSMsg();
      have_local_traj_ = true;

      // vis local traj
      int i_end = floor(planner_manager_->traj_container_.singul_traj_data.duration / 0.02);
      std::vector<Eigen::Vector2d> local_path_list;
      Eigen::Vector2d local_traj_pt;
      for(int i = 0; i < i_end; ++i){
        local_traj_pt = planner_manager_->traj_container_.singul_traj_data.getPos(i * 0.02).head(2);
        local_path_list.push_back(local_traj_pt);
      }
      visualization_->displayGlobalTraj(local_path_list, 0.05, 0);
      planner_manager_->ploy_traj_opt_->displayBackEndMesh(planner_manager_->traj_container_.singul_traj_data, false, gripper_state_);
    } else if (preserve_executing_trajectory) {
      planner_manager_->traj_container_.singul_traj_data = executing_trajectory;
    }

    return plan_success;
  }

  bool REMANIReplanFSM::callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul){
    std::cout << "\033[31mcall EmergencyStop\033[0m" << std::endl;
    dynamic_replan_retry_pending_ = false;
    planner_manager_->EmergencyStop(stop_pos, stop_yaw, singul);
    quadrotor_msgs::PolynomialTraj msg;
    msg.action = quadrotor_msgs::PolynomialTraj::ACTION_ABORT;
    poly_traj_pub_.publish(msg);

    return true;
  }

} // namespace remani_planner
