/*
 * Copyright 2020 California  Institute  of Technology (“Caltech”)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "x_vio_ros/node.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <optional>
#include <utility>

using namespace x;

namespace enc = sensor_msgs::image_encodings;

/** Set up xVIO within a ROS node.
 */
#ifdef MULTI_UAV

Node::Node(ros::NodeHandle &nh,
           std::shared_ptr<Communication> communication)
        : communication_(std::move(communication)) {
    // set vehicle name
    veh_name = ros::this_node::getNamespace();
    // Load parameters from ROS
    loadParamsWithHandle(nh);

    // Set the estimator up with these parameters
    vio_.setUp(params_);

    // Set up inputs/outputs in ROS
    setInterfaceWithHandle(nh);

    // Initialize the filter at startup, if that parameter is enabled
    if (init_at_startup_) {
        ROS_INFO("Initializating xVIO");
        vio_.initAtTime(ros::Time::now().toSec());
    }
}

#endif

Node::Node(ros::NodeHandle &nh) {
    // set vehicle name
    veh_name = ros::this_node::getNamespace();

    // Load parameters from ROS
    loadParamsWithHandle(nh);

    // Set the estimator up with these parameters
    vio_.setUp(params_);

    // Set up inputs/outputs in ROS
    setInterfaceWithHandle(nh);

    // Initialize the filter at startup, if that parameter is enabled
    if (init_at_startup_) {
        ROS_INFO("Initializating xVIO");
        vio_.initAtTime(ros::Time::now().toSec());
    }
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr) {
    // Stream IMU callback notification in Debug mode
#ifdef DEBUG
    ROS_INFO_STREAM("\033[1;33mIMU callback on core:\033[0m "
                    << sched_getcpu()
                    << ", thread: " << std::this_thread::get_id()
                    << " at timestamp: " << imu_msg_ptr->header.stamp);
#endif
    // Read accels
    Vector3 a_m(imu_msg_ptr->linear_acceleration.x,
                imu_msg_ptr->linear_acceleration.y,
                imu_msg_ptr->linear_acceleration.z);

    // Read gyros
    Vector3 w_m(imu_msg_ptr->angular_velocity.x, imu_msg_ptr->angular_velocity.y,
                imu_msg_ptr->angular_velocity.z);

    // Call xVIO IMU propagation
    const auto propagated_state = vio_.processImu(imu_msg_ptr->header.stamp.toSec(),
                                                  imu_msg_ptr->header.seq, w_m, a_m);

#ifdef VERBOSE
    // Publish at IMU rate
    if (propagated_state.has_value()) {
        publishImuRate(*propagated_state);
    }
#endif
}

/** Reconfigure callback
 */
void Node::reconfigure(x_vio::xvioConfig &config, uint32_t level) {
    if (config.INITIALIZE_FILTER) {
        ROS_INFO("xVIO init request");
        // (Re-)initialization should happen after image processing has
        // returned since it is using pointers to the state vector
        // TODO: IMPORTANT!!!!!!!! read above comment
        vio_.initAtTime(ros::Time::now().toNSec());
        config.INITIALIZE_FILTER = false;
    }
}

/** \brief Load user parameters from ROS parameter server.
 *
 *  Parameters stored in params.yaml have been loaded on the ROS parameter
 *  server by the launch file. They are accessed with the node handle.
 */
void Node::loadParamsWithHandle(const ros::NodeHandle &nh) {
    // Eigen is not supported by ROS' getParam function so we import these
    // variable as std::vectors first
    std::vector<double> p, v, q, b_w, b_a, p_ic, q_ic, q_sc, w_s, sigma_dp,
            sigma_dv, sigma_dtheta, sigma_dbw, sigma_dba, g;

    // Import initial state or kill xVIO
    bool vio_lives = true;

    vio_lives = vio_lives && nh.getParam("x_vio/p", p);
    vio_lives = vio_lives && nh.getParam("x_vio/v", v);
    vio_lives = vio_lives && nh.getParam("x_vio/q", q);
    vio_lives = vio_lives && nh.getParam("x_vio/b_w", b_w);
    vio_lives = vio_lives && nh.getParam("x_vio/b_a", b_a);

    vio_lives = vio_lives && nh.getParam("x_vio/sigma_dp", sigma_dp);
    vio_lives = vio_lives && nh.getParam("x_vio/sigma_dv", sigma_dv);
    vio_lives = vio_lives && nh.getParam("x_vio/sigma_dtheta", sigma_dtheta);
    vio_lives = vio_lives && nh.getParam("x_vio/sigma_dbw", sigma_dbw);
    vio_lives = vio_lives && nh.getParam("x_vio/sigma_dba", sigma_dba);

    if (!vio_lives) {
        ROS_ERROR("Initial state parameters are missing!");
    }
    // Import camera calibration or kill xVIO.
    vio_lives = vio_lives && nh.getParam("x_vio/cam1_fx", params_.cam_fx);
    vio_lives = vio_lives && nh.getParam("x_vio/cam1_fy", params_.cam_fy);
    vio_lives = vio_lives && nh.getParam("x_vio/cam1_cx", params_.cam_cx);
    vio_lives = vio_lives && nh.getParam("x_vio/cam1_cy", params_.cam_cy);
    vio_lives = vio_lives && nh.getParam("x_vio/cam1_s", params_.cam_s);
    vio_lives =
            vio_lives && nh.getParam("x_vio/cam1_img_height", params_.img_height);
    vio_lives =
            vio_lives && nh.getParam("x_vio/cam1_img_width", params_.img_width);
    vio_lives = vio_lives && nh.getParam("x_vio/cam1_p_ic", p_ic);
    vio_lives = vio_lives && nh.getParam("x_vio/cam1_q_ic", q_ic);
    vio_lives =
            vio_lives && nh.getParam("x_vio/cam1_time_offset", params_.time_offset);
    vio_lives = vio_lives && nh.getParam("x_vio/sigma_img", params_.sigma_img);
    if (!vio_lives) {
        ROS_ERROR("Camera parameters are missing!");
    }
    // Laser range finder
    vio_lives =
            vio_lives && nh.getParam("x_vio/sigma_range", params_.sigma_range);

    if (!vio_lives) {
        ROS_ERROR("Laser range finder parameters are missing!");
    }
    // Import sun sensor calibration or kill xVIO.
    vio_lives = vio_lives && nh.getParam("x_vio/q_sc", q_sc);
    vio_lives = vio_lives && nh.getParam("x_vio/w_s", w_s);

    if (!vio_lives) {
        ROS_ERROR("Sun sensor parameters are missing!");
    }
    // Import IMU parameters or kill xVIO
    vio_lives = vio_lives && nh.getParam("x_vio/n_a", params_.n_a);
    vio_lives = vio_lives && nh.getParam("x_vio/n_ba", params_.n_ba);
    vio_lives = vio_lives && nh.getParam("x_vio/n_w", params_.n_w);
    vio_lives = vio_lives && nh.getParam("x_vio/n_bw", params_.n_bw);

    if (!vio_lives) {
        ROS_ERROR("IMU parameters are missing!");
    }
#ifdef PHOTOMETRIC_CALI
    // Import Photometric calibration parameters
    vio_lives = vio_lives && nh.getParam("x_vio/temporal_params_div",
                                         params_.temporal_params_div);
    vio_lives =
            vio_lives && nh.getParam("x_vio/spatial_params", params_.spatial_params);
    vio_lives = vio_lives && nh.getParam("x_vio/spatial_params_thr",
                                         params_.spatial_params_thr);
    vio_lives =
            vio_lives && nh.getParam("x_vio/epsilon_gap", params_.epsilon_gap);
    vio_lives =
            vio_lives && nh.getParam("x_vio/epsilon_base", params_.epsilon_base);

    vio_lives = vio_lives &&
                nh.getParam("x_vio/win_size_w_photo", params_.win_size_w_photo);
    vio_lives = vio_lives &&
                nh.getParam("x_vio/win_size_h_photo", params_.win_size_h_photo);
    vio_lives = vio_lives &&
                nh.getParam("x_vio/max_level_photo", params_.max_level_photo);
    vio_lives = vio_lives &&
                nh.getParam("x_vio/min_eig_thr_photo", params_.min_eig_thr_photo);
    vio_lives = vio_lives && nh.getParam("x_vio/fast_detection_delta_photo",
                                         params_.fast_detection_delta_photo);

    if (!vio_lives) {
        ROS_ERROR("Photometric calibration parameters are missing!");
    }
#endif

#ifdef MULTI_UAV
    // Import Multi UAV parameters
    vio_lives = vio_lives &&
                nh.getParam("x_vio/vocabulary_path", params_.vocabulary_path);
    std::cout << "VOCABULARY PATH: " << params_.vocabulary_path << std::endl;
    vio_lives =
            vio_lives && nh.getParam("x_vio/sigma_landmark", params_.sigma_landmark);

    vio_lives = vio_lives && nh.getParam("x_vio/descriptor_scale_factor",
                                         params_.descriptor_scale_factor);
    vio_lives = vio_lives && nh.getParam("x_vio/descriptor_pyramid",
                                         params_.descriptor_pyramid);
    vio_lives = vio_lives && nh.getParam("x_vio/descriptor_patch_size",
                                         params_.descriptor_patch_size);

    vio_lives = vio_lives && nh.getParam("x_vio/ci_msckf_w", params_.ci_msckf_w);

    vio_lives = vio_lives && nh.getParam("x_vio/ci_slam_w", params_.ci_slam_w);

    vio_lives = vio_lives && nh.getParam("x_vio/desc_type", params_.desc_type);

    vio_lives =
            vio_lives && nh.getParam("x_vio/pr_score_thr", params_.pr_score_thr);

    vio_lives = vio_lives &&
                nh.getParam("x_vio/pr_desc_ratio_thr", params_.pr_desc_ratio_thr);

    vio_lives = vio_lives && nh.getParam("x_vio/pr_desc_min_distance",
                                         params_.pr_desc_min_distance);

    if (!vio_lives)
        ROS_ERROR("Multi UAV parameters are missing!");
#endif

    // Import visual front end parameters or kill xVIO
    vio_lives =
            vio_lives && nh.getParam("x_vio/min_eig_thr", params_.min_eig_thr);
    vio_lives = vio_lives && nh.getParam("x_vio/max_level", params_.max_level);
    vio_lives = vio_lives && nh.getParam("x_vio/win_size_w", params_.win_size_w);
    vio_lives = vio_lives && nh.getParam("x_vio/win_size_h", params_.win_size_h);

    vio_lives = vio_lives && nh.getParam("x_vio/fast_detection_delta",
                                         params_.fast_detection_delta);
    vio_lives =
            vio_lives && nh.getParam("x_vio/non_max_supp", params_.non_max_supp);
    vio_lives = vio_lives &&
                nh.getParam("x_vio/block_half_length", params_.block_half_length);
    vio_lives = vio_lives && nh.getParam("x_vio/margin", params_.margin);
    vio_lives = vio_lives && nh.getParam("x_vio/n_feat_min", params_.n_feat_min);
    vio_lives =
            vio_lives && nh.getParam("x_vio/outlier_method", params_.outlier_method);
    vio_lives =
            vio_lives && nh.getParam("x_vio/outlier_param1", params_.outlier_param1);
    vio_lives =
            vio_lives && nh.getParam("x_vio/outlier_param2", params_.outlier_param2);
    vio_lives = vio_lives && nh.getParam("x_vio/n_tiles_h", params_.n_tiles_h);
    vio_lives = vio_lives && nh.getParam("x_vio/n_tiles_w", params_.n_tiles_w);
    vio_lives = vio_lives &&
                nh.getParam("x_vio/max_feat_per_tile", params_.max_feat_per_tile);

    if (!vio_lives) {
        ROS_ERROR("Visual front end parameters are missing!");
    }
    // Import filter parameters or kill xVIO
    vio_lives =
            vio_lives && nh.getParam("x_vio/n_poses_max", params_.n_poses_max);
    vio_lives = vio_lives && nh.getParam("x_vio/n_slam_features_max",
                                         params_.n_slam_features_max);
    vio_lives = vio_lives && nh.getParam("x_vio/rho_0", params_.rho_0);
    vio_lives =
            vio_lives && nh.getParam("x_vio/sigma_rho_0", params_.sigma_rho_0);
    vio_lives = vio_lives && nh.getParam("x_vio/iekf_iter", params_.iekf_iter);
    vio_lives =
            vio_lives && nh.getParam("x_vio/msckf_baseline", params_.msckf_baseline);
    vio_lives = vio_lives &&
                nh.getParam("x_vio/min_track_length", params_.min_track_length);
    vio_lives = vio_lives &&
                nh.getParam("x_vio/traj_timeout_gui", traj_timeout_gui_);
    vio_lives = vio_lives &&
                nh.getParam("x_vio/init_at_startup", init_at_startup_);
    vio_lives = vio_lives && nh.getParam("x_vio/state_buffer_size", params_.state_buffer_size);

    if (!vio_lives) {
        ROS_ERROR("Filter parameters are missing!");
    }
    // Import gravity or fill xVIO
    vio_lives = vio_lives && nh.getParam("x_vio/g", g);

    if (!vio_lives) {
        ROS_ERROR("Gravity parameter is missing!");
    }
    // Convert std::vectors to msc vectors and quaternions in params
    params_.p << p[0], p[1], p[2];
    params_.v << v[0], v[1], v[2];
    params_.q.w() = q[0];
    params_.q.x() = q[1];
    params_.q.y() = q[2];
    params_.q.z() = q[3];
    params_.q.normalize();
    params_.b_w << b_w[0], b_w[1], b_w[2];
    params_.b_a << b_a[0], b_a[1], b_a[2];
    params_.sigma_dp << sigma_dp[0], sigma_dp[1], sigma_dp[2];
    params_.sigma_dv << sigma_dv[0], sigma_dv[1], sigma_dv[2];
    params_.sigma_dtheta << sigma_dtheta[0], sigma_dtheta[1], sigma_dtheta[2];
    params_.sigma_dbw << sigma_dbw[0], sigma_dbw[1], sigma_dbw[2];
    params_.sigma_dba << sigma_dba[0], sigma_dba[1], sigma_dba[2];
    params_.p_ic << p_ic[0], p_ic[1], p_ic[2];
    params_.q_ic.w() = q_ic[0];
    params_.q_ic.x() = q_ic[1];
    params_.q_ic.y() = q_ic[2];
    params_.q_ic.z() = q_ic[3];
    params_.q_ic.normalize();
    params_.q_sc.w() = q_sc[0];
    params_.q_sc.x() = q_sc[1];
    params_.q_sc.y() = q_sc[2];
    params_.q_sc.z() = q_sc[3];
    params_.q_sc.normalize();
    params_.w_s << w_s[0], w_s[1], w_s[2];
    params_.w_s.normalize();
    params_.g << g[0], g[1], g[2];
}

/**
 * Set xVIO subscribers/publishers for ROS.
 */
void Node::setInterfaceWithHandle(ros::NodeHandle &nh) {
    image_transport::ImageTransport it(nh);

    /***************
     * Subscribers *
     ***************/
    // Note: only one of the vision subscribers should actually be receiving data
    // during a given run, either images or matches. Although nothing technically
    // prevents the processing of both in the code, something has most likely
    // been set incorrectly by the user if this is happening.

    imu_sub_ = nh.subscribe("imu", 20, &Node::imuCallback, this);
    img_sub_ = it.subscribe("image_raw", 10, &Node::visionImageCallback, this);

#if defined(MULTI_UAV) && defined(DEBUG)
    other_img_sub_ =
            it.subscribe("/UAV1/image_raw", 2, &Node::otherVisionImageCallback, this);
    it.subscribe("/UAV2/image_raw", 2, &Node::otherVisionImageCallback, this);
    it.subscribe("/UAV3/image_raw", 2, &Node::otherVisionImageCallback, this);
#endif

    matches_sub_ =
            nh.subscribe("features", 10, &Node::visionMatchesCallback, this);
    range_sub_ = nh.subscribe("range", 10, &Node::rangeCallback, this);
    sun_angle_sub_ = nh.subscribe("sun_angles", 10, &Node::sunAngleCallback, this);

    /**************
     * Publishers *
     **************/

    pub_pose_with_cov_image_rate_ =
            nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                    "pose_with_cov_image_rate", 10);

#ifdef MULTI_UAV
    pub_multi_uav_request_ = nh.advertise<x_vio_ros::RequestUAV>("request", 10);
#endif

#if defined(MULTI_UAV) // && defined(DEBUG) // move this inside the verbose
    // block
    matches_image_pub_ = it.advertise("matches", 1);
#endif

#ifdef VERBOSE
    pub_pose_with_cov_imu_rate_ =
            nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                    "pose_with_cov_imu_rate", 2);
    tracker_pub_ = it.advertise("tracker", 1);
    track_manager_pub_ = it.advertise("track_manager", 1);

    pub_tf_imu_rate_ =
            nh.advertise<geometry_msgs::TransformStamped>("tf_imu_rate", 2);
    pub_tf_image_rate_ =
            nh.advertise<geometry_msgs::TransformStamped>("tf_image_rate", 2);
    pub_traj_imu_rate =
            nh.advertise<visualization_msgs::Marker>("traj_imu_rate", 2);
    pub_traj_image_rate_ =
            nh.advertise<visualization_msgs::Marker>("traj_image_rate", 2);
    pub_state_with_cov_image_rate_ =
            nh.advertise<x_vio_ros::InertialStateWithCovarianceStamped>(
                    "state_with_cov_image_rate", 2);
    pub_state_with_cov_imu_rate_ =
            nh.advertise<x_vio_ros::InertialStateWithCovarianceStamped>(
                    "state_with_cov_imu_rate", 2);
    rviz_publisher_.setUpPublishers(params_.n_poses_max,
                                    params_.n_slam_features_max);
#endif

    // Print published/subscribed topics.
    ros::V_string topics;
    ros::this_node::getSubscribedTopics(topics);
    const std::string &nodeName = ros::this_node::getName();
    std::string topicsStr = "\n\t" + nodeName + ":\n\tsubscribed topics:\n";
    for (auto &topic: topics) {
        topicsStr += ("\t\t" + topic + "\n");
    }
    topicsStr += "\tadvertised topics:\n";
    ros::this_node::getAdvertisedTopics(topics);
    for (auto &topic: topics) {
        topicsStr += ("\t\t" + topic + "\n");
    }
    ROS_INFO_STREAM(topicsStr);
}

/** \brief Image callback
 *
 * Handles threading and starts vision processing.
 */
void Node::visionImageCallback(const sensor_msgs::ImageConstPtr &img_ptr) {
#ifdef DEBUG
    std::stringstream img_stamp;
    img_stamp << std::setprecision(20) << img_ptr->header.stamp;
    std::cout << "\033[1;31mImage callback on core:\033[0m" << sched_getcpu()
              << ", thread: " << std::this_thread::get_id()
              << " at timestamp: " << img_stamp.str() << std::endl;
#endif
    visual_queue_.push(VisualData(img_ptr));
#ifndef MULTI_THREAD
    processVision();
#endif
}

/** \brief Visual matches callback
 *
 * Handles threading and start vision processing.
 */
void Node::visionMatchesCallback(
        const std_msgs::Float64MultiArray::ConstPtr &matches_ptr) {
#ifdef DEBUG
    std::cout << "\033[1;31mMatches callback on core:\033[0m" << sched_getcpu()
              << ", thread: " << std::this_thread::get_id() << std::endl;
#endif

    visual_queue_.push(VisualData(matches_ptr));
#ifndef MULTI_THREAD
    processVision();
#endif
}

/** Range callback
 */
void Node::rangeCallback(const sensor_msgs::RangeConstPtr &range_msg_ptr) {
    // Stream IMU callback notification in Debug mode
#ifdef DEBUG
    ROS_INFO_STREAM("\033[1;33mRange callback on core:\033[0m "
                    << sched_getcpu()
                    << ", thread: " << std::this_thread::get_id()
                    << " at timestamp: " << range_msg_ptr->header.stamp);
#endif

    // Initialize XIO range measurement
    RangeMeasurement range;
    range.timestamp = range_msg_ptr->header.stamp.toSec();
    range.range = range_msg_ptr->range;

    // Set it as last range measurement in xVIO
    vio_.setLastRangeMeasurement(range);
}

/** Sun angle callback
 */
void Node::sunAngleCallback(
        const geometry_msgs::Vector3StampedConstPtr &sun_angle_msg_ptr) {
    // Stream sun angle callback notification in Debug mode
#ifdef DEBUG
    ROS_INFO_STREAM("\033[1;33mSun angle callback on core:\033[0m "
                    << sched_getcpu()
                    << ", thread: " << std::this_thread::get_id()
                    << " at timestamp: " << sun_angle_msg_ptr->header.stamp);
#endif

    // Initialize XIO sun angle measurement
    SunAngleMeasurement angle;
    angle.timestamp = sun_angle_msg_ptr->header.stamp.toSec();
    angle.x_angle = sun_angle_msg_ptr->vector.x;
    angle.y_angle = sun_angle_msg_ptr->vector.y;

    // Set it as last sun angle measurement in XIO
    vio_.setLastSunAngleMeasurement(angle);
}

/** \brief Common vision node interface
 *
 * When new vision data is ready to be processed, convert from ROS to xVIO
 * format and call xVIO vision measurement function.
 */
void Node::processVision() {
#ifdef MULTI_THREAD
    while (run_threads_) {
#endif
        visual_queue_.consume_all([&](const VisualData &data) {
                                      TiledImage match_img, feature_img;
                                      std::optional<State> updated_state;
                                      if (data.img.has_value()) {
                                          const auto img_ptr_ = *data.img;

#ifdef DEBUG
                                          std::stringstream img_stamp;
                                          img_stamp << std::setprecision(20) << img_ptr_->header.stamp;
                                          std::cout << "\033[1;31mStarting image processing on core:\033[0m"
                                                    << sched_getcpu()
                                                    << ", thread: " << std::this_thread::get_id()
                                                    << " at timestamp: " << img_ptr_->header.stamp.toNSec() << std::endl;
#endif
                                          // Initialize two image objects: both pass the raw image data. They will
                                          // be modified by the tracker and track manager to plot matches and
                                          // features types, respectively.
                                          const unsigned int frame_number = img_ptr_->header.seq;
                                          cv_bridge::CvImageConstPtr cv_ptr;
                                          try {
                                              cv_ptr = cv_bridge::toCvShare(img_ptr_, enc::MONO8);
                                          } catch (cv_bridge::Exception &e) {
                                              ROS_ERROR("cv_bridge exception: %s", e.what());
                                              return;
                                          }

                                          // Shallow copies
                                          match_img = TiledImage(cv_ptr->image, img_ptr_->header.stamp.toSec(),
                                                                 frame_number, params_.n_tiles_h, params_.n_tiles_w, params_.max_feat_per_tile);
                                          feature_img = TiledImage(match_img);

                                          updated_state = vio_.processImageMeasurement(
                                                  img_ptr_->header.stamp.toSec(), frame_number, match_img,
                                                  feature_img);

#ifdef DEBUG
                                          std::cout << "\033[1;31mEnding image processing on core:\033[0m"
                                                    << sched_getcpu()
                                                    << ", thread: " << std::this_thread::get_id()
                                                    << " at timestamp: " << img_stamp.str() << std::endl;
#endif

                                      } else if (data.matches.has_value()) {
                                          const auto matches_ptr_ = *data.matches;
#ifdef DEBUG
                                          std::cout << "\033[1;31mStarting matches processing on core:\033[0m"
                                                    << sched_getcpu()
                                                    << ", thread: " << std::this_thread::get_id()
                                                    << " at timestamp: " << matches_ptr_->data[4] << std::endl;
#endif
                                          // Increment seq ID
                                          ++seq_;

                                          static cv::Mat cv_match_img(params_.img_height, params_.img_width, CV_8UC1,
                                                                      cv::Scalar(0));

                                          const auto timestamp = matches_ptr_->data[4];
                                          match_img = TiledImage(cv_match_img, timestamp, seq_,
                                                                 params_.n_tiles_h,
                                                                 params_.n_tiles_w, params_.max_feat_per_tile);
                                          feature_img = TiledImage(match_img);

                                          // Convert RpublishImageRateOS match message to std::vector (no ROS allowed beyond the
                                          // wrapper)
                                          std::vector<double> matches = matches_ptr_->data;

                                          // Pass matches to VIO
                                          updated_state = vio_.processMatchesMeasurement(timestamp, seq_, matches,
                                                                                         match_img, feature_img);
                                      } else {
                                          return;
                                      }

                                      if (updated_state.has_value()) {
#ifdef MULTI_UAV
                                          multi_uav_counter_++;
                                          if (multi_uav_counter_ == reduction_factor_) {
#ifdef REQUEST_COMM
                                              cv::Mat descriptors = vio_.getDescriptors();
                                              if (descriptors.rows > 10) {
                                                  publishRequestUAV(ros::Time(updated_state->getTime()), descriptors);
                                              }
#else
                                              std::shared_ptr<SimpleState> state_to_send;
                                              msckf_track_list_.clear();
                                              slam_track_list_.clear();
                                              anchor_idxs_.clear();
                                              opp_track_list_.clear();
                                              vio_.getDataToSend(state_to_send, *updated_state, msckf_track_list_,
                                                                 slam_track_list_, anchor_idxs_, opp_track_list_);
                                              publishMessageUAV(ros::Time(updated_state->getTime()), -1,
                                                                state_to_send, msckf_track_list_, slam_track_list_,
                                                                slam_feature_list_, opp_track_list_, anchor_idxs_);
#endif
                                              multi_uav_counter_ = 0;
                                          }
#endif
                                          // Publish state
                                          publishImageRate(*updated_state);

#ifdef VERBOSE
                                          // Publish images to be displayed in the GUI
                                          const sensor_msgs::ImagePtr match_img_msg =
                                                  cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, match_img)
                                                          .toImageMsg();
                                          tracker_pub_.publish(match_img_msg);

                                          const sensor_msgs::ImagePtr feature_img_msg =
                                                  cv_bridge::CvImage(std_msgs::Header(), enc::RGB8, feature_img)
                                                          .toImageMsg();
                                          track_manager_pub_.publish(feature_img_msg);

                                          // Publish 3D features
                                          publishSLAMFeatures(*updated_state);
                                          Vector3dArray msckf_inliers, msckf_outliers;
                                          vio_.getMsckfFeatures(msckf_inliers, msckf_outliers);
                                          rviz_publisher_.publishMsckfFeatures(msckf_inliers, msckf_outliers);
#endif
                                      }

                                  }
        );

#ifdef MULTI_THREAD
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
#endif

}

#ifdef MULTI_UAV

void Node::processOther() {
#ifdef MULTI_THREAD
    while (run_threads_) {
        message_queue_.consume_all([&](const x_vio_ros::MessageUAV::ConstPtr
                                       other_data_ptr) {
#else
            // Get data
            const x_vio_ros::MessageUAV::ConstPtr other_data_ptr = other_data_ptr_;
#endif
            if (other_data_ptr != NULL) // new image message received
            {
#ifdef DEBUG
                std::stringstream other_stamp;
                other_stamp << std::setprecision(20) << other_data_ptr->header.stamp;
                std::cout
                        << "\033[1;31mStarting Multi UAV message processing on core:\033[0m"
                        << sched_getcpu() << ", thread: " << std::this_thread::get_id()
                        << " at timestamp: " << other_stamp.str() << std::endl;
#endif
                int sender_uav = other_data_ptr->uav_id; // UAV name
                double timestamp = other_data_ptr->header.stamp.toSec();

#ifdef DEBUG
                std::cout << "Message received by " << sender_uav << std::endl;
#endif

                // Construct the state
                int n_poses = other_data_ptr->n_poses;

                Vectorx dynamic_state(16); // length of the dynamic is fixed at 16
                for (int i = 0; i < 16; i++) {
                    dynamic_state[i] = other_data_ptr->dynamic_state[i];
                }
                Vectorx positions_state(3 * n_poses);
                for (int i = 0; i < 3 * n_poses; i++) {
                    positions_state[i] = other_data_ptr->positions_state[i];
                }
                Vectorx orientations_state(4 * n_poses);
                for (int i = 0; i < 4 * n_poses; i++) {
                    orientations_state[i] = other_data_ptr->orientations_state[i];
                }
                const int n_features_state = other_data_ptr->features_state.size();
                Vectorx features_state(n_features_state);
                for (int i = 0; i < n_features_state; i++) {
                    features_state[i] = other_data_ptr->features_state[i];
                }
                int state_size = 15 + 6 * n_poses + n_features_state;

                Matrix cov(state_size, state_size);

                for (int i = 0; i < state_size * state_size; i++) {
                    cov(i) = other_data_ptr->cov[i];
                }
                // construct the features
                TrackListPtr msckf_trks_ptr, slam_trks_ptr, opp_tracks_ptr;
                // FeatureListPtr opp_features_ptr;
                Feature tmp_ftr; // tmp_feature

                // MSCKF tracks initialization
                for_each(other_data_ptr->msckf_tracks.begin(),
                         other_data_ptr->msckf_tracks.end(),
                         [&msckf_trks_ptr, &tmp_ftr](const x_vio_ros::TrackMsg &t) {
                             TrackPtr trk = std::make_shared<Track>(
                                     t.id); // init the track with the received id

                             std::vector<uchar> desc =
                                     t.descriptor; // set the track descriptor
                             // ORB descriptor is 32 byte long
                             cv::Mat m_desc = cv::Mat(
                                     1, 32, CV_8U,
                                     desc.data()); // convert the descriptor vector to image
                             trk->setDescriptor(m_desc); // set the track descriptor

#ifdef GT_DEBUG
                             // init the 3D landmark received from the ground truth data
                             auto land = t.landmark;
                             Vector3 landmark = Vector3(land[0], land[1], land[2]);
#endif
                             // populate the track with the features
                             for_each(t.features.begin(), t.features.end(),
                                      [&](const x_vio_ros::FeatureMsg &f) {
                                          tmp_ftr = Feature(0, f.x, f.y, 0.0);
#ifdef GT_DEBUG
                                          tmp_ftr.setLandmark(
                                              landmark); // set the 3D gt landmark
#endif
                                          trk->push_back(tmp_ftr);
                                      });
                             if (!trk->empty())
                                 msckf_trks_ptr.push_back(trk);
                         });

                // SLAM tracks initialization
                for_each(other_data_ptr->slam_tracks.begin(),
                         other_data_ptr->slam_tracks.end(),
                         [&slam_trks_ptr, &tmp_ftr](const x_vio_ros::TrackMsg &t) {
                             TrackPtr trk = std::make_shared<Track>(
                                     t.id); // init the track with the received id

                             std::vector<uchar> desc =
                                     t.descriptor; // set the track descriptor
                             // ORB descriptor is 32 byte long
                             cv::Mat m_desc = cv::Mat(
                                     1, 32, CV_8U,
                                     desc.data()); // convert the descriptor vector to image
                             trk->setDescriptor(
                                     m_desc.clone()); // set the track descriptor

#ifdef GT_DEBUG
                             // init the 3D landmark received from the ground truth data
                             auto land = t.landmark;
                             Vector3 landmark = Vector3(land[0], land[1], land[2]);
#endif
                             // populate the track with the features
                             for_each(t.features.begin(), t.features.end(),
                                      [&](const x_vio_ros::FeatureMsg &f) {
                                          tmp_ftr = Feature(0, f.x, f.y, 0.0);
#ifdef GT_DEBUG
                                          tmp_ftr.setLandmark(
                                              landmark); // set the 3D gt landmark
#endif
                                          trk->push_back(tmp_ftr);
                                      });

                             if (!trk->empty())
                                 slam_trks_ptr.push_back(trk);
                         });

                /**
                 * @brief OPP features initialization
                 *
                 */
                for_each(other_data_ptr->opp_tracks.begin(),
                         other_data_ptr->opp_tracks.end(),
                         [&opp_tracks_ptr, &tmp_ftr](const x_vio_ros::TrackMsg &t) {
                             TrackPtr trk = std::make_shared<Track>(
                                     t.id); // init the track with the received id
                             std::vector<uchar> desc =
                                     t.descriptor; // set the track descriptor
                             // ORB descriptor is 32 byte long
                             cv::Mat m_desc = cv::Mat(
                                     1, 32, CV_8U,
                                     desc.data()); // convert the descriptor vector to image
                             trk->setDescriptor(
                                     m_desc.clone()); // set the track descriptor

#ifdef GT_DEBUG
                             // init the 3D landmark received from the ground truth data
                             auto land = t.landmark;
                             Vector3 landmark = Vector3(land[0], land[1], land[2]);
#endif
                             // populate the track with the features
                             for_each(t.features.begin(), t.features.end(),
                                      [&](const x_vio_ros::FeatureMsg &f) {
                                          tmp_ftr = Feature(0, f.x, f.y, 0.0);
#ifdef GT_DEBUG
                                          tmp_ftr.setLandmark(
                                              landmark); // set the 3D gt landmark
#endif
                                          trk->push_back(tmp_ftr);
                                      });

                             if (!trk->empty())
                                 opp_tracks_ptr.push_back(trk);
                         });

                std::vector<int> anchor_idxs;
                for (int a: other_data_ptr->anchor_ids) {
                    anchor_idxs.emplace_back(a);
                }

#ifdef GD_DEBUG
                // create a black image for storing the matches
                cv::Mat other_image_(params_.img_height, params_.img_width, CV_8UC1,
                                     cv::Scalar(0));
#endif
                cv::Mat other_image_(params_.img_height, params_.img_width, CV_8UC1,
                                     cv::Scalar(0));
                // Process the received data.
                // TODO: do this in a separate thread.
                auto updated_state = vio_.processOtherMeasurements(
                        timestamp, sender_uav, dynamic_state, positions_state,
                        orientations_state, features_state, cov, msckf_trks_ptr,
                        slam_trks_ptr, opp_tracks_ptr, anchor_idxs, other_image_);


                if (updated_state.has_value()) {
                    // Publish state
                    publishImageRate(*updated_state);
                }

#ifdef DEBUG
                std::cout << "\033[1;31mEnding Multi UAV message processing on core:\033[0m"
                        << sched_getcpu() << ", thread: " << std::this_thread::get_id()
                        << " at timestamp: " << other_stamp.str() << std::endl;
#endif
            }

#ifdef MULTI_THREAD
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        });
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
#endif
}

void Node::processOtherRequest() {
#ifdef MULTI_THREAD
    while (run_threads_) {
        // Check there is indeed new data, otherwise sleep the thread
        request_queue_.consume_all([&](const x_vio_ros::RequestUAV::ConstPtr
                                       other_request_data_ptr) {
#else
            // Get data
            const x_vio_ros::RequestUAV::ConstPtr other_request_data_ptr =
                other_request_data_ptr_;
#endif
            if (other_request_data_ptr != NULL) // new image message received
            {
#ifdef DEBUG
                std::cout
                        << "\033[1;31mStarting Multi UAV request processing on core:\033[0m"
                        << sched_getcpu() << ", thread: " << std::this_thread::get_id()
                        << std::endl;
#endif
                auto timestamp = other_request_data_ptr->timestamp;
                int sender_uav = other_request_data_ptr->uav_id; // UAV name

#ifdef DEBUG
                std::cout << "Request received by " << sender_uav << std::endl;
#endif
                const sensor_msgs::Image img = other_request_data_ptr->descriptors;
                const sensor_msgs::ImagePtr img_ptr(
                        new sensor_msgs::Image(other_request_data_ptr->descriptors));

                cv_bridge::CvImageConstPtr cv_ptr;
                try {
                    cv_ptr = cv_bridge::toCvShare(img_ptr, enc::MONO8);
                } catch (cv_bridge::Exception &e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return;
                }
                cv::Mat desc_mat = cv_ptr->image;

                std::shared_ptr<SimpleState> state_to_send;
                msckf_track_list_.clear();
                slam_track_list_.clear();
                anchor_idxs_.clear();
                opp_track_list_.clear();
                vio_.processOtherRequests(sender_uav, desc_mat, state_to_send,
                                          msckf_track_list_, slam_track_list_,
                                          anchor_idxs_, opp_track_list_);

                if (state_to_send != nullptr) {
                    // std::cout << "Sendind data...\n";
                    publishMessageUAV(timestamp, sender_uav, state_to_send,
                                      msckf_track_list_, slam_track_list_,
                                      slam_feature_list_, opp_track_list_, anchor_idxs_);
                }

#ifdef DEBUG
                std::cout
                        << "\033[1;31mEnding Multi UAV request processing on core:\033[0m"
                        << sched_getcpu() << ", thread: " << std::this_thread::get_id()
                        << std::endl;
#endif
            }

#ifdef MULTI_THREAD
std::this_thread::sleep_for(std::chrono::microseconds(100));
        });
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
#endif
}

#endif

#ifdef VERBOSE

void Node::publishImuRate(const State &state) const {
    // Parse state
    const auto t = std::chrono::duration<double>(state.getTime()).count();
    const Vector3 p = state.getPosition();
    const Quaternion q = state.getOrientation();
    const Matrix &pose_cov = state.getPoseCovariance();

    // Just compute the header once for all messages
    // WARNING: header's seq is overwritten by ROS. There is no point in setting
    // it here.
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.stamp = ros::Time(t);
    msg_pose.header.frame_id = "world";
    msg_pose.pose.pose.position.x = p(0);
    msg_pose.pose.pose.position.y = p(1);
    msg_pose.pose.pose.position.z = p(2);
    msg_pose.pose.pose.orientation.w = q.w();
    msg_pose.pose.pose.orientation.x = q.x();
    msg_pose.pose.pose.orientation.y = q.y();
    msg_pose.pose.pose.orientation.z = q.z();
    eigen2RosPoseCov(pose_cov, msg_pose.pose.covariance);

    // Publishing
    if (pub_pose_with_cov_imu_rate_.getNumSubscribers()) // pose
        pub_pose_with_cov_imu_rate_.publish(msg_pose);

    if (pub_tf_imu_rate_.getNumSubscribers()) // transform
    {
        geometry_msgs::TransformStamped msg_tf;
        msg_tf.header = msg_pose.header;
        msg_tf.child_frame_id = "xVIO" + veh_name;
        msg_tf.transform.translation.x = p(0);
        msg_tf.transform.translation.y = p(1);
        msg_tf.transform.translation.z = p(2);
        msg_tf.transform.rotation.w = q.w();
        msg_tf.transform.rotation.x = q.x();
        msg_tf.transform.rotation.y = q.y();
        msg_tf.transform.rotation.z = q.z();
        pub_tf_imu_rate_.publish(msg_tf);
    }

    static int msg_seq_imu = 0;
    if (pub_traj_imu_rate.getNumSubscribers()) // trajectory
    {
        visualization_msgs::Marker msg_traj;
        InitTrajMsgAtScale(msg_traj, 0.008);
        msg_traj.header = msg_pose.header;
        msg_traj.id = msg_seq_imu++;
        msg_traj.pose.position = msg_pose.pose.pose.position;
        msg_traj.pose.orientation = msg_pose.pose.pose.orientation;
        msg_traj.lifetime = ros::Duration(traj_timeout_gui_);
        pub_traj_imu_rate.publish(msg_traj);
    }

    // Publishing state @ IMU rate
    if (pub_state_with_cov_imu_rate_.getNumSubscribers()) {
        x_vio_ros::InertialStateWithCovarianceStamped msg_state;
        msg_state.header = msg_pose.header;

        const Eigen::VectorXd dyn_states = state.getDynamicStates();
        const int n_dyn_states = dyn_states.rows();
        msg_state.state.resize(n_dyn_states);
        for (int i = 0; i < n_dyn_states; ++i)
            msg_state.state[i] = dyn_states(i);

        const Matrix cov = state.getDynamicCovariance();
        const int n_cov = kSizeCoreErr * kSizeCoreErr;
        msg_state.cov.resize(n_cov);
        for (int i = 0; i < n_cov; ++i) {
            msg_state.cov[i] = cov(i);
        }

        pub_state_with_cov_imu_rate_.publish(msg_state);
    }

    // TF broadcast
    broadcastTf(p, q);
}

/** Publishes the SLAM features in rviz.
 */
void Node::publishSLAMFeatures(const State &state) {
    // Delete all persistent features previously published in rviz
    // (Avoid tracking publishing IDs, make it clever if necessary)
    rviz_publisher_.deleteAllPersistentFeatures();

    const std::vector<Eigen::Vector3d> features =
            vio_.computeSLAMCartesianFeaturesForState(state);

    // Publish in rviz
    for (unsigned int i = 0; i < features.size(); i++) {
        const Eigen::Vector3d& feature = features[i];
        rviz_publisher_.publishPersistentFeature(feature(0), feature(1), feature(2),
                                                 i);
    }
}

#endif
void Node::broadcastTf(const Vector3 &p, const Quaternion &q) const {
    tf::Transform transform;
    geometry_msgs::TransformStamped msgTf;
    msgTf.transform.translation.x = p(0);
    msgTf.transform.translation.y = p(1);
    msgTf.transform.translation.z = p(2);
    msgTf.transform.rotation.w = q.w();
    msgTf.transform.rotation.x = q.x();
    msgTf.transform.rotation.y = q.y();
    msgTf.transform.rotation.z = q.z();

    const geometry_msgs::Vector3 &pos = msgTf.transform.translation;
    const geometry_msgs::Quaternion &ori = msgTf.transform.rotation;
    transform.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
    transform.setRotation(tf::Quaternion(ori.x, ori.y, ori.z, ori.w));
    tf_broadcaster_.sendTransform(tf::StampedTransform(
            transform, ros::Time::now(), "world", "xVIO" + veh_name));
}

void Node::publishImageRate(const State &state) const {
    // Parse state
    const auto t = std::chrono::duration<double>(state.getTime()).count();
    const Vector3 p = state.getPosition();
    const Quaternion q = state.getOrientation();
    const Matrix &pose_cov = state.getPoseCovariance();

    // Just compute the header once for all messages
    // WARNING: header's seq is overwritten by ROS. There is no point in setting
    // it here.
    geometry_msgs::PoseWithCovarianceStamped msg_pose;
    msg_pose.header.stamp = ros::Time(t);
    msg_pose.header.frame_id = "world";
    msg_pose.pose.pose.position.x = p(0);
    msg_pose.pose.pose.position.y = p(1);
    msg_pose.pose.pose.position.z = p(2);
    msg_pose.pose.pose.orientation.w = q.w();
    msg_pose.pose.pose.orientation.x = q.x();
    msg_pose.pose.pose.orientation.y = q.y();
    msg_pose.pose.pose.orientation.z = q.z();
    eigen2RosPoseCov(pose_cov, msg_pose.pose.covariance);

    // Publishing pose @ image rate
    if (pub_pose_with_cov_image_rate_.getNumSubscribers())
        pub_pose_with_cov_image_rate_.publish(msg_pose);

#ifdef VERBOSE
    // Publishing tf @ image rate
    if (pub_tf_image_rate_.getNumSubscribers())
    {
        geometry_msgs::TransformStamped msg_tf;
        msg_tf.header = msg_pose.header;
        msg_tf.child_frame_id = "xVIO" + veh_name;
        msg_tf.transform.translation.x = p(0);
        msg_tf.transform.translation.y = p(1);
        msg_tf.transform.translation.z = p(2);
        msg_tf.transform.rotation.w = q.w();
        msg_tf.transform.rotation.x = q.x();
        msg_tf.transform.rotation.y = q.y();
        msg_tf.transform.rotation.z = q.z();
        pub_tf_image_rate_.publish(msg_tf);
    }

    static int msg_seq_img = 0;
    // Publishing traj @ image rate
    if (pub_traj_image_rate_.getNumSubscribers()) {
        visualization_msgs::Marker msg_traj;
        InitTrajMsgAtScale(msg_traj, 0.02);
        msg_traj.header = msg_pose.header;
        msg_traj.id = msg_seq_img++;
        msg_traj.pose.position = msg_pose.pose.pose.position;
        msg_traj.pose.orientation = msg_pose.pose.pose.orientation;
        msg_traj.lifetime = ros::Duration(traj_timeout_gui_);
        pub_traj_image_rate_.publish(msg_traj);
    }

    // Publishing state @ image rate
    if (pub_state_with_cov_image_rate_.getNumSubscribers()) {
        x_vio_ros::InertialStateWithCovarianceStamped msg_state;
        msg_state.header = msg_pose.header;

        const Eigen::VectorXd dyn_states = state.getDynamicStates();
        const int n_dyn_states = dyn_states.rows();
        msg_state.state.resize(n_dyn_states);
        for (int i = 0; i < n_dyn_states; ++i)
            msg_state.state[i] = dyn_states(i);

        const Matrix cov = state.getDynamicCovariance();
        const int n_cov = kSizeCoreErr * kSizeCoreErr;
        msg_state.cov.resize(n_cov);
        for (int i = 0; i < n_cov; ++i)
            msg_state.cov[i] = cov(i);

        pub_state_with_cov_image_rate_.publish(msg_state);
    }

    // TF broadcast
#endif
    broadcastTf(p, q);
}

/** Initializes a trajectory marker message at a given scale.
 */
void Node::InitTrajMsgAtScale(visualization_msgs::Marker &msg,
                              const double scale) const {
    msg.ns = "trajectory";
    msg.type = visualization_msgs::Marker::CUBE;
    msg.action = visualization_msgs::Marker::ADD;
    msg.scale.x = scale;
    msg.scale.y = scale;
    msg.scale.z = scale;
    msg.color.a = 1.0;
    msg.color.r = 0.; // blue
    msg.color.g = 0.;
    msg.color.b = 0.5;
    msg.lifetime = ros::Duration(0.0);
}

void Node::eigen2RosPoseCov(
        const Matrix &cov_eigen,
        geometry_msgs::PoseWithCovariance::_covariance_type &cov_ros) const {
    for (int r = 0; r < 6; r++) {
        for (int c = 0; c < 6; c++) {
            cov_ros[6 * r + c] = cov_eigen(r, c);
        }
    }
}

#ifdef MULTI_THREAD

void Node::stopNode() {
    run_threads_ = false;
}

#endif

#if defined(MULTI_UAV) && defined(DEBUG)

void Node::otherVisionImageCallback(const sensor_msgs::ImageConstPtr &img_ptr) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_ptr, enc::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    other_image_ = cv_ptr->image;
}

#endif

#ifdef MULTI_UAV

void Node::multiUavMessageCallback(
        const x_vio_ros::MessageUAV::ConstPtr &other_data_ptr) {
#ifdef DEBUG
    std::stringstream other_stamp;
    other_stamp << std::setprecision(20) << other_data_ptr->header.stamp;
    std::cout << "\033[1;31mMulti UAV Message callback on core:\033[0m"
              << sched_getcpu() << ", thread: " << std::this_thread::get_id()
              << " at timestamp: " << other_stamp.str() << std::endl;
#endif

#ifdef MULTI_THREAD
    // store the new data for processing and wake the processing thread
    message_queue_.push(other_data_ptr);
#else
    other_data_ptr_ = other_data_ptr;
    processOther();
#endif
}

void Node::multiUavRequestCallback(
        const x_vio_ros::RequestUAV::ConstPtr &other_data_ptr) {
#ifdef DEBUG
    std::stringstream other_stamp;
    std::cout << "\033[1;31mMulti UAV Request callback on core:\033[0m"
              << sched_getcpu() << ", thread: " << std::this_thread::get_id()
              << std::endl;
#endif

#ifdef MULTI_THREAD
    // store the new data for processing and wake the processing thread
    request_queue_.push(other_data_ptr);
#else
    other_request_data_ptr_ = other_data_ptr;
    processOtherRequest();
#endif
}

void Node::publishRequestUAV(const ros::Time &timestamp,
                             const cv::Mat &descriptros) {
#ifdef DEBUG
    std::stringstream other_stamp;
    std::cout << "\033[1;31mSending Request message. Descriptor size: \033[0m"
              << (descriptros.rows * descriptros.cols) << std::endl;
#endif

    static x_vio_ros::RequestUAV msg;

    msg.timestamp = timestamp;
    msg.uav_id = std::stoi(veh_name.substr(4, veh_name.size()));

    const sensor_msgs::ImagePtr desc =
            cv_bridge::CvImage(std_msgs::Header(), enc::MONO8, descriptros)
                    .toImageMsg();
    msg.descriptors = *desc;

    pub_multi_uav_request_.publish(msg);
}

void Node::publishMessageUAV(const ros::Time &timestamp, int receiver_id,
                             const std::shared_ptr<SimpleState> &state,
                             const TrackList &msckf_tracks,
                             const TrackList &slam_tracks,
                             const FeatureList &slam_features,
                             const TrackList &opp_tracks,
                             const std::vector<int> &anchor_idxs) {
    // we need to have enough point to be worthy to communicate them.
    if (msckf_tracks.size() + slam_tracks.size() + opp_tracks.size() < 10) {
        return;
    }
#ifdef DEBUG
    std::stringstream other_stamp;
    std::cout << "\033[1;31mSending Data message to\033[0m UAV" << receiver_id
              << std::endl;
#endif
    // Distributed UAV message
    x_vio_ros::MessageUAV msg;

    // set sender UAV id
    msg.uav_id = std::stoi(
            veh_name.substr(4, veh_name.size())); // remove UAV from the name to get
    // only the Id (UAV[ID])
    // create header
    msg.header.stamp = timestamp;

    // Populate the multi UAV message
    std::vector<x_vio_ros::FeatureMsg> feature_list;
    x_vio_ros::TrackMsg tmp_track;
    x_vio_ros::FeatureMsg tmp_feature;

    /*******************************
     *       TRACKS & FEATURES
     ********************************/

    // init MSCK tracks message
    for (const auto & msckf_track : msckf_tracks) { // iterate over the tracks
        tmp_track = x_vio_ros::TrackMsg();               // re-init the track

        for (size_t j = 0; j < msckf_track.size();
             j++) {                                  // iterate over the features
            tmp_feature = x_vio_ros::FeatureMsg();     // re-init the feature
            tmp_feature.x = msckf_track[j].getX(); // pass the undistort features
            tmp_feature.y = msckf_track[j].getY(); // pass the undistort features
            // don't add the descriptor to each feature. It is the same for all
            // add the descriptor to the track -> save memory

            tmp_track.features.push_back(tmp_feature);
        }
        cv::Mat d = msckf_track[0].getDescriptor();
        tmp_track.descriptor = std::vector<uchar>(d.begin<uchar>(), d.end<uchar>());

#ifdef GT_DEBUG
        // set the gt landmark
        Vector3 landmark = msckf_track.back().getLandmark();
        tmp_track.landmark.push_back(landmark[0]);
        tmp_track.landmark.push_back(landmark[1]);
        tmp_track.landmark.push_back(landmark[2]);
#endif
        tmp_track.id = msckf_track.getId();
        msg.msckf_tracks.push_back(tmp_track);
    }

    for (const auto & slam_track : slam_tracks) { // iterate over the tracks
        tmp_track = x_vio_ros::TrackMsg();              // re-init the track
        for (size_t j = 0; j < slam_track.size();
             j++) {                                 // iterate over the features
            tmp_feature = x_vio_ros::FeatureMsg();    // re-init the feature
            tmp_feature.x = slam_track[j].getX(); // pass the undistort features
            tmp_feature.y = slam_track[j].getY(); // pass the undistort features
            // don't add the descriptor to each feature. It is the same for all
            // add the descriptor to the track -> save memory

            tmp_track.features.push_back(tmp_feature);
        }
        cv::Mat d = slam_track[0].getDescriptor();
        tmp_track.descriptor = std::vector<uchar>(d.begin<uchar>(), d.end<uchar>());

#ifdef GT_DEBUG
        Vector3 landmark = slam_track.back().getLandmark();
        tmp_track.landmark.push_back(landmark[0]);
        tmp_track.landmark.push_back(landmark[1]);
        tmp_track.landmark.push_back(landmark[2]);
#endif

        tmp_track.id = slam_track.getId();
        msg.slam_tracks.push_back(tmp_track);
    }

    for (const auto & opp_track : opp_tracks) { // iterate over the tracks
        tmp_track = x_vio_ros::TrackMsg();
        if (opp_track.size() < 3) {
            continue; // re-init the track
        }
        for (size_t j = 0; j < opp_track.size();
             j++) {                                // iterate over the features
            tmp_feature = x_vio_ros::FeatureMsg();   // re-init the feature
            tmp_feature.x = opp_track[j].getX(); // pass the undistort features
            tmp_feature.y = opp_track[j].getY(); // pass the undistort features
            // don't add the descriptor to each feature. It is the same for all
            // add the descriptor to the track -> save memory

            tmp_track.features.push_back(tmp_feature);
        }
        cv::Mat d = opp_track[0].getDescriptor();
        tmp_track.descriptor = std::vector<uchar>(d.begin<uchar>(), d.end<uchar>());

#ifdef GT_DEBUG
        Vector3 landmark = opp_track.back().getLandmark();
        tmp_track.landmark.push_back(landmark[0]);
        tmp_track.landmark.push_back(landmark[1]);
        tmp_track.landmark.push_back(landmark[2]);
#endif

        tmp_track.id = opp_track.getId();
        msg.opp_tracks.push_back(tmp_track);
    }

    /*****************
     *      STATE
     ******************/
    msg.n_poses = state->nPosesMax(); // useful for when the slam feartures are
    // included in the state

    // Dynamic state
    const Vectorx dynamic_state = state->getDynamicState();
    const int n_dynamic_state = dynamic_state.rows();
    msg.dynamic_state.resize(n_dynamic_state);
    for (int i = 0; i < n_dynamic_state; i++) {
        msg.dynamic_state[i] = dynamic_state(i);
    }
    // Position state
    const Matrix positions_state =
            state->getPositionState(); // TODO: Change to vector instead of Matrix
    const int n_positions_state = positions_state.rows();
    msg.positions_state.resize(n_positions_state);
    for (int i = 0; i < n_positions_state; i++) {
        msg.positions_state[i] = positions_state(i);
    }
    // Orientation state
    // NB. These are quaternions.
    const Matrix orientations_state =
            state->getOrientationState(); //  TODO: Change to vector instead of Matrix

    const int n_orientations_state = orientations_state.rows();
    msg.orientations_state.resize(n_orientations_state);
    for (int i = 0; i < n_orientations_state; i++) {
        msg.orientations_state[i] = orientations_state(i);
    }
    // Features state
    const Matrix features_state =
            state->getFeatureState(); //  TODO: Change to vector instead of Matrix
    const int n_features_state = features_state.rows();
    msg.features_state.resize(n_features_state);
    for (int i = 0; i < n_features_state; i++) {
        msg.features_state[i] = features_state(i);
    }
    // store the anchor pose ids
    msg.anchor_ids.resize(anchor_idxs.size());
    for (size_t i = 0; i < anchor_idxs.size(); i++) {
        msg.anchor_ids[i] = anchor_idxs[i];
    }

    /*************************
     *       COVARIANCE
     **************************/

    // store the covariance data
    const Matrix cov = state->getCovariance();

    const int n_cov =
            (state->nErrorStates()) * (state->nErrorStates()); // == cov.rows*cov.cols
    msg.cov.resize(n_cov);
    for (int i = 0; i < n_cov; i++) {
        msg.cov[i] = cov(i);
    }
    communication_->publish(receiver_id, msg);
}

#endif