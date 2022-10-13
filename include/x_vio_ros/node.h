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

#pragma once

#include <x/vio/types.h>
#include <x/vio/vio.h>

#ifdef MULTI_UAV
#include <set>
#include <x/vision/types.h>
#include <x_vio_ros/FeatureMsg.h>
#include <x_vio_ros/MessageUAV.h>
#include <x_vio_ros/RequestUAV.h>
#include <x_vio_ros/TrackMsg.h>
#include <x_vio_ros/communication.h>
#endif

#include <x_vio_ros/rviz_publisher.h>
#include <x_vio_ros/xvioConfig.h>

#include <x_vio_ros/InertialStateWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h> // Sun angle messages
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>

#ifdef MULTI_THREAD

#include <boost/lockfree/spsc_queue.hpp>

#endif

#include <optional>
#include <iostream>

namespace x {

    struct VisualData {
        std::optional<sensor_msgs::ImageConstPtr> img;
        std::optional<std_msgs::Float64MultiArray::ConstPtr> matches;

        explicit VisualData(sensor_msgs::ImageConstPtr imagePtr) : img(std::move(imagePtr)) {};

        explicit VisualData(std_msgs::Float64MultiArray::ConstPtr matchesPtr) : matches(std::move(matchesPtr)) {};

        VisualData() = delete;
    };

    typedef boost::lockfree::spsc_queue<VisualData,
            boost::lockfree::capacity<10>>
            VisualDataQueue;

#ifdef MULTI_UAV
    typedef boost::lockfree::spsc_queue<x_vio_ros::MessageUAV::ConstPtr,
                                        boost::lockfree::capacity<10>>
        MessageQueue;
    typedef boost::lockfree::spsc_queue<x_vio_ros::RequestUAV::ConstPtr,
                                        boost::lockfree::capacity<10>>
        RequestQueue;
#endif

    class Communication;

    class Node {
    public:
        Node(ros::NodeHandle &nh);

#ifdef MULTI_UAV
        Node(ros::NodeHandle &nh, std::shared_ptr<Communication>  communication);
#endif

        void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);

        void visionImageCallback(const sensor_msgs::ImageConstPtr &img);

        void visionMatchesCallback(
                const std_msgs::Float64MultiArray::ConstPtr &matches_ptr);

        void rangeCallback(const sensor_msgs::RangeConstPtr &range_msg_ptr);

        void sunAngleCallback(
                const geometry_msgs::Vector3StampedConstPtr &sun_angle_msg_ptr);

        void processVision();

        void reconfigure(x_vio::xvioConfig &config, uint32_t level);

#ifdef MULTI_UAV
        void otherVisionImageCallback(const sensor_msgs::ImageConstPtr &img);

        /**
         * @brief This callback is called by the Communication thread.
         *
         * @param other_data_ptr
         */
        void multiUavMessageCallback(
            const x_vio_ros::MessageUAV::ConstPtr &other_data_ptr);
        void multiUavRequestCallback(
            const x_vio_ros::RequestUAV::ConstPtr &other_data_ptr);
        void processOther();
        void processOtherRequest();
#endif

#ifdef MULTI_THREAD

        void stopNode();

#endif

    private:
        x::Params params_;                    ///< User parameters

        ros::Subscriber imu_sub_;             ///< Subscriber to IMU topic
        image_transport::Subscriber img_sub_; ///< Subscriber to image topic
        ros::Subscriber matches_sub_; ///< Subscriber to (visual) matches topic (e.g.
        ///< output of a simulated feature tracker)
        ros::Subscriber range_sub_;   ///< Subscriber to range topic
        ros::Subscriber sun_angle_sub_; ///< Subscriber to sun angle topic

        VisualDataQueue visual_queue_;


        bool init_at_startup_ = true;
        double traj_timeout_gui_;

        /**
         * For multi uav TF publisher
         * */
        std::string veh_name = "UAV0";

        /**
         * Publisher for pose with covariance at image rate.
         */
        ros::Publisher pub_pose_with_cov_image_rate_;

        /**
         * Publisher for state with covariance at image rate.
         */
        ros::Publisher pub_state_with_cov_image_rate_;

#ifdef MULTI_THREAD
        std::atomic<bool> run_threads_{true};
#endif

#ifdef MULTI_UAV
        int tot_message_uav_ = 0, tot_request_uav_ = 0;
        int tot_bytes_message_uav_ = 0, tot_bytes_request_uav_ = 0;

        std::shared_ptr<Communication> communication_;
        image_transport::Subscriber other_img_sub_; ///< Subscriber to image topic

        /**
         * Publisher for multi UAV request
         */
        ros::Publisher pub_multi_uav_request_;

        TrackList msckf_track_list_, slam_track_list_, opp_track_list_;
        FeatureList slam_feature_list_;
        std::vector<int> anchor_idxs_;

        x_vio_ros::MessageUAV::ConstPtr other_data_ptr_ =
            NULL; // pointer to received data
        x_vio_ros::RequestUAV::ConstPtr other_request_data_ptr_ =
            NULL;             // pointer to received data
        cv::Mat other_image_; // used to create the match image in DEBUG mode. Can be
        // empty for the release mode.
        int multi_uav_counter_ = 0;      // counter for reducing multi UAV messages
        const int reduction_factor_ = 1; // 3 means 30Hz/3 = 10 Hz
#ifdef MULTI_THREAD
        MessageQueue message_queue_;
        RequestQueue request_queue_;
#endif
#endif

#if defined(MULTI_UAV) // && defined(DEBUG)
        image_transport::Publisher
            matches_image_pub_; ///< Publisher corners response
                                // data that will be passed to the other UAV
#endif


#ifdef VERBOSE
        image_transport::Publisher tracker_pub_; ///< Publishes tracker output image
        image_transport::Publisher
                track_manager_pub_; ///< Publishes track manager output image

        ros::Publisher pub_pose_with_cov_imu_rate_; ///< Publisher for pose with covariance at IMU rate.

        ros::Publisher pub_tf_imu_rate_;   ///< Publishes 6DoF transform at IMU rate
        ros::Publisher pub_tf_image_rate_; ///< Publishes 6DoF transform at image rate
        ros::Publisher pub_traj_imu_rate;  ///< Publishes trajectory at IMU rate
        ros::Publisher pub_traj_image_rate_; ///< Publishes trajectory at image rate

        ros::Publisher pub_state_with_cov_imu_rate_; ///< Publisher for state with covariance at IMU rate.


        rvizPublisher rviz_publisher_;
#endif
        mutable tf::TransformBroadcaster tf_broadcaster_;

        VIO vio_;             ///< XIO base instance
        unsigned int seq_{0};

        void loadParamsWithHandle(const ros::NodeHandle &nh);

        void setInterfaceWithHandle(ros::NodeHandle &nh);

        /**
         * Publish state content on ROS at image rate.
         *
         * @param[in] state Input state.
         */
        void publishImageRate(const State &state) const;

#ifdef MULTI_UAV
        void publishRequestUAV(const ros::Time &timestamp,
                               const cv::Mat &descriptros);
        void publishMessageUAV(const ros::Time &timestamp, const int receiver_id,
                               const std::shared_ptr<SimpleState> &state,
                               const TrackList &msckf_tracks,
                               const TrackList &slam_tracks,
                               const FeatureList &slam_features,
                               const TrackList &opp_tracks,
                               const std::vector<int> &anchor_idxs);
#endif

        void InitTrajMsgAtScale(visualization_msgs::Marker &msg,
                                const double scale) const;

#ifdef VERBOSE

        /**
         * Publishes ROS messages at IMU rate.
         *
         * @param[in] state The state propagated with the last IMU message.
         */
        void publishImuRate(const State &state) const;

        void publishSLAMFeatures(const State &state);

#endif

        /**
         * Broadcats a ROS TF transform from the input position and orientation.
         *
         * @param[in] p Position
         * @param[in] q Orientation unit quaternion.
         */
        void broadcastTf(const Vector3 &p, const Quaternion &q) const;

        void eigen2RosPoseCov(
                const Matrix &cov_eigen,
                geometry_msgs::PoseWithCovariance::_covariance_type &cov_ros) const;
    };
} // end namespace x
