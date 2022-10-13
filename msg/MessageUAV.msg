# message for multi uav communication

Header    header

# UAV id
int8 uav_id

# UAV state and covariance information
float64[] dynamic_state
float64[] positions_state
float64[] orientations_state
float64[] features_state
float64[] cov
int8      n_poses
int8[]    anchor_ids

# Features and Tracks

x_vio_ros/TrackMsg[] msckf_tracks
x_vio_ros/TrackMsg[] slam_tracks
x_vio_ros/TrackMsg[] opp_tracks
# x_vio_ros/FeatureMsg[] opp_features

#x_vio_ros/FeatureMsg[] pr_features

# uint8[] pr_feature_descriptors

