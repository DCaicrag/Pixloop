// Edit from https://github.com/autowarefoundation/autoware.universe/tree/main/vehicle/accel_brake_map_calibrator/accel_brake_map_calibrator
//
// Copyright 2022 Guanghang Cai
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ACCEL_BRAKE_MAP_CALIBRATOR__ACCEL_BRAKE_MAP_CALIBRATOR_NODE_HPP_
#define ACCEL_BRAKE_MAP_CALIBRATOR__ACCEL_BRAKE_MAP_CALIBRATOR_NODE_HPP_

#include "raw_vehicle_cmd_converter/accel_map.h"
#include "raw_vehicle_cmd_converter/brake_map.h"
#include "ros/ros.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pix_hooke_driver_msgs/v2a_drivestafb_530.h"
#include "pix_hooke_driver_msgs/v2a_brakestafb_531.h"
#include "pix_hooke_driver_msgs/v2a_steerstafb_532.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

#include "accel_brake_map_calibrator/SaveCalibration.h"

#include <fstream>
#include <memory>
#include <string>
#include <vector>

struct DataStamped
{
  DataStamped(const double _data, const ros::Time & _data_time)
  : data{_data}, data_time{_data_time}
  {
  }
  double data;
  ros::Time data_time;
};

using DataStampedPtr = std::shared_ptr<DataStamped>;

class AccelBrakeMapCalibrator
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::string csv_default_map_dir_;
  ros::Publisher original_map_occ_pub_;
  ros::Publisher update_map_occ_pub_;
  ros::Publisher original_map_raw_pub_;
  ros::Publisher update_map_raw_pub_;
  ros::Publisher data_count_pub_;
  ros::Publisher data_count_with_self_pose_pub_;
  ros::Publisher data_ave_pub_;
  ros::Publisher data_std_pub_;
  ros::Publisher index_pub_;
  ros::Publisher update_suggest_pub_;
  ros::Publisher debug_pub_;
  ros::Publisher current_map_error_pub_;
  ros::Publisher updated_map_error_pub_;
  ros::Publisher map_error_ratio_pub_;

  ros::Subscriber velocity_sub_;
  ros::Subscriber steer_sub_;
  ros::Subscriber throttle_sub_;
  ros::Subscriber brake_sub_;
  ros::Subscriber imu_sub_;

  ros::ServiceServer save_calibration_server_;

  // Timer
  ros::Timer timer_;
  ros::Timer timer_output_csv_;

  void initTimer(double period_s);
  void initOutputCSVTimer(double period_s);

  // data storage
  std::shared_ptr<geometry_msgs::TwistStamped> twist_ptr_;
  std::vector<std::shared_ptr<geometry_msgs::TwistStamped>> twist_vec_;
  std::vector<DataStampedPtr> accel_pedal_vec_;  // for delayed pedal
  std::vector<DataStampedPtr> brake_pedal_vec_;  // for delayed pedal
  DataStampedPtr steer_ptr_;
  DataStampedPtr accel_pedal_ptr_;
  DataStampedPtr brake_pedal_ptr_;
  DataStampedPtr delayed_accel_pedal_ptr_;
  DataStampedPtr delayed_brake_pedal_ptr_;
  DataStampedPtr imu_pitch_ptr_;

  // params
  int get_pitch_method_;
  int update_method_;
  double covariance_;
  double acceleration_ = 0.0;
  double acceleration_time_;
  double pre_acceleration_ = 0.0;
  double pre_acceleration_time_;
  double jerk_ = 0.0;
  double accel_pedal_speed_ = 0.0;
  double brake_pedal_speed_ = 0.0;
  double pitch_ = 0.0;
  double update_hz_;
  double max_steer_;
  double velocity_min_threshold_;
  double velocity_diff_threshold_;
  double pedal_diff_threshold_;
  double max_steer_threshold_;
  double max_pitch_threshold_;
  double max_jerk_threshold_;
  double pedal_velocity_thresh_;
  double map_update_gain_;
  double max_accel_;
  double min_accel_;
  double pedal_to_accel_delay_;
  int max_data_count_;
  bool auto_update_;
  bool pedal_accel_graph_output_ = false;
  bool progress_file_output_ = false;
  double update_suggest_thresh_;
  std::string csv_calibrated_map_dir_;
  std::string output_accel_file_;
  std::string output_brake_file_;

  // log 
  std::ofstream output_log_;

  // for calculating differential of msg
  const double dif_twist_time_ = 0.2;   // 200ms
  const double dif_pedal_time_ = 0.16;  // 160ms
  const std::size_t twist_vec_max_size_ = 100;
  const std::size_t pedal_vec_max_size_ = 100;
  const double timeout_sec_ = 0.1;
  const int max_data_save_num_ = 10000;
  const double map_resolution_ = 0.1;
  const double max_jerk_ = 5.0;

  // Algorithm
  AccelMap accel_map_;
  BrakeMap brake_map_;

  // for evaluation
  AccelMap new_accel_map_;
  BrakeMap new_brake_map_;
  std::vector<double> part_original_accel_mse_que_;
  std::vector<double> full_original_accel_mse_que_;
  std::vector<double> new_accel_mse_que_;
  std::size_t full_mse_que_size_ = 100000;
  std::size_t part_mse_que_size_ = 3000;
  double full_original_accel_rmse_ = 0.0;
  double part_original_accel_rmse_ = 0.0;
  double new_accel_rmse_ = 0.0;

  // Accel / Brake Map value
  std::vector<std::vector<double>> accel_map_value_;
  std::vector<std::vector<double>> brake_map_value_;
  std::vector<std::vector<double>> update_accel_map_value_;
  std::vector<std::vector<double>> update_brake_map_value_;
  std::vector<std::vector<std::vector<double>>> map_value_data_;
  std::vector<double> accel_vel_index_;
  std::vector<double> brake_vel_index_;
  std::vector<double> accel_pedal_index_;
  std::vector<double> brake_pedal_index_;
  bool update_success_;
  int update_success_count_ = 0;
  int update_count_ = 0;
  int lack_of_data_count_ = 0;
  int failed_to_get_pitch_count_ = 0;
  int too_large_pitch_count_ = 0;
  int too_low_speed_count_ = 0;
  int too_large_steer_count_ = 0;
  int too_large_jerk_count_ = 0;
  int invalid_acc_brake_count_ = 0;
  int too_large_pedal_spd_count_ = 0;
  int update_fail_count_ = 0;

  // for map update
  double map_offset_ = 0.0;
  double map_coef_ = 1.0;
  const double forgetting_factor_ = 0.999;
  const double coef_update_skip_thresh_ = 0.1;

  // callback functions
  void callbackVelocity(const geometry_msgs::TwistStampedConstPtr & msg);
  void callbackSteer(const pix_hooke_driver_msgs::v2a_steerstafb_532::ConstPtr & msg);
  void callbackThrottle(const pix_hooke_driver_msgs::v2a_drivestafb_530::ConstPtr & msg);
  void callbackBrake(const pix_hooke_driver_msgs::v2a_brakestafb_531::ConstPtr & msg);
  void callbackImu(const sensor_msgs::Imu::ConstPtr & msg);
  void timerCallback(const ros::TimerEvent & e);
  void timerCallbackOutputCSV(const ros::TimerEvent & e);

  // service response functions 
  bool saveCalibration(accel_brake_map_calibrator::SaveCalibration::Request &req,
                        accel_brake_map_calibrator::SaveCalibration::Response &res);
  
  double lowpass(const double original, const double current, const double gain = 0.8);
  double getPedalSpeed(
    const DataStampedPtr & prev_pedal, const DataStampedPtr & current_pedal,
    const double prev_pedal_speed);
  double getAccel(
    const std::shared_ptr<geometry_msgs::TwistStamped> & prev_twist,
    const std::shared_ptr<geometry_msgs::TwistStamped> & current_twist);
  double getJerk();
  bool getCurrentPitch(double * pitch);
  bool indexValueSearch(
    const std::vector<double> value_index, const double value, const double value_thresh,
    int * searched_index);
  int nearestValueSearch(const std::vector<double> value_index, const double value);
  int nearestPedalSearch();
  int nearestVelSearch();
  void takeConsistencyOfAccelMap();
  void takeConsistencyOfBrakeMap();
  void executeUpdate(
    const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
    const int brake_pedal_index, const int brake_vel_index);
  bool updateEachValOffset(
    const bool accel_mode, const int accel_pedal_index, const int accel_vel_index,
    const int brake_pedal_index, const int brake_vel_index, const double measured_acc,
    const double map_acc);
  void updateTotalMapOffset(const double measured_acc, const double map_acc);
  bool updateAccelBrakeMap();
  void publishFloat32(const std::string publish_type, const double val);
  void publishUpdateSuggestFlag();
  double getPitchCompensatedAcceleration();
  void executeEvaluation();
  double calculateEstimatedAcc(
    const double throttle, const double brake, const double vel, AccelMap & accel_map,
    BrakeMap & brake_map);
  double calculateAccelSquaredError(
    const double throttle, const double brake, const double vel, AccelMap & accel_map,
    BrakeMap & brake_map);
  std::vector<double> getMapColumnFromUnifiedIndex(
    const std::vector<std::vector<double>> & accel_map_value,
    const std::vector<std::vector<double>> & brake_map_value, const std::size_t index);
  double getPedalValueFromUnifiedIndex(const std::size_t index);
  int getUnifiedIndexFromAccelBrakeIndex(const bool accel_map, const std::size_t index);
  template <class T>
  void pushDataToVec(const T data, const std::size_t max_size, std::vector<T> * vec);
  template <class T>
  T getNearestTimeDataFromVec(
    const T base_data, const double back_time, const std::vector<T> & vec);
  DataStampedPtr getNearestTimeDataFromVec(
    DataStampedPtr base_data, const double back_time, const std::vector<DataStampedPtr> & vec);
  double getAverage(const std::vector<double> & vec);
  double getStandardDeviation(const std::vector<double> & vec);
  bool isTimeout(const ros::Time & stamp, const ros::Time & current_time, const double timeout_sec);
  
  nav_msgs::OccupancyGrid getOccMsg(
    const std::string frame_id, const double height, const double width, const double resolution,
    const std::vector<int8_t> & map_value);

  /* Debug */
  std_msgs::Float32MultiArray debug_values_;
  enum DBGVAL {
    CURRENT_SPEED = 0,
    CURRENT_ACCEL_PEDAL = 1,
    CURRENT_BRAKE_PEDAL = 2,
    CURRENT_RAW_ACCEL = 3,
    CURRENT_ACCEL = 4,
    CURRENT_RAW_ACCEL_SPEED = 5,
    CURRENT_ACCEL_SPEED = 6,
    CURRENT_RAW_BRAKE_SPEED = 7,
    CURRENT_BRAKE_SPEED = 8,
    CURRENT_RAW_PITCH = 9,
    CURRENT_PITCH = 10,
    CURRENT_STEER = 11,
    SUCCESS_TO_UPDATE = 12,
    CURRENT_JERK = 13,
  };
  static constexpr unsigned int num_debug_values_ = 14;
  void publishMap(
    const std::vector<std::vector<double>> accel_map_value,
    const std::vector<std::vector<double>> brake_map_value, const std::string publish_type);
  void publishCountMap();
  void publishIndex();
  bool writeMapToCSV(
    std::vector<double> vel_index, std::vector<double> pedal_index,
    std::vector<std::vector<double>> value_map, std::string filename);
  void addIndexToCSV(std::ofstream * csv_file);
  void addLogToCSV(
    std::ofstream * csv_file, const double & timestamp, const double velocity, const double accel,
    const double pitched_accel, const double accel_pedal, const double brake_pedal,
    const double accel_pedal_speed, const double brake_pedal_speed, const double pitch,
    const double steer, const double jerk, const double full_original_accel_mse,
    const double part_original_accel_mse, const double new_accel_mse);

  enum GET_PITCH_METHOD { TF = 0, IMU = 1, NONE = 2 };

  enum UPDATE_METHOD {
    UPDATE_OFFSET_EACH_CELL = 0,
    UPDATE_OFFSET_TOTAL = 1,
  };

public:
  AccelBrakeMapCalibrator();
};

#endif  // ACCEL_BRAKE_MAP_CALIBRATOR__ACCEL_BRAKE_MAP_CALIBRATOR_NODE_HPP_
