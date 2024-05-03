#pragma once

#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "fifo.h"
#include "input.h"
#include "laser_ts.h"

inline float ToRad(float degree) { return degree * M_PI / 180.0f; }

#define PANDAR128_READ_PACKET_SIZE (1800)
#define PANDARQT128_READ_PACKET_SIZE (200)
#define PANDAR80_READ_PACKET_SIZE (500)
#define PANDAR90_READ_PACKET_SIZE (200)
#define PANDAR64S_READ_PACKET_SIZE (400)
#define PANDAR40S_READ_PACKET_SIZE (100)
#define PANDAR_LASER_NUMBER_INDEX (6)
#define PANDAR_MAJOR_VERSION_INDEX (2)

#ifndef CIRCLE
#define CIRCLE (36000)
#endif

#define PANDARSDK_TCP_COMMAND_PORT (9347)
#define LIDAR_NODE_TYPE "lidar"
#define LIDAR_ANGLE_SIZE_10 (10)
#define LIDAR_ANGLE_SIZE_18 (18)
#define LIDAR_ANGLE_SIZE_20 (20)
#define LIDAR_ANGLE_SIZE_40 (40)
#define LIDAR_ANGLE_SIZE_80 (80)
#define LIDAR_RETURN_BLOCK_SIZE_1 (1)
#define LIDAR_RETURN_BLOCK_SIZE_2 (2)

#define GPS_PACKET_SIZE (512)
#define GPS_PACKET_FLAG_SIZE (2)
#define GPS_PACKET_YEAR_SIZE (2)
#define GPS_PACKET_MONTH_SIZE (2)
#define GPS_PACKET_DAY_SIZE (2)
#define GPS_PACKET_HOUR_SIZE (2)
#define GPS_PACKET_MINUTE_SIZE (2)
#define GPS_PACKET_SECOND_SIZE (2)
#define GPS_ITEM_NUM (7)

#define PANDAR128_LASER_NUM (128)
#define PANDAR64S_LASER_NUM (64)
#define PANDAR40S_LASER_NUM (40)
#define PANDAR80_LASER_NUM (80)
#define PANDAR90_LASER_NUM (90)
#define PANDAR128_BLOCK_NUM (2)
#define MAX_BLOCK_NUM (8)
#define PANDAR128_DISTANCE_UNIT (0.004)
#define PANDAR128_SOB_SIZE (2)
#define PANDAR128_VERSION_MAJOR_SIZE (1)
#define PANDAR128_VERSION_MINOR_SIZE (1)
#define PANDAR128_HEAD_RESERVED1_SIZE (2)
#define PANDAR128_LASER_NUM_SIZE (1)
#define PANDAR128_BLOCK_NUM_SIZE (1)
#define PANDAR128_ECHO_COUNT_SIZE (1)
#define PANDAR128_ECHO_NUM_SIZE (1)
#define PANDAR128_HEAD_RESERVED2_SIZE (2)
#define PANDAR128_HEAD_SIZE                                                    \
  (PANDAR128_SOB_SIZE + PANDAR128_VERSION_MAJOR_SIZE +                         \
   PANDAR128_VERSION_MINOR_SIZE + PANDAR128_HEAD_RESERVED1_SIZE +              \
   PANDAR128_LASER_NUM_SIZE + PANDAR128_BLOCK_NUM_SIZE +                       \
   PANDAR128_ECHO_COUNT_SIZE + PANDAR128_ECHO_NUM_SIZE +                       \
   PANDAR128_HEAD_RESERVED2_SIZE)
#define PANDAR128_AZIMUTH_SIZE (2)
#define DISTANCE_SIZE (2)
#define INTENSITY_SIZE (1)
#define CONFIDENCE_SIZE (1)
#define WEIGHT_FACTOR_SIZE (1)
#define ENVLIGHT_SIZE (1)
#define PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE (DISTANCE_SIZE + INTENSITY_SIZE)
#define PANDAR128_UNIT_WITH_CONFIDENCE_SIZE                                    \
  (DISTANCE_SIZE + INTENSITY_SIZE + CONFIDENCE_SIZE)
#define PANDAR128_BLOCK_SIZE                                                   \
  (PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * PANDAR128_LASER_NUM +              \
   PANDAR128_AZIMUTH_SIZE)
#define PANDAR128_TAIL_RESERVED1_SIZE (3)
#define PANDAR128_TAIL_RESERVED2_SIZE (3)
#define PANDAR128_SHUTDOWN_FLAG_SIZE (1)
#define PANDAR128_TAIL_RESERVED3_SIZE (3)
#define PANDAR128_MOTOR_SPEED_SIZE (2)
#define PANDAR128_TS_SIZE (4)
#define PANDAR128_RETURN_MODE_SIZE (1)
#define PANDAR128_FACTORY_INFO (1)
#define PANDAR128_UTC_SIZE (6)
#define PANDAR128_TAIL_SIZE                                                    \
  (PANDAR128_TAIL_RESERVED1_SIZE + PANDAR128_TAIL_RESERVED2_SIZE +             \
   PANDAR128_SHUTDOWN_FLAG_SIZE + PANDAR128_TAIL_RESERVED3_SIZE +              \
   PANDAR128_MOTOR_SPEED_SIZE + PANDAR128_TS_SIZE +                            \
   PANDAR128_RETURN_MODE_SIZE + PANDAR128_FACTORY_INFO + PANDAR128_UTC_SIZE)
#define PANDAR128_PACKET_SIZE                                                  \
  (PANDAR128_HEAD_SIZE + PANDAR128_BLOCK_SIZE * PANDAR128_BLOCK_NUM +          \
   PANDAR128_TAIL_SIZE)
#define PANDAR128_SEQ_NUM_SIZE (4)
#define PANDAR128_PACKET_SEQ_NUM_SIZE                                          \
  (PANDAR128_PACKET_SIZE + PANDAR128_SEQ_NUM_SIZE)
#define PANDAR128_WITHOUT_CONF_UNIT_SIZE (DISTANCE_SIZE + INTENSITY_SIZE)

#define PANDAR128_TASKFLOW_STEP_SIZE (225)
#define PANDAR64S_TASKFLOW_STEP_SIZE (225)
#define PANDAR40S_TASKFLOW_STEP_SIZE (225)
#define PANDAR80_TASKFLOW_STEP_SIZE (250)
#define PANDARQT128_TASKFLOW_STEP_SIZE (100)
#define PANDAR128_CRC_SIZE (4)
#define PANDAR128_FUNCTION_SAFETY_SIZE (17)
#define HS_LIDAR_QT128_COORDINATE_CORRECTION_ODOG (0.0354)
#define HS_LIDAR_QT128_COORDINATE_CORRECTION_OGOT (-0.0072)

#define ETHERNET_MTU (1500)

#define CIRCLE_ANGLE (36000)
#define MOTOR_SPEED_600 (600)
#define MOTOR_SPEED_900 (900)
#define MOTOR_SPEED_1200 (1200)
#define MAX_REDUNDANT_POINT_NUM (1000)

#define MIN_DISTANCE (0.2f)
#define MAX_DISTANCE (200.0f)

namespace pcl {

struct __attribute__((aligned(16))) PointXYZIRT {
  float x;
  float y;
  float z;
  std::uint8_t i;
  std::uint8_t r;
  std::uint16_t t;
};

} // namespace pcl

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIRT,
                         (float, x, x)
                         (float, y, y)
                         (float, z, z)
                         (std::uint8_t, i, i)
                         (std::uint8_t, r, r)
                         (std::uint16_t, t, t))
// clang-format on

namespace pandar128 {

struct __attribute__((__packed__)) Pandar128Unit {
  uint16_t u16Distance;
  uint8_t u8Intensity;
};

struct __attribute__((__packed__)) Pandar128Block {
  uint16_t fAzimuth;
  Pandar128Unit units[PANDAR128_LASER_NUM];
};

struct __attribute__((__packed__)) Pandar128HeadVersion14 {
  uint16_t u16Sob;
  uint8_t u8VersionMajor;
  uint8_t u8VersionMinor;
  uint16_t u16Reserve1;
  uint8_t u8LaserNum;
  uint8_t u8BlockNum;
  uint8_t u8EchoCount;
  uint8_t u8DistUnit;
  uint8_t u8EchoNum;
  uint8_t u8Flags;
  inline int unitSize() const {
    return PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE +
           (hasConfidence() ? CONFIDENCE_SIZE : 0) +
           (hasWeightFactor() ? WEIGHT_FACTOR_SIZE : 0) +
           (hasWeightFactor() ? ENVLIGHT_SIZE : 0);
  };
  inline bool hasSeqNum() const { return u8Flags & 1; }
  inline bool hasImu() const { return u8Flags & 2; }
  inline bool hasFunctionSafety() const { return u8Flags & 4; }
  inline bool hasSignature() const { return u8Flags & 8; }
  inline bool hasConfidence() const { return u8Flags & 0x10; }
  inline bool hasWeightFactor() const { return u8Flags & 0x20; }
  inline bool hasEnvLight() const { return u8Flags & 0x40; }
};

struct __attribute__((__packed__)) Pandar128FailSafe {
  // TODO
  uint8_t nReserved[17];
};

struct __attribute__((__packed__)) Pandar128TailVersion14 {
  uint8_t nReserved1[3];
  uint8_t nReserved2[3];
  uint8_t nReserved3[3];
  uint16_t nAzimuthFlag;
  uint8_t nShutdownFlag;
  uint8_t nReturnMode;
  uint16_t nMotorSpeed;
  uint8_t nUTCTime[6];
  uint32_t nTimestamp;
  uint8_t nFactoryInfo;
  uint32_t nSeqNum;
  // TODO: imu info
  inline uint8_t getAngleState(int blockIndex) const {
    return (nAzimuthFlag >> (2 * (7 - blockIndex))) & 0x03;
  }
  inline uint8_t getOperationMode() const { return nShutdownFlag & 0x0f; }
};

struct __attribute__((__packed__)) Pandar128PacketVersion14 {
  Pandar128HeadVersion14 head;
  Pandar128Block blocks[PANDAR128_BLOCK_NUM];
  Pandar128FailSafe failsafe;
  Pandar128TailVersion14 tail;
};

struct Driver {
  Driver(const YAML::Node &cfg);
  bool start();

  enum Status {
    IDLE = 0,
    INIT,
    RUNNING,
    STOPPED,
  };

protected:
  bool init();
  bool run();

  // for async task
  void loadConfigFile();
  void packetGeneration();
  void packetProcessing();
  void pointsPublishing();

  // handler
  void handlePacket(const std::shared_ptr<PandarPacket> &pkt);
  void handleDataPacket(const std::shared_ptr<PandarPacket> &pkt);
  void handleGpsPacket(const std::shared_ptr<PandarPacket> &pkt);
  void handleDataBlock(const Pandar128Block &blk, const int blk_id,
                       const Pandar128PacketVersion14 &pkt);

  // load config
  bool loadCalibration();
  bool loadFireTime();

  using PointCloudXYZIRT = pcl::PointCloud<pcl::PointXYZIRT>;
  using TimePcdPair = std::pair<double, PointCloudXYZIRT>;

  // helper
  int loadCalibrationFromString(const std::string &calibration_content);
  double getPacketTime(const Pandar128PacketVersion14 &pkt) const;
  bool checkBlock(const double time, const float yaw);
  std::shared_ptr<TimePcdPair> makeNewPcd(const double time) const;

  inline float angleDiff(const float a1, const float a2) const;
  inline int angleToIndex(const float angle) const;
  inline float fastCos(const float angle) const;
  inline float fastSin(const float angle) const;

protected:
  Status status_ = IDLE;

  std::shared_ptr<Input> input_ = nullptr;

  static constexpr int QueueSize = PANDAR128_READ_PACKET_SIZE;

  fifo<std::shared_ptr<PandarPacket>, QueueSize> busy_queue_;
  fifo<std::shared_ptr<PandarPacket>, QueueSize> free_queue_;

  fifo<std::shared_ptr<TimePcdPair>, 10> pcd_queue_;

  std::shared_ptr<TimePcdPair> cur_pcd_;
  std::list<float> blk_yaws_;
  int cool_down_ = 0;

  bool calibration_loaded_ = false;
  bool firetime_loaded_ = false;

  std::array<float, PANDAR128_LASER_NUM> pitchAngles_;
  std::array<float, PANDAR128_LASER_NUM> yawAngles_;
  std::array<float, CIRCLE> cos_;
  std::array<float, CIRCLE> sin_;

  void *tcp_command_client_ = nullptr;
  LasersTSOffset laser_offset_;
  ros::NodeHandlePtr nh_ = nullptr;
  ros::Publisher points_pub_;

  // param
  std::string pcap_file_ = "";
  std::string calibration_file_ = "";
  std::string firetime_file_ = "";
  std::string channel_config_file_ = "";
  std::string devip_str_ = "";
  bool do_coordinate_correction_ = false;
  double start_angle_ = 0.0; // unit deg
};

} // namespace pandar128
