#include "sensor_msgs/PointCloud2.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <fstream>
#include <future>
#include <iostream>

#include "driver.h"
#include "tcp_command_client.h"
#include "yaml_extension.h"

namespace pandar128 {

Driver::Driver(const YAML::Node &cfg) {
  TryGetParameter(cfg, "start_angle", start_angle_);
  TryGetParameter(cfg, "pcap_file", pcap_file_);
  TryGetParameter(cfg, "calibration_file", calibration_file_);
  TryGetParameter(cfg, "firetime_file", firetime_file_);
  TryGetParameter(cfg, "channel_config_file", channel_config_file_);
  TryGetParameter(cfg, "do_coordinate_correction", do_coordinate_correction_);
  TryGetParameter(cfg["input"], "device_ip", devip_str_);

  // init the free_queue_
  for (auto i = 0; i < QueueSize; ++i) {
    free_queue_.push(std::make_shared<PandarPacket>());
  }
}

bool Driver::start() {
  if (!init()) {
    std::cerr << "Failed to init" << std::endl;
    exit(-1);
  }

  if (!run()) {
    std::cerr << "Failed to run" << std::endl;
    exit(-1);
  }

  return true;
}

bool Driver::init() {
  // init ros related
  ros::init(ros::M_string(), "pandar128");
  nh_ = ros::NodeHandlePtr(new ros::NodeHandle);

  // init tcp_command_client if pcap file not present
  if (pcap_file_ == "") {
    tcp_command_client_ =
        TcpCommandClientNew(devip_str_.c_str(), PANDARSDK_TCP_COMMAND_PORT);
    if (!tcp_command_client_) {
      std::cerr << "Failed to create tcp command client" << std::endl;
      return false;
    }
  }

  // init cos,sin table
  for (auto i = 0; i < CIRCLE; ++i) {
    float angle = float(i) * 0.01;
    cos_[i] = cosf(ToRad(angle));
    sin_[i] = sinf(ToRad(angle));
  }

  // advertise
  points_pub_ =
      nh_->advertise<sensor_msgs::PointCloud2>("/pandar128_points", 10);

  // update status to INIT
  status_ = INIT;

  return true;
}

bool Driver::run() {
  auto init = std::async(std::launch::async, [&]() { loadConfigFile(); });
  auto pkt_gen = std::async(std::launch::async, [&]() { packetGeneration(); });
  auto pkt_proc = std::async(std::launch::async, [&]() { packetProcessing(); });
  auto pts_pub = std::async(std::launch::async, [&]() { pointsPublishing(); });

  ros::spin();

  status_ = STOPPED;
  std::cout << "Stopping driver..." << std::endl;

  return true;
}

void Driver::packetGeneration() {
  bool succeed;
  std::shared_ptr<PandarPacket> pkt;
  while (status_ == RUNNING) {
    // get an slot from free_queue_
    free_queue_.pop(pkt);

    // get packet from input and push into busy queue
    do {
      // loop until we get an valid packet
      while (1 == input_->getPacket(pkt.get())) {
        std::cout << "drop invalid packet" << std::endl;
      }

      // try to push into busy queue
      succeed = busy_queue_.push(pkt, 0);
      if (!succeed) {
        std::cerr << "busy_queue_ overrun, packet maybe lost" << std::endl;
      }
    } while (!succeed);
  }
}

void Driver::packetProcessing() {
  std::shared_ptr<PandarPacket> pkt;
  while (status_ == RUNNING) {
    busy_queue_.pop(pkt);
    handlePacket(pkt);
  }
}

void Driver::pointsPublishing() {
  int seq = 0;
  std::shared_ptr<std::pair<double, PointCloudXYZIRT>> pcd_pair;
  while (status_ == RUNNING) {
    pcd_queue_.pop(pcd_pair);
    const auto &[time, pcd] = *pcd_pair;
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = "pandar128";
    msg.header.seq = seq++;
    msg.header.stamp.fromSec(time);
    pcl::toROSMsg(pcd, msg);
    points_pub_.publish(msg);
  }
}

void Driver::handlePacket(const std::shared_ptr<PandarPacket> &pkt) {
  if (pkt->size == 512) {
    handleGpsPacket(pkt);
  } else {
    // TODO: check if time locked
    handleDataPacket(pkt);
  }

  // return the pkt to free_queue_
  if (!free_queue_.push(pkt, 0)) {
    std::cerr << "Failed to push into free_queue_, this should never happen"
              << std::endl;
    exit(-1);
  }
}

void Driver::handleDataPacket(const std::shared_ptr<PandarPacket> &packet) {
  auto pkt = reinterpret_cast<Pandar128PacketVersion14 *>(packet->data);

  for (auto blk_id = 0; blk_id < pkt->head.u8BlockNum; ++blk_id) {
    handleDataBlock(pkt->blocks[blk_id], blk_id, *pkt);
  }
}

double Driver::getPacketTime(const Pandar128PacketVersion14 &pkt) const {
  struct tm t { // clang-format off
    .tm_sec  = pkt.tail.nUTCTime[5],
    .tm_min  = pkt.tail.nUTCTime[4],
    .tm_hour = pkt.tail.nUTCTime[3],
    .tm_mday = pkt.tail.nUTCTime[2],
    .tm_mon  = pkt.tail.nUTCTime[1] - 1,
    .tm_year = pkt.tail.nUTCTime[0],
    .tm_isdst = 0,
  }; // clang-format on

  double unix_second = static_cast<double>(mktime(&t));
  double pkt_time = unix_second + double(pkt.tail.nTimestamp) * 1e-6;
  return pkt_time;
}

void Driver::handleDataBlock(const Pandar128Block &blk, const int blk_id,
                             const Pandar128PacketVersion14 &pkt) {
  const int ret_mode = pkt.tail.nReturnMode;
  const int op_mode = pkt.tail.getOperationMode();
  const int blk_state = pkt.tail.getAngleState(blk_id);

  double pkt_time = getPacketTime(pkt);

  int blk_offset_ns =
      laser_offset_.getBlockTS(blk_id, ret_mode, op_mode, PANDAR128_LASER_NUM);
  double blk_time = pkt_time + blk_offset_ns * 1e-9;

  float blk_yaw = float(blk.fAzimuth) / 100.0f;

  if (!checkBlock(blk_time, blk_yaw)) {
    return;
  }

  // process laser in block
  for (auto lid = 0; lid < PANDAR128_LASER_NUM; ++lid) {
    const auto &ray = blk.units[lid];
    float distance = float(ray.u16Distance) * PANDAR128_DISTANCE_UNIT;
    if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
      continue;
    }

    float time_offset =
        laser_offset_.getTSOffset(lid, op_mode, blk_state, distance, 1);

    float yaw_original = blk_yaw + yawAngles_[lid];
    float yaw = yaw_original + laser_offset_.getAngleOffset(
                                   time_offset, pkt.tail.nMotorSpeed, 1);

    float pitch = pitchAngles_[lid];
    if (do_coordinate_correction_) {
      pitch += laser_offset_.getPitchOffset("", pitch, distance);
    }

    float xy_distance = distance * fastCos(pitch);

    if (do_coordinate_correction_) {
      yaw += laser_offset_.getAzimuthOffset("", yaw_original, blk_yaw,
                                            xy_distance);
    }

    double ray_time = blk_time + time_offset * 1e-9;

    pcl::PointXYZIRT p;
    p.x = xy_distance * fastSin(yaw);
    p.y = xy_distance * fastCos(yaw);
    p.z = distance * fastSin(pitch);
    p.i = ray.u8Intensity;
    p.r = lid + 1;
    p.t = (ray_time - cur_pcd_->first) / 2;
    cur_pcd_->second.emplace_back(p);
  }
}

float Driver::angleDiff(const float a1, const float a2) const {
  auto diff = a1 - a2;
  if (diff < -180) {
    diff += 360;
  } else if (diff >= 180) {
    diff -= 360;
  }
  return diff;
}

int Driver::angleToIndex(const float angle) const {
  int idx = floor(angle * 100 + 0.5);
  if (idx >= CIRCLE) {
    idx -= CIRCLE;
  } else if (idx < 0) {
    idx += CIRCLE;
  }
  return idx;
}

float Driver::fastCos(const float angle) const {
  return cos_[angleToIndex(angle)];
}

float Driver::fastSin(const float angle) const {
  return sin_[angleToIndex(angle)];
}

void Driver::handleGpsPacket(const std::shared_ptr<PandarPacket> &packet) {
  // TODO: handle gps
}

bool Driver::checkBlock(const double time, const float yaw) {
  if (cool_down_ > 0) {
    --cool_down_;
  }

  blk_yaws_.emplace_back(yaw);
  if (blk_yaws_.size() < 4) {
    return false;
  }

  if (blk_yaws_.size() > 4) {
    blk_yaws_.pop_front();
  }

  // check if we cross the start angle
  float left_angle = blk_yaws_.front();
  float right_angle = blk_yaws_.back();
  float diff1 = angleDiff(left_angle, start_angle_);
  float diff2 = angleDiff(right_angle, start_angle_);
  float prod = diff1 * diff2;
  if (prod < 0 && abs(prod) < 10 && cool_down_ <= 0) {
    cool_down_ = 10;
    if (cur_pcd_) {
      if (!pcd_queue_.push(cur_pcd_, 0)) {
        std::cerr << "pcd_queue_ overrun, drop pcd" << std::endl;
      }
    }
    cur_pcd_ = makeNewPcd(time);
  } else if (cur_pcd_ == nullptr) {
    return false;
  }

  return true;
}

std::shared_ptr<Driver::TimePcdPair>
Driver::makeNewPcd(const double time) const {
  auto pair = std::make_shared<TimePcdPair>();
  pair->first = time;
  return pair;
}

bool Driver::loadCalibration() {
  if (calibration_loaded_) {
    return true;
  }

  if (pcap_file_ == "") {
    char *buffer = nullptr;
    uint32_t len = 0;
    int ret = TcpCommandGetLidarCalibration(tcp_command_client_, &buffer, &len);
    if (ret != 0 || buffer == nullptr) {
      return false;
    }
    std::string content(buffer);
    free(buffer);
    if (0 != loadCalibrationFromString(content)) {
      return false;
    }
  } else {
    std::ifstream fin(calibration_file_);
    if (!fin.is_open()) {
      return false;
    }
    std::string content;
    fin.seekg(0, std::ios::end);
    auto length = fin.tellg();
    content.resize(length);
    fin.seekg(0, std::ios::beg);
    fin.read(content.data(), length);
    fin.close();
    if (0 != loadCalibrationFromString(content)) {
      return false;
    }
  }

  calibration_loaded_ = true;
  return true;
}

bool Driver::loadFireTime() {
  if (firetime_loaded_) {
    return true;
  }

  if (pcap_file_ == "") {
    char *buffer = nullptr;
    uint32_t len = 0;
    auto ret = TcpCommandGetLidarFiretime(tcp_command_client_, &buffer, &len);
    if (ret != 0 || !buffer) {
      return false;
    }
    std::string content = std::string(buffer);
    free(buffer);
    if (0 != laser_offset_.ParserFiretimeData(content)) {
      return false;
    }
  } else {
    std::ifstream fin(firetime_file_);
    if (!fin.is_open()) {
      return false;
    }
    int length = 0;
    std::string content;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    content.resize(length);
    fin.seekg(0, std::ios::beg);
    fin.read(content.data(), length);
    fin.close();
    if (0 != laser_offset_.ParserFiretimeData(content)) {
      return false;
    }
  }

  firetime_loaded_ = true;
  return true;
}

void Driver::loadConfigFile() {
  while (status_ == INIT && (!loadCalibration() || !loadFireTime())) {
    std::cout << "Fail to load config files, retrying..." << std::endl;
    sleep(1);
  }

  if (status_ == INIT) {
    std::cout << "Driver running" << std::endl;
    status_ = RUNNING;
  } else {
    std::cout << "Abort loading configs" << std::endl;
  }
}

int Driver::loadCalibrationFromString(const std::string &calibration_content) {
  std::istringstream ifs(calibration_content);

  std::string line;
  if (std::getline(ifs, line)) { // first line "Laser id,Elevation,Azimuth"
    std::cout << "Parse Lidar Correction..." << std::endl;
  }

  int lineCounter = 0;
  std::vector<std::string> firstLine;
  boost::split(firstLine, line, boost::is_any_of(","));
  if (firstLine[0] == "EEFF" || firstLine[0] == "eeff") {
    std::getline(ifs, line);
    for (int i = 0; i < PANDAR128_LASER_NUM; i++) {
      std::getline(ifs, line);
      if (line.length() < strlen("1,1,1")) {
        return -1;
      } else {
        lineCounter++;
      }

      float elev, azimuth;
      int lineId = 0;
      std::stringstream ss(line);
      std::string subline;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> lineId;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> elev;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> azimuth;

      if (lineId != lineCounter) {
        std::cerr << "laser id error %d %d" << lineId << ", " << lineCounter
                  << std::endl;
        return -1;
      }
      pitchAngles_[lineId - 1] = elev;
      yawAngles_[lineId - 1] = azimuth;
    }
  } else {
    while (std::getline(ifs, line)) {
      if (line.length() < strlen("1,1,1")) {
        return -1;
      } else {
        lineCounter++;
      }

      float elev, azimuth;
      int lineId = 0;

      std::stringstream ss(line);
      std::string subline;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> lineId;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> elev;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> azimuth;

      if (lineId != lineCounter) {
        std::cerr << "laser id error %d %d" << lineId << ", " << lineCounter
                  << std::endl;
        return -1;
      }

      pitchAngles_[lineId - 1] = elev;
      yawAngles_[lineId - 1] = azimuth;
    }
  }

  return 0;
}

} // namespace pandar128
