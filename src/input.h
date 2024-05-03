/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  Pandar128 3D LIDAR data input classes
 *
 *    These classes provide raw Pandar128 LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     pandar::Input -- base class for accessing the data
 *                      independently of its source
 *
 *     pandar::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     pandar::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef __PANDAR_INPUT_H
#define __PANDAR_INPUT_H

#include <map>
#include <netinet/in.h>
#include <pcap.h>
#include <yaml-cpp/yaml.h>

#define UDP_VERSION_MAJOR_1 (1)
#define UDP_VERSION_MINOR_3 (3)
#define UDP_VERSION_MINOR_4 (4)
#define UDP_VERSION_MAJOR_3 (3)
#define UDP_VERSION_MINOR_2 (2)
#define UDP_VERSION_1_3 "1.3"
#define UDP_VERSION_1_4 "1.4"
#define UDP_VERSION_3_2 "3.2"
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
#define PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE (DISTANCE_SIZE + INTENSITY_SIZE)
#define PANDAR128_UNIT_WITH_CONFIDENCE_SIZE                                    \
  (DISTANCE_SIZE + INTENSITY_SIZE + CONFIDENCE_SIZE)
#define PANDAR128_TAIL_RESERVED1_SIZE (3)
#define PANDAR128_TAIL_RESERVED2_SIZE (3)
#define PANDAR128_SHUTDOWN_FLAG_SIZE (1)
#define PANDAR128_TAIL_RESERVED3_SIZE (3)
#define PANDAR128_MOTOR_SPEED_SIZE (2)
#define PANDAR128_AZIMUTH_FLAG_SIZE (2)
#define PANDAR128_TS_SIZE (4)
#define PANDAR128_RETURN_MODE_SIZE (1)
#define PANDAR128_FACTORY_INFO (1)
#define PANDAR128_UTC_SIZE (6)
#define PANDAR128_CRC_SIZE (4)
#define PANDAR128_FUNCTION_SAFETY_SIZE (17)
#define PANDAR128_IMU_SIZE (22)
#define PANDAR128_SIGNATURE_SIZE (32)
#define PANDAR128_SEQ_NUM_SIZE (4)

enum enumIndex {
  TIMESTAMP_INDEX,
  UTC_INDEX,
  SEQUENCE_NUMBER_INDEX,
  PACKET_SIZE,
};

static std::map<enumIndex, int> udpVersion13 = {
    {TIMESTAMP_INDEX, 796},
    {UTC_INDEX, 802},
    {SEQUENCE_NUMBER_INDEX, 817},
    {PACKET_SIZE, 812},
};

static std::map<enumIndex, int> udpVersion14 = {
    {TIMESTAMP_INDEX, 826},
    {UTC_INDEX, 820},
    {SEQUENCE_NUMBER_INDEX, 831},
    {PACKET_SIZE, 893},
};

static std::map<enumIndex, int> udpVersion32 = {
    {TIMESTAMP_INDEX, 826},
    {UTC_INDEX, 820},
    {SEQUENCE_NUMBER_INDEX, 831},
    {PACKET_SIZE, 893},
};

struct PandarPacket {
  uint8_t data[1500];
  uint32_t size;
};

namespace pandar128 {

static uint16_t DATA_PORT_NUMBER = 2368; // default data port
static uint16_t GPS_PORT_NUMBER = 8308;  // default position port

#define PANDAR128_SEQUENCE_NUMBER_OFFSET (831)
/** @brief pandar input base class */
class Input {
public:
  Input(const YAML::Node &cfg);
  virtual ~Input() {}

  /** @brief Read one pandar packet.
   *
   * @param pkt points to pandarPacket message
   *
   * @returns 0 if successful,
   *          -1 if end of file
   *          > 0 if incomplete packet (is this possible?)
   */
  virtual int getPacket(PandarPacket *pkt) = 0;
  bool checkPacket(PandarPacket *pkt);
  void setUdpVersion(uint8_t major, uint8_t minor);
  std::string getUdpVersion();

protected:
  uint16_t port_ = DATA_PORT_NUMBER;
  std::string devip_str_ = "";
  std::string m_sUdpVresion;
  bool m_bGetUdpVersion;
  int m_iTimestampIndex;
  int m_iUtcIindex;
  int m_iSequenceNumberIndex;
  int m_iPacketSize;
};

/** @brief Live pandar input from socket. */
class InputSocket : public Input {
public:
  InputSocket(const YAML::Node &cfg);
  virtual ~InputSocket();

  virtual int getPacket(PandarPacket *pkt);
  void setDeviceIP(const std::string &ip);

private:
  int sockfd_;
  in_addr devip_;
  uint32_t m_u32Sequencenum;
  uint8_t seqnub1;
  uint8_t seqnub2;
  uint8_t seqnub3;
  uint8_t seqnub4;
  std::string hostip_str_ = "";
  std::string multicastip_str_ = "";
  uint16_t gpsport_ = GPS_PORT_NUMBER;
};

/** @brief pandar input from PCAP dump file.
 *
 * Dump files can be grabbed by libpcap, pandar's DSR software,
 * ethereal, wireshark, tcpdump, or the \ref vdump_command.
 */
class InputPCAP : public Input {
public:
  InputPCAP(const YAML::Node &cfg);
  virtual ~InputPCAP();

  virtual int getPacket(PandarPacket *pkt);
  void setDeviceIP(const std::string &ip);

private:
  std::string filename_;
  pcap_t *pcap_ = nullptr;
  bpf_program pcap_packet_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_ = true;
  bool read_once_ = false;
  bool read_fast_ = false;
  double repeat_delay_ = 0.0;
  int gap;
  int64_t last_pkt_ts;
  int count;
  int64_t last_time;
  int64_t current_time;
  int64_t pkt_ts;
};

} // namespace pandar128

#endif // __PANDAR_INPUT_H
