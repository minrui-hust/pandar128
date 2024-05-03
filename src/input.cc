/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *  Copyright (c) 2020 Hesai Photonics Technology, Lingwen Fang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the Pandar128 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

#include "input.h"
#include "platUtil.h"
#include "yaml_extension.h"

namespace pandar128 {

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
Input::Input(const YAML::Node &cfg) {
  TryGetParameter(cfg, "device_ip", devip_str_);
  TryGetParameter(cfg, "port", port_);

  if (!devip_str_.empty()) {
    std::cout << "Only accepting packets from IP address: " << devip_str_
              << std::endl;
  }

  m_sUdpVresion = "";
  m_bGetUdpVersion = false;
  m_iTimestampIndex = 0;
  m_iUtcIindex = 0;
  m_iSequenceNumberIndex = 0;
  m_iPacketSize = 0;
}

bool Input::checkPacket(PandarPacket *pkt) {
  if (pkt->size < 100)
    return false;
  if (pkt->data[0] != 0xEE && pkt->data[1] != 0xFF) {
    std::cerr << "Packet with invaild delimiter" << std::endl;
    return false;
  }
  if (m_sUdpVresion == UDP_VERSION_1_3) {
    if (pkt->size == 812) {
      return true;
    } else {
      printf("Packet size mismatch.caculated size:812, packet size:%d",
             pkt->size);
      return false;
    }
  }
  uint8_t laserNum = pkt->data[6];
  uint8_t blockNum = pkt->data[7];
  uint8_t flags = pkt->data[11];

  bool hasSeqNum = (flags & 1);
  bool hasImu = (flags & 2);
  bool hasFunctionSafety = (flags & 4);
  bool hasSignature = (flags & 8);
  bool hasConfidence = (flags & 0x10);

  m_iUtcIindex =
      PANDAR128_HEAD_SIZE +
      (hasConfidence
           ? PANDAR128_UNIT_WITH_CONFIDENCE_SIZE * laserNum * blockNum
           : PANDAR128_UNIT_WITHOUT_CONFIDENCE_SIZE * laserNum * blockNum) +
      PANDAR128_AZIMUTH_SIZE * blockNum + PANDAR128_CRC_SIZE +
      (hasFunctionSafety ? PANDAR128_FUNCTION_SAFETY_SIZE : 0) +
      PANDAR128_TAIL_RESERVED1_SIZE + PANDAR128_TAIL_RESERVED2_SIZE +
      PANDAR128_TAIL_RESERVED3_SIZE + PANDAR128_AZIMUTH_FLAG_SIZE +
      PANDAR128_SHUTDOWN_FLAG_SIZE + PANDAR128_RETURN_MODE_SIZE +
      PANDAR128_MOTOR_SPEED_SIZE;
  m_iTimestampIndex = m_iUtcIindex + PANDAR128_UTC_SIZE;
  m_iSequenceNumberIndex =
      m_iTimestampIndex + PANDAR128_TS_SIZE + PANDAR128_FACTORY_INFO;

  uint32_t size = m_iSequenceNumberIndex + (hasImu ? PANDAR128_IMU_SIZE : 0) +
                  (hasSeqNum ? PANDAR128_SEQ_NUM_SIZE : 0) +
                  PANDAR128_CRC_SIZE +
                  (hasSignature ? PANDAR128_SIGNATURE_SIZE : 0);
  if (pkt->size == size) {
    return true;
  } else {
    std::cerr << "Packet size mismatch.caculated size:" << size
              << ", packet size:" << pkt->size << std::endl;
    return false;
  }
}

void Input::setUdpVersion(uint8_t major, uint8_t minor) {
  switch (major) {
  case UDP_VERSION_MAJOR_1: {
    switch (minor) {
    case UDP_VERSION_MINOR_3:
      m_sUdpVresion = UDP_VERSION_1_3;
      m_iTimestampIndex = udpVersion13[TIMESTAMP_INDEX];
      m_iUtcIindex = udpVersion13[UTC_INDEX];
      m_iSequenceNumberIndex = udpVersion13[SEQUENCE_NUMBER_INDEX];
      m_iPacketSize = udpVersion13[PACKET_SIZE];
      m_bGetUdpVersion = true;
      break;

    case UDP_VERSION_MINOR_4:
      m_sUdpVresion = UDP_VERSION_1_4;
      m_iTimestampIndex = udpVersion14[TIMESTAMP_INDEX];
      m_iUtcIindex = udpVersion14[UTC_INDEX];
      m_iSequenceNumberIndex = udpVersion14[SEQUENCE_NUMBER_INDEX];
      m_iPacketSize = udpVersion14[PACKET_SIZE];
      m_bGetUdpVersion = true;
      break;

    default:
      printf("error udp version minor: %d\n", minor);
      break;
    }
  } break;
  case UDP_VERSION_MAJOR_3: {
    switch (minor) {
    case UDP_VERSION_MINOR_2:
      m_sUdpVresion = UDP_VERSION_3_2;
      m_iTimestampIndex = udpVersion32[TIMESTAMP_INDEX];
      m_iUtcIindex = udpVersion32[UTC_INDEX];
      m_iSequenceNumberIndex = udpVersion32[SEQUENCE_NUMBER_INDEX];
      m_iPacketSize = udpVersion32[PACKET_SIZE];
      m_bGetUdpVersion = true;
      break;

    default:
      printf("error udp version minor: %d\n", minor);
      break;
    }
  } break;
  default:
    printf("error udp version minor: %d\n", minor);
    break;
  }
}

std::string Input::getUdpVersion() { return m_sUdpVresion; }

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number
 */
InputSocket::InputSocket(const YAML::Node &cfg) : Input(cfg) {
  TryGetParameter(cfg, "host_ip", hostip_str_);
  TryGetParameter(cfg, "multicast_ip", multicastip_str_);
  TryGetParameter(cfg, "gps_port", gpsport_);

  m_u32Sequencenum = 0;
  sockfd_ = -1;
  seqnub1 = 0;
  seqnub2 = 0;
  seqnub3 = 0;
  seqnub4 = 0;

  if (!devip_str_.empty()) {
    inet_aton(devip_str_.c_str(), &devip_);
  }

  // connect to Pandar UDP port
  std::cout << "Opening UDP socket: port " << port_;
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1) {
    perror("socket"); // TODO: ROS_ERROR errno
    return;
  }

  sockaddr_in my_addr;                  // my address information
  memset(&my_addr, 0, sizeof(my_addr)); // initialize to zeros
  my_addr.sin_family = AF_INET;         // host byte order
  my_addr.sin_port = htons(port_);      // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill in my IP

  if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
    perror("bind"); // TODO: ROS_ERROR errno
    return;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block");
    return;
  }

  int getchecksum = 0;
  socklen_t option_int = sizeof(int);
  int get_error =
      getsockopt(sockfd_, SOL_SOCKET, SO_NO_CHECK, &getchecksum, &option_int);
  int nochecksum = 1;
  int set_error = setsockopt(sockfd_, SOL_SOCKET, SO_NO_CHECK, &nochecksum,
                             sizeof(nochecksum));
  std::cout << "Pandar socket fd is " << sockfd_ << std::endl;
  int nRecvBuf = 26214400;
  setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, (const char *)&nRecvBuf,
             sizeof(int));
  if (multicastip_str_ != "") {
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(multicastip_str_.c_str());
    mreq.imr_interface.s_addr =
        hostip_str_ == "" ? htons(INADDR_ANY) : inet_addr(hostip_str_.c_str());
    int ret = setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                         (const char *)&mreq, sizeof(mreq));
    if (ret < 0) {
      printf("Multicast IP error,set correct multicast ip address or keep it "
             "empty %s %s\n",
             multicastip_str_.c_str(), hostip_str_.c_str());
    } else {
      printf("Recive data from multicast ip address %s %s\n",
             multicastip_str_.c_str(), hostip_str_.c_str());
    }
  }
}

/** @brief destructor */
InputSocket::~InputSocket(void) { (void)close(sockfd_); }

// return : 0 - lidar
//          2 - gps
//          1 - error
/** @brief Get one pandar packet. */
int InputSocket::getPacket(PandarPacket *pkt) {
  uint32_t seqnub;

  int isgps = 0;
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000; // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  int retval = poll(fds, 1, POLL_TIMEOUT);

  if (retval < 0) // poll() error?
  {
    if (errno != EINTR)
      std::cerr << "poll() error: " << strerror(errno) << std::endl;
    return 1;
  }
  if (retval == 0) // poll() timeout?
  {
    std::cerr << "Pandar poll() timeout" << std::endl;
    return 1;
  }
  if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
      (fds[0].revents & POLLNVAL)) // device error?
  {
    std::cerr << "poll() reports Pandar error" << std::endl;
    return 1;
  }

  ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], 10000, 0,
                            (sockaddr *)&sender_address, &sender_address_len);
  pkt->size = nbytes;
  if (!m_bGetUdpVersion) {
    return 0;
  }
  if (pkt->size == 512) {
    return 2;
  } else if (!checkPacket(pkt)) {
    return 1; // Packet size not match
  }

  static uint32_t dropped = 0, u32StartSeq = 0;
  static uint32_t startTick = GetTickCount();
  if (m_bGetUdpVersion) {
    if (!(m_sUdpVresion == UDP_VERSION_1_4 &&
          !(pkt->data[11] & 1))) { // Packet has UDP sequence number
      uint32_t *pSeq = (uint32_t *)&pkt->data[m_iSequenceNumberIndex];
      seqnub = *pSeq;

      if (m_u32Sequencenum == 0) {
        m_u32Sequencenum = seqnub;
        u32StartSeq = m_u32Sequencenum;
      } else {
        uint32_t diff = seqnub - m_u32Sequencenum;
        if (diff > 1) {
          std::cout << "seq diff: " << diff << std::endl;
          dropped += diff - 1;
        }
      }
      m_u32Sequencenum = seqnub;

      uint32_t endTick = GetTickCount();

      if (endTick - startTick >= 1000 && dropped > 0) {
        printf("!!!!!!!!!! dropped: %d, %d, percent, %f", dropped,
               m_u32Sequencenum - u32StartSeq,
               float(dropped) / float(m_u32Sequencenum - u32StartSeq) * 100.0);
        dropped = 0;
        u32StartSeq = m_u32Sequencenum;
        startTick = endTick;
      }
    }
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number
 *  @param packet_rate expected device packet frequency (Hz)
 *  @param filename PCAP dump file name
 */
InputPCAP::InputPCAP(const YAML::Node &cfg) : Input(cfg) {
  TryGetParameter(cfg, "filename", filename_);
  TryGetParameter(cfg, "read_once", read_once_);
  TryGetParameter(cfg, "read_fast", read_fast_);
  TryGetParameter(cfg, "repeat_delay", repeat_delay_);

  if (read_once_)
    std::cout << "Read input file only once." << std::endl;
  if (read_fast_)
    std::cout << "Read input file as quickly as possible." << std::endl;
  if (repeat_delay_ > 0.0)
    std::cout << "Delay" << repeat_delay_
              << " seconds before repeating input file." << std::endl;

  // Open the PCAP dump file
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL) {
    std::cerr << "Error opening Pandar socket dump file." << std::endl;
    return;
  }

  std::stringstream filter;
  if (devip_str_ != "") // using specific IP?
  {
    filter << "src host " << devip_str_ << " && ";
  }
  filter << "udp dst port " << port_;
  pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1,
               PCAP_NETMASK_UNKNOWN);
  gap = 100;
  last_pkt_ts = 0;
  last_time = 0;
  current_time = 0;
  pkt_ts = 0;
}

/** destructor */
InputPCAP::~InputPCAP(void) { pcap_close(pcap_); }

// return : 0 - lidar
//          2 - gps
//          1 - error
/** @brief Get one pandar packet. */
int InputPCAP::getPacket(PandarPacket *pkt) {
  pcap_pkthdr *pktHeader;
  const unsigned char *packetBuf;
  struct tm t;

  if (pcap_next_ex(pcap_, &pktHeader, &packetBuf) >= 0) {
    const uint8_t *packet = packetBuf + 42;
    memcpy(&pkt->data[0], packetBuf + 42, pktHeader->caplen - 42);
    pkt->size = pktHeader->caplen - 42;
    count++;
    if (!m_bGetUdpVersion)
      return 0;
    if (pktHeader->caplen == (512 + 42)) {
      // ROS_ERROR("GPS");
      return 2;
    } else if (!checkPacket(pkt)) {
      return 1; // Packet size not match
    }
    if (m_bGetUdpVersion && count >= gap) {
      count = 0;

      t.tm_year = packet[m_iUtcIindex];
      t.tm_mon = packet[m_iUtcIindex + 1] - 1;
      t.tm_mday = packet[m_iUtcIindex + 2];
      t.tm_hour = packet[m_iUtcIindex + 3];
      t.tm_min = packet[m_iUtcIindex + 4];
      t.tm_sec = packet[m_iUtcIindex + 5];
      t.tm_isdst = 0;

      pkt_ts = mktime(&t) * 1000000 +
               ((packet[m_iTimestampIndex] & 0xff) |
                (packet[m_iTimestampIndex + 1] & 0xff) << 8 |
                ((packet[m_iTimestampIndex + 2] & 0xff) << 16) |
                ((packet[m_iTimestampIndex + 3] & 0xff) << 24));
      struct timeval sys_time;
      gettimeofday(&sys_time, NULL);
      current_time = sys_time.tv_sec * 1000000 + sys_time.tv_usec;

      if (0 == last_pkt_ts) {
        last_pkt_ts = pkt_ts;
        last_time = current_time;
      } else {
        int64_t sleep_time =
            (pkt_ts - last_pkt_ts) - (current_time - last_time);
        // ROS_WARN("pkt time: %u,use time: %u,sleep time: %u",pkt_ts -
        // last_pkt_ts,current_time - last_time, sleep_time);

        if (sleep_time > 0) {
          struct timeval waitTime;
          waitTime.tv_sec = sleep_time / 1000000;
          waitTime.tv_usec = sleep_time % 1000000;

          int err;

          do {
            err = select(0, NULL, NULL, NULL, &waitTime);
          } while (err < 0 && errno != EINTR);
        }

        last_pkt_ts = pkt_ts;
        last_time = current_time;
        last_time += sleep_time;
      }
    }

    // if (pcapFile != NULL) {
    //   pcap_close(pcapFile);
    //   pcapFile = NULL;
    // }
    return 0;
  }
  return 1;
}

} // namespace pandar128
