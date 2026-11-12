#pragma once

// LDROBOT STL-19P / LD19 (Waveshare D500) UART frames.
// Development manual: 230400 8N1, one-way, header 0x54, VerLen fixed at 0x2C
// (protocol version 1, 12 points). Distances are millimetres, angles are 0.01°,
// speed is degrees/second, timestamp is milliseconds rolling over at 30000.
// CRC-8 is the table published in that manual, over every byte except itself.
//
// The sensor's angle increases clockwise from its front (left-handed).
// ScanAssembler mirrors that into a REP-103 LaserScan: 0 rad is the same
// forward, positive is counter-clockwise, angle_min is -pi.

#include <stddef.h>
#include <stdint.h>

namespace stl19p {

constexpr uint8_t kHeader = 0x54;
constexpr uint8_t kVerLen = 0x2C;
constexpr int kPointsPerPacket = 12;
constexpr int kPacketSize = 47;  // 10 + 3 * 12
constexpr int kScanBins = 500;   // 360° / 0.72°, the sensor's stated step
constexpr int kMinPointsPerScan = 200;
constexpr float kRangeMinM = 0.03f;
constexpr float kRangeMaxM = 12.0f;
constexpr uint16_t kTimestampModuloMs = 30000;

constexpr char kFrameId[] = "lidar_link";

struct Point {
  float angle_deg;  // clockwise from the sensor front, [0, 360)
  float range_m;    // NaN when the return is missing or outside range_min/max
  float intensity;  // device confidence 0–255, or NaN when range_m is NaN
};

struct Packet {
  uint16_t speed_dps;
  uint16_t start_cdeg;
  uint16_t end_cdeg;
  uint16_t timestamp_ms;
  uint8_t n;
  Point points[kPointsPerPacket];
};

struct Scan {
  float ranges[kScanBins];
  float intensities[kScanBins];
  float angle_min;
  float angle_max;
  float angle_increment;
  float time_increment;
  float scan_time;
  float range_min;
  float range_max;
  uint16_t points;
  uint16_t returns;
};

uint8_t crc8(const uint8_t* data, size_t len);

class Parser {
 public:
  Parser();

  // True when this byte completed a CRC-valid packet (written to *out).
  bool feed_byte(uint8_t byte, Packet* out);

  uint32_t packets() const { return packets_; }
  uint32_t crc_errors() const { return crc_errors_; }
  uint32_t resyncs() const { return resyncs_; }

 private:
  void replay_tail();

  uint8_t buf_[kPacketSize];
  uint8_t got_;
  uint8_t need_;
  bool replaying_;
  uint32_t packets_;
  uint32_t crc_errors_;
  uint32_t resyncs_;
};

class ScanAssembler {
 public:
  ScanAssembler();

  // Bins one packet. When a revolution closes, writes that scan to *out and
  // returns true. The point that crossed the seam starts the next revolution.
  bool add(const Packet& pkt, Scan* out);

  // Points buffered in the revolution that has not closed yet.
  uint32_t open_points() const { return points_; }

 private:
  void clear_bins();
  void publish(const Packet& pkt, Scan* out);

  float ranges_[kScanBins];
  float intensities_[kScanBins];
  float prev_deg_;
  uint16_t rev_start_ms_;
  uint32_t points_;
  bool have_prev_;
  bool have_start_;
};

}  // namespace stl19p
