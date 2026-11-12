#include "stl19p.h"

#include <math.h>
#include <string.h>

#include <cmath>

namespace stl19p {

namespace {

constexpr double kPi = 3.14159265358979323846;

// LDROBOT development-manual CRC-8 (poly 0x4D, init 0). The manual's worked
// example ends in 0x50 and this table reproduces that byte.
constexpr uint8_t kCrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};

uint16_t rd16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

float nan_f() { return nanf(""); }

// Clockwise lidar degrees → bin in a CCW scan. Index 0 is the rear (−π),
// index kScanBins/2 is the sensor front. Done in centidegrees: a float
// divide of an exact cardinal angle lands just under the bin index and
// truncates into the neighbour.
int bin_for(float lidar_deg) {
  double deg = std::fmod(static_cast<double>(lidar_deg), 360.0);
  if (deg < 0.0) {
    deg += 360.0;
  }
  int cdeg = static_cast<int>(std::lround(deg * 100.0)) % 36000;
  if (cdeg < 0) {
    cdeg += 36000;
  }
  const int from_rear = (18000 - cdeg + 36000) % 36000;
  int idx = (from_rear * kScanBins) / 36000;
  if (idx < 0) {
    idx = 0;
  }
  if (idx >= kScanBins) {
    idx = kScanBins - 1;
  }
  return idx;
}

void decode(const uint8_t* raw, Packet* out) {
  out->speed_dps = rd16(raw + 2);
  out->start_cdeg = rd16(raw + 4);
  out->end_cdeg = rd16(raw + 42);
  out->timestamp_ms = rd16(raw + 44);
  out->n = kPointsPerPacket;

  const uint16_t start = out->start_cdeg;
  const uint16_t end = out->end_cdeg;
  const uint32_t span = (end >= start) ? static_cast<uint32_t>(end - start)
                                       : static_cast<uint32_t>(end) + 36000u - start;
  for (int i = 0; i < kPointsPerPacket; i++) {
    const uint8_t* pt = raw + 6 + 3 * i;
    const uint16_t mm = rd16(pt);
    const uint8_t intensity = pt[2];
    float cdeg = static_cast<float>(start) +
                 (static_cast<float>(span) * static_cast<float>(i)) /
                     static_cast<float>(kPointsPerPacket - 1);
    if (cdeg >= 36000.0f) {
      cdeg -= 36000.0f;
    }
    Point& dst = out->points[i];
    dst.angle_deg = cdeg * 0.01f;
    // 0 mm is "no return". Outside the datasheet window is not a hit either.
    if (mm < 30 || mm > 12000) {
      dst.range_m = nan_f();
      dst.intensity = nan_f();
    } else {
      dst.range_m = static_cast<float>(mm) * 0.001f;
      dst.intensity = static_cast<float>(intensity);
    }
  }
}

}  // namespace

uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc = kCrcTable[(crc ^ data[i]) & 0xff];
  }
  return crc;
}

Parser::Parser()
    : got_(0), need_(0), replaying_(false), packets_(0), crc_errors_(0), resyncs_(0) {}

void Parser::replay_tail() {
  // A shifted copy of the same rejected frame must not recurse. The tail is
  // shorter than a frame, so one pass only re-arms on a header byte inside it.
  if (replaying_) {
    got_ = 0;
    need_ = 0;
    return;
  }
  uint8_t saved[kPacketSize];
  memcpy(saved, buf_, kPacketSize);
  got_ = 0;
  need_ = 0;
  resyncs_++;
  replaying_ = true;
  Packet discard;
  for (int i = 1; i < kPacketSize; i++) {
    (void)feed_byte(saved[i], &discard);
  }
  replaying_ = false;
}

bool Parser::feed_byte(uint8_t byte, Packet* out) {
  if (got_ == 0) {
    if (byte == kHeader) {
      buf_[got_++] = byte;
    }
    return false;
  }

  buf_[got_++] = byte;
  if (got_ == 2 && buf_[1] != kVerLen) {
    const uint8_t second = buf_[1];
    got_ = 0;
    need_ = 0;
    resyncs_++;
    if (second == kHeader) {
      buf_[got_++] = kHeader;
    }
    return false;
  }
  if (got_ == 2) {
    need_ = kPacketSize;
  }
  if (need_ == 0 || got_ < need_) {
    return false;
  }

  got_ = 0;
  need_ = 0;
  if (buf_[kPacketSize - 1] != crc8(buf_, kPacketSize - 1)) {
    crc_errors_++;
    replay_tail();
    return false;
  }
  if (out) {
    decode(buf_, out);
  }
  packets_++;
  return out != nullptr;
}

ScanAssembler::ScanAssembler() : prev_deg_(0.0f), rev_start_ms_(0), points_(0), have_prev_(false), have_start_(false) {
  clear_bins();
}

void ScanAssembler::clear_bins() {
  const float n = nan_f();
  for (int i = 0; i < kScanBins; i++) {
    ranges_[i] = n;
    intensities_[i] = n;
  }
  points_ = 0;
}

void ScanAssembler::publish(const Packet& pkt, Scan* out) {
  memcpy(out->ranges, ranges_, sizeof(out->ranges));
  memcpy(out->intensities, intensities_, sizeof(out->intensities));
  const double inc = (2.0 * kPi) / static_cast<double>(kScanBins);
  out->angle_min = static_cast<float>(-kPi);
  out->angle_increment = static_cast<float>(inc);
  out->angle_max = static_cast<float>(-kPi + (kScanBins - 1) * inc);
  out->range_min = kRangeMinM;
  out->range_max = kRangeMaxM;
  out->points = points_ > 0xffffu ? 0xffffu : static_cast<uint16_t>(points_);

  uint16_t returns = 0;
  for (int i = 0; i < kScanBins; i++) {
    if (!std::isnan(out->ranges[i])) {
      returns++;
    }
  }
  out->returns = returns;

  const uint16_t dt =
      static_cast<uint16_t>((static_cast<uint32_t>(pkt.timestamp_ms) + kTimestampModuloMs - rev_start_ms_) %
                            kTimestampModuloMs);
  float scan_time = static_cast<float>(dt) / 1000.0f;
  if (scan_time < 0.05f || scan_time > 0.5f) {
    if (pkt.speed_dps >= 1500 && pkt.speed_dps <= 7000) {
      scan_time = 360.0f / static_cast<float>(pkt.speed_dps);
    } else {
      scan_time = 0.1f;
    }
  }
  out->scan_time = scan_time;
  out->time_increment = scan_time / static_cast<float>(kScanBins);
}

bool ScanAssembler::add(const Packet& pkt, Scan* out) {
  bool emitted = false;
  for (uint8_t i = 0; i < pkt.n && i < kPointsPerPacket; i++) {
    const Point& pt = pkt.points[i];
    if (have_prev_ && (prev_deg_ - pt.angle_deg) > 180.0f) {
      if (!emitted && points_ >= kMinPointsPerScan && out) {
        publish(pkt, out);
        emitted = true;
      }
      clear_bins();
      rev_start_ms_ = pkt.timestamp_ms;
      have_start_ = true;
    }
    if (!have_start_) {
      rev_start_ms_ = pkt.timestamp_ms;
      have_start_ = true;
    }
    if (!std::isnan(pt.range_m)) {
      const int bin = bin_for(pt.angle_deg);
      if (std::isnan(ranges_[bin]) || pt.range_m < ranges_[bin]) {
        ranges_[bin] = pt.range_m;
        intensities_[bin] = pt.intensity;
      }
    }
    points_++;
    prev_deg_ = pt.angle_deg;
    have_prev_ = true;
  }
  return emitted;
}

}  // namespace stl19p
