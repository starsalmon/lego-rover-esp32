// Host parser test. No hardware, no PlatformIO.
// g++ -std=c++17 -Wall -Wextra -Iinclude -o /tmp/test_stl19p test/test_stl19p.cpp src/stl19p.cpp && /tmp/test_stl19p

#include "stl19p.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846f;

int g_fails = 0;

void expect(bool cond, const char* msg) {
  if (!cond) {
    fprintf(stderr, "FAIL %s\n", msg);
    g_fails++;
  }
}

void expect_near(float got, float want, float tol, const char* msg) {
  if (!(fabsf(got - want) <= tol)) {
    fprintf(stderr, "FAIL %s (got %.6f want %.6f)\n", msg, got, want);
    g_fails++;
  }
}

// LD19 / STL-19P development manual section 3.3, hex dump (the prose table's
// last-point figures do not match this dump; the dump is what CRC-checks).
const uint8_t kManual[] = {
    0x54, 0x2C, 0x68, 0x08, 0xAB, 0x7E, 0xE0, 0x00, 0xE4, 0xDC, 0x00, 0xE2, 0xD9, 0x00, 0xE5, 0xD5,
    0x00, 0xE3, 0xD3, 0x00, 0xE4, 0xD0, 0x00, 0xE9, 0xCD, 0x00, 0xE4, 0xCA, 0x00, 0xE2, 0xC7, 0x00,
    0xE9, 0xC5, 0x00, 0xE5, 0xC2, 0x00, 0xE5, 0xC0, 0x00, 0xE5, 0xBE, 0x82, 0x3A, 0x1A, 0x50};

void put16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xff);
  p[1] = static_cast<uint8_t>(v >> 8);
}

void build_packet(uint8_t* out, uint16_t speed, uint16_t start_cdeg, uint16_t end_cdeg, uint16_t ts,
                  uint16_t mm, uint8_t intensity) {
  memset(out, 0, stl19p::kPacketSize);
  out[0] = stl19p::kHeader;
  out[1] = stl19p::kVerLen;
  put16(out + 2, speed);
  put16(out + 4, start_cdeg);
  for (int i = 0; i < stl19p::kPointsPerPacket; i++) {
    put16(out + 6 + 3 * i, mm);
    out[8 + 3 * i] = intensity;
  }
  put16(out + 42, end_cdeg);
  put16(out + 44, ts);
  out[46] = stl19p::crc8(out, 46);
}

int feed_all(stl19p::Parser* parser, const uint8_t* data, size_t len, stl19p::Packet* last) {
  int n = 0;
  for (size_t i = 0; i < len; i++) {
    stl19p::Packet pkt;
    if (parser->feed_byte(data[i], &pkt)) {
      if (last) {
        *last = pkt;
      }
      n++;
    }
  }
  return n;
}

void test_manual_packet() {
  expect(stl19p::crc8(kManual, 46) == 0x50, "manual crc");
  stl19p::Parser parser;
  stl19p::Packet pkt;
  const uint8_t junk[] = {0x00, 0x11, 0x54, 0x00};
  expect(feed_all(&parser, junk, sizeof(junk), &pkt) == 0, "junk is not a frame");
  expect(feed_all(&parser, kManual, sizeof(kManual), &pkt) == 1, "manual frame");
  expect(pkt.speed_dps == 2152, "speed 2152 dps");
  expect(pkt.start_cdeg == 32427, "start 324.27 deg");
  expect(pkt.end_cdeg == 33470, "end 334.70 deg");
  expect(pkt.timestamp_ms == 6714, "timestamp");
  expect(pkt.n == 12, "12 points");
  expect_near(pkt.points[0].angle_deg, 324.27f, 0.02f, "point0 angle");
  expect_near(pkt.points[11].angle_deg, 334.70f, 0.02f, "point11 angle");
  expect_near(pkt.points[0].range_m, 0.224f, 1e-5f, "point0 range");
  expect_near(pkt.points[0].intensity, 228.0f, 0.1f, "point0 intensity");
  expect_near(pkt.points[11].range_m, 0.192f, 1e-5f, "point11 range");
  expect_near(pkt.points[11].intensity, 229.0f, 0.1f, "point11 intensity");
  expect(pkt.points[11].angle_deg > pkt.points[0].angle_deg, "angles increase");
}

void test_crc_and_resync() {
  uint8_t bad[stl19p::kPacketSize];
  memset(bad, 0, sizeof(bad));
  bad[0] = stl19p::kHeader;
  bad[1] = stl19p::kVerLen;
  bad[10] = 0xff;
  bad[46] = 0xff;
  expect(stl19p::crc8(bad, 46) != bad[46], "crafted frame is actually bad");

  uint8_t good[stl19p::kPacketSize];
  build_packet(good, 3600, 0, 800, 10, 1000, 200);
  // A 0x54 inside a valid distance must not split the frame.
  put16(good + 6, 0x0054);
  good[8] = 42;
  good[46] = stl19p::crc8(good, 46);

  uint8_t stream[stl19p::kPacketSize * 3];
  memcpy(stream, bad, stl19p::kPacketSize);
  memcpy(stream + stl19p::kPacketSize, good, stl19p::kPacketSize);
  memcpy(stream + 2 * stl19p::kPacketSize, kManual, stl19p::kPacketSize);

  stl19p::Parser parser;
  stl19p::Packet pkt{};
  const int n = feed_all(&parser, stream, sizeof(stream), &pkt);
  expect(n == 2, "bad frame dropped, two good frames kept");
  expect(parser.crc_errors() >= 1, "crc error counted");
  expect(pkt.speed_dps == 2152, "last frame is the manual example");
}

void test_invalid_ranges() {
  uint8_t raw[stl19p::kPacketSize];
  stl19p::Parser parser;
  stl19p::Packet pkt;

  build_packet(raw, 3600, 0, 800, 1, 0, 10);
  expect(feed_all(&parser, raw, sizeof(raw), &pkt) == 1, "zero-distance frame");
  expect(std::isnan(pkt.points[0].range_m), "0 mm is NaN");
  expect(std::isnan(pkt.points[0].intensity), "invalid intensity is NaN");

  build_packet(raw, 3600, 0, 800, 2, 20, 10);
  expect(feed_all(&parser, raw, sizeof(raw), &pkt) == 1, "too-near frame");
  expect(std::isnan(pkt.points[3].range_m), "20 mm is NaN");

  build_packet(raw, 3600, 0, 800, 3, 13000, 10);
  expect(feed_all(&parser, raw, sizeof(raw), &pkt) == 1, "too-far frame");
  expect(std::isnan(pkt.points[3].range_m), "13000 mm is NaN");

  build_packet(raw, 3600, 0, 800, 4, 30, 7);
  expect(feed_all(&parser, raw, sizeof(raw), &pkt) == 1, "near edge");
  expect_near(pkt.points[0].range_m, 0.03f, 1e-6f, "30 mm kept");

  build_packet(raw, 3600, 0, 800, 5, 12000, 9);
  expect(feed_all(&parser, raw, sizeof(raw), &pkt) == 1, "far edge");
  expect_near(pkt.points[0].range_m, 12.0f, 1e-4f, "12000 mm kept");
}

void test_wrap_angles() {
  uint8_t raw[stl19p::kPacketSize];
  build_packet(raw, 3600, 35000, 1000, 20, 1500, 80);
  stl19p::Parser parser;
  stl19p::Packet pkt;
  expect(feed_all(&parser, raw, sizeof(raw), &pkt) == 1, "wrap frame");
  expect_near(pkt.points[0].angle_deg, 350.0f, 0.05f, "wrap start");
  expect(pkt.points[11].angle_deg < 15.0f, "wrap end is past 0");
  bool crossed = false;
  for (int i = 1; i < 12; i++) {
    if (pkt.points[i].angle_deg + 180.0f < pkt.points[i - 1].angle_deg) {
      crossed = true;
    }
  }
  expect(crossed, "interpolated angles cross 0");
}

void feed_packet(stl19p::Parser* parser, stl19p::ScanAssembler* scans, const uint8_t* raw,
                 int* got, stl19p::Scan* latest) {
  for (int i = 0; i < stl19p::kPacketSize; i++) {
    stl19p::Packet pkt;
    if (!parser->feed_byte(raw[i], &pkt)) {
      continue;
    }
    if (scans->add(pkt, latest)) {
      (*got)++;
    }
  }
}

void test_revolution() {
  stl19p::Parser parser;
  stl19p::ScanAssembler scans;
  stl19p::Scan scan{};
  int got = 0;
  uint8_t raw[stl19p::kPacketSize];

  auto one_rev = [&](int ts0) {
    for (int i = 0; i < 36; i++) {
      const uint16_t start = static_cast<uint16_t>(i * 1000);
      uint16_t mm = 2000;
      if (i == 0) mm = 1234;    // lidar 0° → forward bin
      if (i == 9) mm = 3456;    // lidar 90° clockwise → right
      if (i == 18) mm = 2222;   // lidar 180° → rear
      build_packet(raw, 3600, start, static_cast<uint16_t>(start + 800),
                   static_cast<uint16_t>(ts0 + i * 3), mm, 180);
      feed_packet(&parser, &scans, raw, &got, &scan);
    }
    build_packet(raw, 3600, 35800, 200, static_cast<uint16_t>(ts0 + 36 * 3), 2000, 180);
    feed_packet(&parser, &scans, raw, &got, &scan);
  };

  const int before = got;
  one_rev(0);
  expect(got == before + 1, "one scan per revolution");
  expect_near(scan.angle_min, -kPi, 1e-5f, "angle_min");
  expect(scan.angle_max > 3.0f && scan.angle_max < kPi, "angle_max");
  expect_near(scan.angle_increment, 2.0f * kPi / 500.0f, 1e-6f, "increment");
  expect_near(scan.angle_min + 499.0f * scan.angle_increment, scan.angle_max, 1e-4f, "span");
  expect_near(scan.range_min, 0.03f, 1e-6f, "range_min");
  expect_near(scan.range_max, 12.0f, 1e-5f, "range_max");
  expect(scan.scan_time > 0.05f && scan.scan_time < 0.2f, "scan_time");
  expect(scan.time_increment > 0.0f, "time_increment");
  expect(scan.points >= 200, "enough points");
  expect_near(scan.ranges[250], 1.234f, 1e-4f, "forward bin is lidar 0");
  expect_near(scan.ranges[125], 3.456f, 1e-4f, "right bin is lidar 90 clockwise");
  expect_near(scan.ranges[0], 2.222f, 1e-4f, "rear bin is lidar 180");
  expect_near(scan.intensities[250], 180.0f, 0.1f, "forward intensity");
  for (int i = 0; i < stl19p::kScanBins; i++) {
    if (std::isnan(scan.ranges[i])) {
      expect(std::isnan(scan.intensities[i]), "empty bin intensity is NaN");
      continue;
    }
    expect(scan.ranges[i] > 0.03f && scan.ranges[i] <= 12.0f, "finite range in window");
    expect(scan.ranges[i] != 0.0f, "range is not zero");
  }

  one_rev(5000);
  expect(got == before + 2, "second revolution");
}

}  // namespace

int main() {
  test_manual_packet();
  test_crc_and_resync();
  test_invalid_ranges();
  test_wrap_angles();
  test_revolution();
  if (g_fails) {
    fprintf(stderr, "%d failure(s)\n", g_fails);
    return 1;
  }
  printf("stl19p parser tests passed\n");
  return 0;
}
