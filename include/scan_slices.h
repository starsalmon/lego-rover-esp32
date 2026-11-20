#pragma once

// Splits one STL-19P revolution into best-effort LaserScan slices.
//
// micro-ROS best-effort output is a single MTU buffer with no XRCE
// fragmentation, so a 4 kB LaserScan cannot go best effort. Each slice here is
// a LaserScan of kBinsPerSlice consecutive bins (ranges + intensities) that
// fits in one 1400-byte datagram with room to spare.
//
// Wire contract with the NUC reassembler (robot-swarm-brain
// maps/lidar/scan_assemble.py), topic /rover/scan_slice:
//   * every slice of one revolution carries the SAME header.stamp (the
//     revolution stamp, exactly what the full /rover/scan carried). That stamp
//     is the grouping key; consecutive revolutions are ~100 ms apart.
//   * slice k covers bins [k * kBinsPerSlice, (k + 1) * kBinsPerSlice):
//     angle_min = full_min + k * kBinsPerSlice * inc, angle_max = angle_min +
//     (kBinsPerSlice - 1) * inc. The reassembler recovers k from angle_min.
//   * angle_increment, time_increment, scan_time, range_min/max are the full
//     revolution's values. Ray j of slice k was measured at
//     stamp + (k * kBinsPerSlice + j) * time_increment.
//   * frame_id is lidar_link, as on the full scan.

#include <stdint.h>

#include "stl19p.h"

namespace scan_slices {

constexpr int kSlices = 4;
constexpr int kBinsPerSlice = stl19p::kScanBins / kSlices;
static_assert(kBinsPerSlice * kSlices == stl19p::kScanBins, "slices must tile the scan exactly");

// Custom-transport MTU from metas/colcon_pros3_tof.meta.
constexpr uint32_t kMtu = 1400;
// XRCE session header (4) + submessage header (4) + WRITE_DATA base (4),
// plus 4 for a CDR encapsulation header in case the typesupport adds one.
constexpr uint32_t kXrceOverhead = 16;

constexpr uint32_t align4(uint32_t n) { return (n + 3u) & ~3u; }

// XCDR1 size of sensor_msgs/LaserScan with frame_len chars of frame_id and
// n ranges + m intensities. stamp (8), frame_id (4 + len + NUL), pad to 4,
// 7 floats (28), two float sequences (4 + 4n each).
constexpr uint32_t laser_scan_cdr_bytes(uint32_t frame_len, uint32_t n, uint32_t m) {
  return align4(8u + 4u + frame_len + 1u) + 28u + 4u + 4u * n + 4u + 4u * m;
}

constexpr uint32_t kSliceCdrBytes =
    laser_scan_cdr_bytes(sizeof(stl19p::kFrameId) - 1, kBinsPerSlice, kBinsPerSlice);
constexpr uint32_t kSliceWireBytes = kSliceCdrBytes + kXrceOverhead;
// Keep >= 200 bytes headroom under the MTU.
static_assert(kSliceWireBytes + 200u <= kMtu, "scan slice does not fit one best-effort datagram");

struct SliceGeom {
  int first_bin;
  int bins;
  float angle_min;
  float angle_max;
};

inline SliceGeom slice_geom(int k, float full_angle_min, float angle_increment) {
  SliceGeom g;
  g.first_bin = k * kBinsPerSlice;
  g.bins = kBinsPerSlice;
  // Same double math as ScanAssembler so slice 0 matches the full angle_min.
  const double a0 = static_cast<double>(full_angle_min) +
                    static_cast<double>(g.first_bin) * static_cast<double>(angle_increment);
  g.angle_min = static_cast<float>(a0);
  g.angle_max = static_cast<float>(a0 + static_cast<double>(kBinsPerSlice - 1) *
                                            static_cast<double>(angle_increment));
  return g;
}

}  // namespace scan_slices
