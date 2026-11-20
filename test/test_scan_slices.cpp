// Host test for scan slicing. No hardware, no PlatformIO.
// g++ -std=c++17 -Wall -Wextra -Iinclude -o /tmp/test_scan_slices test/test_scan_slices.cpp && /tmp/test_scan_slices

#include "scan_slices.h"

#include <math.h>
#include <stdio.h>

namespace {

int g_fails = 0;

void expect(bool cond, const char* msg) {
  if (!cond) {
    fprintf(stderr, "FAIL %s\n", msg);
    g_fails++;
  }
}

}  // namespace

int main() {
  using namespace scan_slices;
  const double pi = 3.14159265358979323846;
  const float inc = static_cast<float>(2.0 * pi / stl19p::kScanBins);
  const float amin = static_cast<float>(-pi);

  expect(kSlices == 4 && kBinsPerSlice == 125, "4 slices of 125 bins");
  // Full scan today: 60 + 8 * 500 = 4060 bytes, i.e. three reliable fragments.
  expect(laser_scan_cdr_bytes(10, 500, 500) == 4060, "full scan CDR size");
  expect(kSliceCdrBytes == 1060, "slice CDR size");
  expect(kSliceWireBytes < kMtu, "slice under MTU");
  printf("slice: %d bins, %u CDR bytes, %u on the wire (MTU %u)\n", kBinsPerSlice,
         static_cast<unsigned>(kSliceCdrBytes), static_cast<unsigned>(kSliceWireBytes),
         static_cast<unsigned>(kMtu));

  int covered = 0;
  for (int k = 0; k < kSlices; k++) {
    SliceGeom g = slice_geom(k, amin, inc);
    expect(g.first_bin == covered, "slices are contiguous");
    covered += g.bins;
    // Reassembler's index recovery.
    const double kk = (static_cast<double>(g.angle_min) - static_cast<double>(amin)) /
                      (static_cast<double>(g.bins) * static_cast<double>(inc));
    expect(fabs(kk - k) < 1e-3, "index recoverable from angle_min");
    expect(fabsf(g.angle_max - g.angle_min - (g.bins - 1) * inc) < 1e-5f, "angle_max span");
  }
  expect(covered == stl19p::kScanBins, "slices cover the scan");
  SliceGeom last = slice_geom(kSlices - 1, amin, inc);
  const float full_max = static_cast<float>(-pi + (stl19p::kScanBins - 1) * (2.0 * pi / stl19p::kScanBins));
  expect(fabsf(last.angle_max - full_max) < 1e-5f, "last slice ends at full angle_max");
  expect(fabsf(slice_geom(0, amin, inc).angle_min - amin) < 1e-7f, "slice 0 starts at full angle_min");

  if (g_fails) {
    fprintf(stderr, "%d failure(s)\n", g_fails);
    return 1;
  }
  printf("test_scan_slices OK\n");
  return 0;
}
