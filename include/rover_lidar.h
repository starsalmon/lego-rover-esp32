#pragma once

#include <stdint.h>

#include "stl19p.h"

// STL-19P on UART1 RX (LIDAR_RX_PIN). PWM is not driven: the wire is at GND
// so the lidar's internal controller holds ~10 Hz. Parsing runs on its own
// task so a micro-ROS ping or an I2C transaction cannot drop UART bytes.
// LaserScan is published from the main loop; micro-ROS is not called here.

bool rover_lidar_begin();
bool rover_lidar_started();

// Copies the newest full revolution. *seq increases once per published scan.
// False when no revolution has closed yet or the snapshot lock was busy.
bool rover_lidar_copy_scan(stl19p::Scan* out, uint32_t* seq);

uint32_t rover_lidar_packets();
uint32_t rover_lidar_crc_errors();
uint32_t rover_lidar_resyncs();
uint32_t rover_lidar_scans();
uint32_t rover_lidar_open_points();
// uart_read_bytes() total over the most recently closed 1 s window.
uint32_t rover_lidar_bytes_last_sec();
// IDF UART_FRAME_ERR + UART_PARITY_ERR, cumulative.
uint32_t rover_lidar_frame_errors();
// IDF UART_FIFO_OVF + UART_BUFFER_FULL, cumulative.
uint32_t rover_lidar_overflows();
float rover_lidar_hz();
// True when a CRC-valid frame arrived within window_ms of now_ms.
bool rover_lidar_recent(uint32_t now_ms, uint32_t window_ms);
