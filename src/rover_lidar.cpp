#include "rover_lidar.h"

#include <string.h>

#include <Arduino.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "rover_pins_pros3.h"

namespace {

constexpr uart_port_t kUart = UART_NUM_1;
constexpr int kBaud = 230400;

stl19p::Parser g_parser;
stl19p::ScanAssembler g_assembler;
stl19p::Scan g_slots[2];
stl19p::Scan g_ready;

SemaphoreHandle_t g_lock = nullptr;
volatile uint32_t g_seq = 0;
volatile int g_latest = 0;
volatile uint32_t g_packets = 0;
volatile uint32_t g_crc_errors = 0;
volatile uint32_t g_resyncs = 0;
volatile uint32_t g_scans = 0;
volatile uint32_t g_open_points = 0;
volatile uint32_t g_bytes_last_sec = 0;
volatile uint32_t g_frame_errors = 0;
volatile uint32_t g_overflows = 0;
volatile uint32_t g_last_packet_ms = 0;
volatile uint32_t g_prev_scan_ms = 0;
volatile float g_hz = 0.0f;
QueueHandle_t g_uart_q = nullptr;
uint32_t g_byte_window_ms = 0;
uint32_t g_byte_window_accum = 0;
bool g_started = false;

void note_bytes(int n) {
  const uint32_t now = millis();
  if (g_byte_window_ms == 0) {
    g_byte_window_ms = now;
  }
  if (n > 0) {
    g_byte_window_accum += static_cast<uint32_t>(n);
  }
  if (now - g_byte_window_ms >= 1000) {
    g_bytes_last_sec = g_byte_window_accum;
    g_byte_window_accum = 0;
    g_byte_window_ms = now;
  }
}

void drain_uart_events() {
  if (!g_uart_q) {
    return;
  }
  uart_event_t ev;
  while (xQueueReceive(g_uart_q, &ev, 0) == pdTRUE) {
    if (ev.type == UART_FRAME_ERR || ev.type == UART_PARITY_ERR) {
      g_frame_errors++;
    } else if (ev.type == UART_FIFO_OVF || ev.type == UART_BUFFER_FULL) {
      g_overflows++;
      uart_flush_input(kUart);
      xQueueReset(g_uart_q);
      break;
    }
  }
}

void note_scan(uint32_t now) {
  const uint32_t prev = g_prev_scan_ms;
  if (prev != 0 && now != prev) {
    g_hz = 1000.0f / static_cast<float>(now - prev);
  }
  g_prev_scan_ms = now;
  g_scans++;
}

void lidar_task(void*) {
  uint8_t buf[512];
  for (;;) {
    drain_uart_events();
    const int n = uart_read_bytes(kUart, buf, sizeof(buf), pdMS_TO_TICKS(20));
    note_bytes(n);
    if (n > 0) {
      for (int i = 0; i < n; i++) {
        stl19p::Packet pkt;
        if (!g_parser.feed_byte(buf[i], &pkt)) {
          continue;
        }
        g_last_packet_ms = millis();
        if (!g_assembler.add(pkt, &g_ready)) {
          continue;
        }
        if (g_lock && xSemaphoreTake(g_lock, pdMS_TO_TICKS(5)) == pdTRUE) {
          const int slot = g_latest ^ 1;
          memcpy(&g_slots[slot], &g_ready, sizeof(g_ready));
          g_latest = slot;
          g_seq++;
          xSemaphoreGive(g_lock);
          note_scan(millis());
        }
      }
      g_packets = g_parser.packets();
      g_crc_errors = g_parser.crc_errors();
      g_resyncs = g_parser.resyncs();
      g_open_points = g_assembler.open_points();
    }
  }
}

}  // namespace

bool rover_lidar_begin() {
  if (g_started) {
    return true;
  }
  uart_config_t cfg = {};
  cfg.baud_rate = kBaud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.rx_flow_ctrl_thresh = 0;
  cfg.source_clk = UART_SCLK_APB;
  if (uart_param_config(kUart, &cfg) != ESP_OK) {
    Serial.println("lidar UART config failed");
    return false;
  }
  // TX stays unrouted. Default UART1 TX is GPIO17 (LDO2) on this core; never
  // let begin() fall back to that. Lidar Tx (3.3 V) lands on LIDAR_RX_PIN.
  if (uart_set_pin(kUart, UART_PIN_NO_CHANGE, LIDAR_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) !=
      ESP_OK) {
    Serial.println("lidar UART pins failed");
    return false;
  }
  int rx_buf = 0;
  const int kTry[] = {16384, 4096, 1024};
  for (int sz : kTry) {
    g_uart_q = nullptr;
    // The driver creates the event queue. Depth 20 is drained by lidar_task;
    // without that drain, FRAME_ERR / FIFO_OVF would be dropped once it fills.
    if (uart_driver_install(kUart, sz, 0, 20, &g_uart_q, 0) == ESP_OK) {
      rx_buf = sz;
      break;
    }
    g_uart_q = nullptr;
  }
  if (rx_buf == 0) {
    Serial.println("lidar UART install failed");
    return false;
  }
  g_lock = xSemaphoreCreateMutex();
  if (!g_lock ||
      xTaskCreatePinnedToCore(lidar_task, "lidar", 8192, nullptr, 3, nullptr, 0) != pdPASS) {
    uart_driver_delete(kUart);
    Serial.println("lidar task failed");
    return false;
  }
  g_started = true;
  Serial.printf("STL-19P UART%d RX GPIO%d %d 8N1, rxbuf=%d (PWM tied to GND, TX unused)\n",
                static_cast<int>(kUart), LIDAR_RX_PIN, kBaud, rx_buf);
  return true;
}

bool rover_lidar_started() { return g_started; }

bool rover_lidar_copy_scan(stl19p::Scan* out, uint32_t* seq) {
  if (!g_started || !g_lock || !out || !seq) {
    return false;
  }
  if (xSemaphoreTake(g_lock, pdMS_TO_TICKS(2)) != pdTRUE) {
    return false;
  }
  const uint32_t cur = g_seq;
  if (cur == 0) {
    xSemaphoreGive(g_lock);
    return false;
  }
  memcpy(out, &g_slots[g_latest], sizeof(*out));
  xSemaphoreGive(g_lock);
  *seq = cur;
  return true;
}

uint32_t rover_lidar_packets() { return g_packets; }
uint32_t rover_lidar_crc_errors() { return g_crc_errors; }
uint32_t rover_lidar_resyncs() { return g_resyncs; }
uint32_t rover_lidar_scans() { return g_scans; }
uint32_t rover_lidar_open_points() { return g_open_points; }
uint32_t rover_lidar_bytes_last_sec() { return g_bytes_last_sec; }
uint32_t rover_lidar_frame_errors() { return g_frame_errors; }
uint32_t rover_lidar_overflows() { return g_overflows; }
float rover_lidar_hz() { return g_hz; }

bool rover_lidar_recent(uint32_t now_ms, uint32_t window_ms) {
  const uint32_t last = g_last_packet_ms;
  return last != 0 && (now_ms - last) <= window_ms;
}
