#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#include <uxr/client/util/time.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

// Shared with DIAG WiFiUDP (rover_diag) — must not overlap parsePacket/beginPacket.
extern "C" bool rover_net_lock(uint32_t timeout_ms);
extern "C" void rover_net_unlock(void);

extern "C" {

static WiFiUDP udp_client;

bool platformio_transport_open(struct uxrCustomTransport * transport)
{
  struct micro_ros_agent_locator * locator = (struct micro_ros_agent_locator *) transport->args;
  if (!rover_net_lock(500)) {
    return false;
  }
  const bool ok = (true == udp_client.begin(locator->port));
  rover_net_unlock();
  return ok;
}

bool platformio_transport_close(struct uxrCustomTransport * transport)
{
  (void)transport;
  if (!rover_net_lock(500)) {
    return false;
  }
  udp_client.stop();
  rover_net_unlock();
  return true;
}

size_t platformio_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
  (void)errcode;
  struct micro_ros_agent_locator *locator = (struct micro_ros_agent_locator *)transport->args;

  if (!rover_net_lock(200)) {
    return 0;
  }
  size_t sent = 0;
  if (true == udp_client.beginPacket(locator->address, locator->port)) {
    sent = udp_client.write(buf, len);
    sent = true == udp_client.endPacket() ? sent : 0;
  }
  udp_client.flush();
  rover_net_unlock();
  return sent;
}

size_t platformio_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
  (void)errcode;
  (void)transport;

  int64_t start_time = uxr_millis();

  // Hold net lock only around WiFiUDP ops — not during delay — so DIAG can proceed
  // between polls; still no overlapping parsePacket/beginPacket.
  while ((uxr_millis() - start_time) < ((int64_t)timeout)) {
    if (!rover_net_lock(50)) {
      delay(1);
      continue;
    }
    const int pkt = udp_client.parsePacket();
    if (pkt != 0) {
      size_t available = 0;
      if (udp_client.available()) {
        available = udp_client.read(buf, len);
      }
      rover_net_unlock();
      return (available > 0) ? available : 0;
    }
    rover_net_unlock();
    delay(1);
  }

  return 0;
}

}
