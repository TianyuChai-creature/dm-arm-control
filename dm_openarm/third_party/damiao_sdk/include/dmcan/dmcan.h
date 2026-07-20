#pragma once

/**
 * C API for Damiao libdm_device (dmcan), aligned with dmcan-sdk / resources/u2canfd.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum dmcan_device_type {
  DMCAN_USB2CANFD = 0,
  DMCAN_USB2CANFD_DUAL = 1,
  DMCAN_LINKX4C = 2,
} dmcan_device_type;

#pragma pack(push, 1)

typedef struct dmcan_channel_can_info {
  uint8_t channel;
  bool canfd;
  uint32_t can_baudrate;
  uint32_t canfd_baudrate;
  float can_sp;
  float canfd_sp;
} dmcan_channel_can_info;

typedef struct usb_rx_frame_head {
  uint32_t can_id : 29;
  uint32_t esi : 1;
  uint32_t ext : 1;
  uint32_t rtr : 1;
  uint64_t timestamp;
  uint8_t channel;
  uint8_t canfd : 1;
  uint8_t dir : 1;
  uint8_t brs : 1;
  uint8_t ack : 1;
  uint8_t dlc : 4;
  uint16_t reserved;
} usb_rx_frame_head;

typedef struct usb_rx_frame {
  usb_rx_frame_head head;
  uint8_t payload[64];
} usb_rx_frame;

#pragma pack(pop)

typedef struct dmcan_context dmcan_context;
typedef struct dmcan_device_handle dmcan_device_handle;

typedef void (*dmcan_frame_callback)(dmcan_device_handle* handle, usb_rx_frame* frame);

void dmcan_context_create(dmcan_context** ctx);
void dmcan_context_destroy(dmcan_context* ctx);
void dmcan_print_version(dmcan_context* ctx);
int dmcan_find_devices(dmcan_context* ctx);
int dmcan_find_devices_with_type(dmcan_context* ctx, int device_type);
void dmcan_show_all_devices(dmcan_context* ctx);
bool dmcan_device_get(dmcan_context* ctx, dmcan_device_handle** out, int index);

bool dmcan_device_open(dmcan_device_handle* handle);
void dmcan_device_close(dmcan_device_handle* handle);
void dmcan_device_enable_channel(dmcan_device_handle* handle, uint8_t channel);
void dmcan_device_disable_channel(dmcan_device_handle* handle, uint8_t channel);
bool dmcan_device_set_channel_baudrate(dmcan_device_handle* handle, uint8_t channel,
                                       dmcan_channel_can_info info);
bool dmcan_device_get_channel_baudrate(dmcan_device_handle* handle, uint8_t channel,
                                       dmcan_channel_can_info* info);
void dmcan_device_hook_recv_callback(dmcan_device_handle* handle, dmcan_frame_callback cb);
void dmcan_device_hook_sent_callback(dmcan_device_handle* handle, dmcan_frame_callback cb);
void dmcan_device_hook_err_callback(dmcan_device_handle* handle, dmcan_frame_callback cb);
bool dmcan_device_send_can(dmcan_device_handle* handle, uint8_t channel, uint32_t can_id,
                            bool canfd, bool ext, bool rtr, bool brs, uint8_t dlen,
                            const uint8_t* data);
uint8_t dmcan_utils_get_len_from_dlc(uint8_t dlc);
uint8_t dmcan_utils_get_dlc_from_len(uint8_t len);

#ifdef __cplusplus
}
#endif
