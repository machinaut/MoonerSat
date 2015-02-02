/* cdcacm.h - cdcacm defines */

#ifndef STM32_CDCACM_H_
#define STM32_CDCACM_H_

// TODO: clean these up
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>

extern const char * usb_strings[];
extern const struct usb_device_descriptor dev;
extern const struct usb_config_descriptor config;
extern uint8_t usbd_control_buffer[128];

void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue);

#endif // !defined STM32_CDCACM_H_
