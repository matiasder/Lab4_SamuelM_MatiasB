#ifndef BLUETOOTH_SPP_H
#define BLUETOOTH_SPP_H

#include <stdint.h>
#include <stdbool.h>

typedef void (*ble_rx_callback_t)(
    const uint8_t *data,
    uint16_t len,
    void *ctx);

typedef void (*ble_tx_ready_callback_t)(
    bool ready,
    void *ctx);

void bluetooth_init(void);

bool ble_is_connected(void);

bool ble_can_send(void);

void ble_register_rx_callback(
    ble_rx_callback_t callback,
    void *ctx);

void ble_register_tx_ready_callback(
    ble_tx_ready_callback_t callback,
    void *ctx);

int ble_send(
    const uint8_t *data,
    uint16_t len);

int ble_send_string(const char *str);

#endif