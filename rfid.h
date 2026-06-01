#ifndef RFID_H
#define RFID_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t uid[10];
    uint8_t length;
} rfid_uid_t;

void rfid_init(
    int miso,
    int mosi,
    int sclk,
    int cs);

bool rfid_card_available(void);

bool rfid_get_uid(rfid_uid_t *out_uid);

#endif