#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c.h"

#define LCD_COMMAND 0x00
#define LCD_DATA 0x01

#ifndef LCD1602_PCF8574_ADDRESS
#define LCD1602_PCF8574_ADDRESS 0x27
#endif

void configuracion_i2c(
    i2c_port_t port,
    gpio_num_t sda_pin,
    gpio_num_t scl_pin);

void inicializar_display(void);

void lcd_clear(void);

void lcd_set_cursor(uint8_t row, uint8_t col);

void enviar_al_lcd(uint8_t data, uint8_t mode);

#endif