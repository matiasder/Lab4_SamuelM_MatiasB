#include "display.h"

#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* Mutex compartido con main.c para arbitrar el bus I2C */
extern SemaphoreHandle_t g_i2c_mutex;

static i2c_port_t s_i2c_port = I2C_NUM_0;
static uint8_t s_lcd_address = LCD1602_PCF8574_ADDRESS;

#define LCD_PIN_RS 0x01
/* LCD_PIN_RW eliminado: siempre escribimos, RW debe estar en 0 */
#define LCD_PIN_EN 0x04
#define LCD_PIN_BL 0x08

/* ---------------------------------------------------------------------------
 * FIX 1: lcd_i2c_write ahora toma/libera g_i2c_mutex para no colisionar
 *         con las transacciones del RTC en el mismo bus I2C.
 * ---------------------------------------------------------------------------*/
static void lcd_i2c_write(uint8_t value)
{
    if (g_i2c_mutex != NULL) {
        xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50));
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_lcd_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    (void)i2c_master_cmd_begin(s_i2c_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    if (g_i2c_mutex != NULL) {
        xSemaphoreGive(g_i2c_mutex);
    }
}

static void lcd_write_nibble(uint8_t nibble, uint8_t mode, bool pulse_enable)
{
    uint8_t value = (uint8_t)(nibble & 0xF0);

    /* FIX 2: LCD_PIN_RW eliminado. R/W = 0 indica escritura al HD44780.
     *         Antes tenía | LCD_PIN_RW lo que ponía R/W = 1 (lectura),
     *         haciendo que el LCD ignorara todos los datos enviados. */
    value = (uint8_t)(value | LCD_PIN_BL);

    if ((mode & LCD_DATA) != 0) {
        value |= LCD_PIN_RS;
    }

    if (pulse_enable) {
        lcd_i2c_write((uint8_t)(value | LCD_PIN_EN));
        esp_rom_delay_us(1);
    }

    lcd_i2c_write((uint8_t)(value & (uint8_t)~LCD_PIN_EN));
    esp_rom_delay_us(50);
}

static void lcd_write_byte(uint8_t data, uint8_t mode)
{
    lcd_write_nibble((uint8_t)(data & 0xF0), mode, true);
    lcd_write_nibble((uint8_t)((data << 4) & 0xF0), mode, true);
}

static void lcd_write_command(uint8_t command)
{
    lcd_write_byte(command, LCD_COMMAND);

    if (command == 0x01 || command == 0x02) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void configuracion_i2c(
    i2c_port_t port,
    gpio_num_t sda_pin,
    gpio_num_t scl_pin)
{
    s_i2c_port = port;

    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0,
    };

    i2c_param_config(port, &config);
    i2c_driver_install(port, config.mode, 0, 0, 0);
}

void inicializar_display(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));

    /* FIX 3: pulse_enable cambiado a true en los cuatro nibbles de reset.
     *         Sin el pulso EN el HD44780 nunca captura el comando y queda
     *         en estado indefinido, mostrando basura al encender. */
    lcd_write_nibble(0x30, LCD_COMMAND, true);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write_nibble(0x30, LCD_COMMAND, true);
    esp_rom_delay_us(150);

    lcd_write_nibble(0x30, LCD_COMMAND, true);
    esp_rom_delay_us(150);

    /* Cambio a modo 4 bits */
    lcd_write_nibble(0x20, LCD_COMMAND, true);
    vTaskDelay(pdMS_TO_TICKS(1));

    /* A partir de aquí el bus ya está en 4 bits; usar lcd_write_command */
    lcd_write_command(0x28); /* Function set: 4-bit, 2 líneas, 5x8 */
    lcd_write_command(0x08); /* Display off                         */
    lcd_write_command(0x01); /* Clear display                       */
    lcd_write_command(0x06); /* Entry mode: incrementar, no shift   */
    lcd_write_command(0x0C); /* Display on, cursor off, blink off   */
}

void lcd_clear(void)
{
    lcd_write_command(0x01);
}

void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t address;

    switch (row) {
    case 0:
        address = (uint8_t)(0x00 + col);
        break;
    case 1:
        address = (uint8_t)(0x40 + col);
        break;
    default:
        address = (uint8_t)(0x00 + col);
        break;
    }

    lcd_write_command((uint8_t)(0x80 | address));
}

void enviar_al_lcd(uint8_t data, uint8_t mode)
{
    lcd_write_byte(data, mode);
}