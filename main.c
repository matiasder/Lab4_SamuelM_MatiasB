#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "bluetooth_spp.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "rfid.h"

/* ──────────────────── PINES ──────────────────── */
#define LED_RED_PIN   GPIO_NUM_18
#define LED_GREEN_PIN GPIO_NUM_19
#define LED_BLUE_PIN  GPIO_NUM_21

#define BUZZER_PIN            GPIO_NUM_12
#define BUZZER_PWM_MODE       LEDC_LOW_SPEED_MODE
#define BUZZER_PWM_TIMER      LEDC_TIMER_0
#define BUZZER_PWM_CHANNEL    LEDC_CHANNEL_0
#define BUZZER_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define BUZZER_SHORT_FREQ_HZ  2000
#define BUZZER_LONG_FREQ_HZ   1000

#define RESET_PIN   17
#define RFID_MISO_PIN 26
#define RFID_MOSI_PIN 25
#define RFID_SCLK_PIN 27
#define RFID_CS_PIN   14

#define RTC_I2C_PORT  I2C_NUM_0
#define RTC_SDA_PIN   GPIO_NUM_23
#define RTC_SCL_PIN   GPIO_NUM_22

#define DISPLAY_I2C_PORT I2C_NUM_1
#define DISPLAY_SDA_PIN  GPIO_NUM_33
#define DISPLAY_SCL_PIN  GPIO_NUM_32

#define RTC_DS1307_ADDRESS   0x68
#define LCD_PCF8574_ADDRESS  0x27   /* Cambia a 0x3F si tu módulo lo requiere */

#define RFID_POLL_DELAY_MS     20
#define DISPLAY_REFRESH_MS    150
#define EVENT_MESSAGE_TIMEOUT_MS 2000
#define ACTIVE_MESSAGE_MAX_LEN 16

#define NVS_NAMESPACE_APP      "app_state"
#define NVS_KEY_LAST_MESSAGE   "last_msg"

/* ──────────────────── PCF8574 → LCD 1602 (4-bit) ──────────────────── */
/*
 * Mapeo de bits del PCF8574 al bus del HD44780:
 *   P0 = RS   P1 = RW   P2 = EN   P3 = BL (backlight)
 *   P4 = D4   P5 = D5   P6 = D6   P7 = D7
 */
#define LCD_RS  0x01
#define LCD_RW  0x02
#define LCD_EN  0x04
#define LCD_BL  0x08   /* backlight siempre encendido */

static TickType_t ignore_reads_until = 0;

/* lcd_write_i2c — usa DISPLAY_I2C_PORT (I2C_NUM_1), sin mutex
 * (solo la tarea control_display lo llama) */
static void lcd_write_i2c(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_PCF8574_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data | LCD_BL, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(DISPLAY_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

static void lcd_pulse(uint8_t data)
{
    lcd_write_i2c(data);            /* dato estable, EN=0 */
    esp_rom_delay_us(1);
    lcd_write_i2c(data | LCD_EN);   /* flanco de subida EN */
    esp_rom_delay_us(1);
    lcd_write_i2c(data & ~LCD_EN);  /* flanco de bajada EN */
    esp_rom_delay_us(50);
}

static void lcd_write4(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble << 4);   /* nibble a P4-P7 */
    if (mode) data |= LCD_RS;
    lcd_pulse(data);
}

static void lcd_send(uint8_t value, uint8_t mode)
{
    lcd_write4(value >> 4,   mode); /* nibble alto primero */
    lcd_write4(value & 0x0F, mode); /* nibble bajo */
}

static void lcd_cmd(uint8_t cmd)
{
    lcd_send(cmd, 0);
    if (cmd == 0x01 || cmd == 0x02)
        vTaskDelay(pdMS_TO_TICKS(5));
}

static void lcd_char(char c)
{
    lcd_send((uint8_t)c, 1);
}

static void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd_write4(0x03, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write4(0x03, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write4(0x03, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write4(0x02, 0); /* cambia a 4 bits */
    lcd_cmd(0x28);       /* 4-bit, 2 líneas lógicas (2004 usa 2x en memoria), 5x8 */
    lcd_cmd(0x0C);       /* display ON, cursor OFF */
    lcd_cmd(0x06);       /* entry mode: incremento, sin shift */
    lcd_cmd(0x01);       /* clear */
    vTaskDelay(pdMS_TO_TICKS(5));
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
    /* Offsets estándar para LCD 2004 (y compatibles 1602):
     *   Fila 0 → 0x00, Fila 1 → 0x40, Fila 2 → 0x14, Fila 3 → 0x54 */
    const uint8_t rows[] = {0x00, 0x40, 0x14, 0x54};
    lcd_cmd(0x80 | (col + rows[row]));
}

static void lcd_write_line(uint8_t row, const char *text)
{
    char buf[21];
    snprintf(buf, sizeof(buf), "%-20.20s", text);
    lcd_set_cursor(row, 0);
    for (int i = 0; i < 20; i++) lcd_char(buf[i]);
}


/* ──────────────────── I2C ──────────────────── */
SemaphoreHandle_t g_i2c_mutex = NULL;

static void configuracion_i2c(i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = sda,
        .scl_io_num       = scl,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(port, &cfg);
    i2c_driver_install(port, cfg.mode, 0, 0, 0);
}

/* ──────────────────── TIPOS ──────────────────── */
typedef enum {
    SIS_BLOQUEADO,
    SIS_CONCEDIDO,
    SIS_DENEGADO,
    SIS_ACTIVO,
} system_state_t;

typedef struct {
    uint8_t  second;
    uint8_t  minute;
    uint8_t  hour;
    uint8_t  day_of_week;
    uint8_t  day;
    uint8_t  month;
    uint16_t year;
} rtc_datetime_t;

/* ──────────────────── ESTADO GLOBAL ──────────────────── */
static SemaphoreHandle_t g_state_mutex = NULL;

static volatile bool   acceso_denegado       = false;
static system_state_t  system_state          = SIS_BLOQUEADO;
static bool            ui_dirty              = true;
static TickType_t      state_deadline        = 0;
static TickType_t      denied_event_start    = 0;

static uint8_t last_uid[10] = {0};
static uint8_t last_uid_length = 0;

static char active_message[17]      = "Sin mensajes";
static bool active_message_received = false;

static char ui_line1[17] = "Panel bloqueado";
static char ui_line2[17] = "Acerque credenc";

static rtc_datetime_t rtc_current_time = {
    .second = 0, .minute = 50, .hour = 12,
    .day_of_week = 1, .day = 1, .month = 1, .year = 2026,
};

static const uint8_t llave_admin[] = { 0xD9, 0x00, 0xE3, 0x56 };

/* ──────────────────── BUZZER ──────────────────── */
static void buzzer_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode       = BUZZER_PWM_MODE,
        .duty_resolution  = BUZZER_PWM_RESOLUTION,
        .timer_num        = BUZZER_PWM_TIMER,
        .freq_hz          = BUZZER_SHORT_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .speed_mode = BUZZER_PWM_MODE,
        .channel    = BUZZER_PWM_CHANNEL,
        .timer_sel  = BUZZER_PWM_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = BUZZER_PIN,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&c);
}

static void buzzer_stop(void)
{
    ledc_set_duty(BUZZER_PWM_MODE, BUZZER_PWM_CHANNEL, 0);
    ledc_update_duty(BUZZER_PWM_MODE, BUZZER_PWM_CHANNEL);
}

static void buzzer_beep(uint32_t freq_hz, uint32_t duration_ms)
{
    ledc_set_freq(BUZZER_PWM_MODE, BUZZER_PWM_TIMER, freq_hz);
    uint32_t duty = (1U << BUZZER_PWM_RESOLUTION) / 2U;
    ledc_set_duty(BUZZER_PWM_MODE, BUZZER_PWM_CHANNEL, duty);
    ledc_update_duty(BUZZER_PWM_MODE, BUZZER_PWM_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    buzzer_stop();
}

/* ──────────────────── RTC DS1307 ──────────────────── */
static bool rtc_i2c_write(uint8_t reg, const uint8_t *data, size_t len)
{
    if (!g_i2c_mutex) return false;
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RTC_DS1307_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (len > 0) i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(RTC_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(g_i2c_mutex);
    return r == ESP_OK;
}

static bool rtc_i2c_read(uint8_t reg, uint8_t *data, size_t len)
{
    if (!g_i2c_mutex) return false;
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RTC_DS1307_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RTC_DS1307_ADDRESS << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(RTC_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    xSemaphoreGive(g_i2c_mutex);
    return r == ESP_OK;
}

static uint8_t bcd_to_bin(uint8_t v) { return (uint8_t)(((v >> 4) * 10U) + (v & 0x0F)); }
static uint8_t bin_to_bcd(uint8_t v) { return (uint8_t)(((v / 10U) << 4) | (v % 10U)); }

static bool rtc_read_datetime(rtc_datetime_t *out)
{
    uint8_t raw[7] = {0};
    if (!rtc_i2c_read(0x00, raw, 7)) return false;
    out->second      = bcd_to_bin(raw[0] & 0x7F);
    out->minute      = bcd_to_bin(raw[1] & 0x7F);
    out->hour        = bcd_to_bin(raw[2] & 0x3F);
    out->day_of_week = bcd_to_bin(raw[3] & 0x07);
    out->day         = bcd_to_bin(raw[4] & 0x3F);
    out->month       = bcd_to_bin(raw[5] & 0x1F);
    out->year        = (uint16_t)(2000U + bcd_to_bin(raw[6]));
    return true;
}

static bool rtc_write_datetime(const rtc_datetime_t *dt)
{
    uint8_t raw[7];
    raw[0] = bin_to_bcd(dt->second      & 0x7F);
    raw[1] = bin_to_bcd(dt->minute      & 0x7F);
    raw[2] = bin_to_bcd(dt->hour        & 0x3F);
    raw[3] = bin_to_bcd(dt->day_of_week & 0x07);
    raw[4] = bin_to_bcd(dt->day         & 0x3F);
    raw[5] = bin_to_bcd(dt->month       & 0x1F);
    raw[6] = bin_to_bcd((uint8_t)(dt->year % 100U));
    return rtc_i2c_write(0x00, raw, 7);
}

static bool rtc_is_running(void)
{
    uint8_t sec = 0;
    if (!rtc_i2c_read(0x00, &sec, 1)) return false;
    return (sec & 0x80) == 0;
}

static bool rtc_parse_compile_date(rtc_datetime_t *out)
{
    static const char *months[] = {
        "Jan","Feb","Mar","Apr","May","Jun",
        "Jul","Aug","Sep","Oct","Nov","Dec"
    };
    int day, year, hour, minute, second;
    char month_text[4] = {0};

    if (sscanf(__DATE__, "%3s %d %d", month_text, &day, &year) != 3) return false;
    if (sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second) != 3) return false;

    int month = 1;
    for (int i = 0; i < 12; ++i) {
        if (strcmp(month_text, months[i]) == 0) { month = i + 1; break; }
    }
    out->day         = (uint8_t)day;
    out->month       = (uint8_t)month;
    out->year        = (uint16_t)year;
    out->hour        = (uint8_t)hour;
    out->minute      = (uint8_t)minute;
    out->second      = (uint8_t)second;
    out->day_of_week = 1;
    return true;
}

static void rtc_print_debug(const rtc_datetime_t *dt)
{
    printf("[RTC] %04u-%02u-%02u %02u:%02u:%02u\n",
           dt->year, dt->month, dt->day,
           dt->hour, dt->minute, dt->second);
}

static void rtc_bootstrap_if_needed(void)
{
    rtc_datetime_t dt;
    if (rtc_is_running() && rtc_read_datetime(&dt)) {
        rtc_current_time = dt;
        rtc_print_debug(&rtc_current_time);
        return;
    }
    if (rtc_parse_compile_date(&dt)) {
        dt.hour = 12;
        dt.minute = 50;
        dt.second = 0;
    }
    if (rtc_write_datetime(&dt)) {
        rtc_current_time = dt;
        rtc_print_debug(&rtc_current_time);
        return;
    }
    rtc_print_debug(&rtc_current_time);
}

static bool normalize_message_payload(const uint8_t *data, uint16_t len, char *out, size_t out_size)
{
    if (!data || !out || out_size < 2 || len == 0) return false;

    size_t w = 0;
    for (uint16_t i = 0; i < len && w < (out_size - 1); ++i) {
        uint8_t c = data[i];
        if (c == '\r' || c == '\n') break;
        if (isprint((int)c)) {
            out[w++] = (char)c;
        }
    }
    out[w] = '\0';
    return w > 0;
}

static void persist_last_message(const char *msg)
{
    if (!msg || msg[0] == '\0') return;

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE_APP, NVS_READWRITE, &handle) != ESP_OK) return;

    if (nvs_set_str(handle, NVS_KEY_LAST_MESSAGE, msg) == ESP_OK) {
        (void)nvs_commit(handle);
    }
    nvs_close(handle);
}

static void restore_last_message_from_nvs(void)
{
    nvs_handle_t handle;
    char stored[ACTIVE_MESSAGE_MAX_LEN + 1] = {0};
    size_t stored_len = sizeof(stored);

    if (nvs_open(NVS_NAMESPACE_APP, NVS_READONLY, &handle) != ESP_OK) return;

    if (nvs_get_str(handle, NVS_KEY_LAST_MESSAGE, stored, &stored_len) == ESP_OK && stored[0] != '\0') {
        snprintf(active_message, sizeof(active_message), "%s", stored);
        active_message_received = true;
        ui_dirty = true;
    }

    nvs_close(handle);
}

static bool try_update_rtc_time_from_ble(const uint8_t *data, uint16_t len)
{
    char payload[32] = {0};
    int hour, minute, second;

    if (!normalize_message_payload(data, len, payload, sizeof(payload))) return false;

    if (sscanf(payload, "HORA=%d:%d:%d", &hour, &minute, &second) != 3 &&
        sscanf(payload, "TIME=%d:%d:%d", &hour, &minute, &second) != 3) {
        return false;
    }

    if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
        return true;
    }

    rtc_datetime_t new_time;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    new_time = rtc_current_time;
    xSemaphoreGive(g_state_mutex);

    new_time.hour = (uint8_t)hour;
    new_time.minute = (uint8_t)minute;
    new_time.second = (uint8_t)second;

    if (rtc_write_datetime(&new_time)) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        rtc_current_time = new_time;
        ui_dirty = true;
        xSemaphoreGive(g_state_mutex);

        if (ble_can_send()) {
            ble_send_string("Hora actualizada\r\n");
        }
    }

    return true;
}

/* ──────────────────── HELPERS ──────────────────── */
static bool uid_checker(const rfid_uid_t *uid, const uint8_t *ref, uint8_t len)
{
    if (uid->length != len) return false;
    return memcmp(uid->uid, ref, len) == 0;
}

static void format_uid_preview(
    const uint8_t *uid, uint8_t length,
    char *out, size_t out_size)
{
    if (out_size == 0) return;
    out[0] = '\0';
    size_t used = 0;
    uint8_t preview = (length > 4) ? 4 : length;
    for (uint8_t i = 0; i < preview; ++i) {
        int w = snprintf(out + used, out_size - used, "%02X", uid[i]);
        if (w < 0 || (size_t)w >= out_size - used) break;
        used += (size_t)w;
    }
    if (length > 4 && used + 2 < out_size) snprintf(out + used, out_size - used, "..");
}

/* ──────────────────── UI / DISPLAY ──────────────────── */
static void set_status_screen_locked(void)
{
    char time_line[17];
    snprintf(time_line, sizeof(time_line), "%02u:%02u:%02u",
             rtc_current_time.hour,
             rtc_current_time.minute,
             rtc_current_time.second);

    switch (system_state) {
    case SIS_BLOQUEADO:
        snprintf(ui_line1, sizeof(ui_line1), "Panel bloqueado");
        snprintf(ui_line2, sizeof(ui_line2), "Acerque credenc");
        break;
    case SIS_CONCEDIDO:
        snprintf(ui_line1, sizeof(ui_line1), "Acceso concedido");
        snprintf(ui_line2, sizeof(ui_line2), "%s", time_line);
        break;
    case SIS_DENEGADO:
        snprintf(ui_line1, sizeof(ui_line1), "Acceso denegado");
        snprintf(ui_line2, sizeof(ui_line2), "UID no reg.");
        break;
    case SIS_ACTIVO:
        snprintf(ui_line1, sizeof(ui_line1), "%s",
                 active_message_received ? active_message : "Sin mensajes");
        snprintf(ui_line2, sizeof(ui_line2), "%s", time_line);
        break;
    }
}

/* Renderiza ambas líneas en el LCD físico.
 * NOTA: El LCD está en I2C_NUM_1 que NO comparte bus con el RTC (I2C_NUM_0),
 * por lo que no necesita g_i2c_mutex. Se usa un mutex propio si se quisiera
 * proteger acceso concurrente desde varias tareas, pero aquí solo
 * control_display lo llama, así que no hace falta. */
static void render_display(void)
{
    lcd_write_line(0, ui_line1);
    lcd_write_line(1, ui_line2);
}


/* ──────────────────── BLE ──────────────────── */
static void send_ble_snapshot(const char *prefix)
{
    if (!ble_can_send()) return;

    char uid_preview[12];
    char message[128];
    system_state_t state;
    bool denied;
    uint8_t uid_length;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    state      = system_state;
    denied     = acceso_denegado;
    uid_length = last_uid_length;
    if (uid_length > 0)
        format_uid_preview(last_uid, uid_length, uid_preview, sizeof(uid_preview));
    else
        snprintf(uid_preview, sizeof(uid_preview), "SIN TARJETA");
    xSemaphoreGive(g_state_mutex);

    snprintf(message, sizeof(message),
             "%s | estado=%s | acceso=%s | uid=%s\r\n",
             prefix,
             (state == SIS_ACTIVO) ? "ACTIVO" : "BLOQUEADO",
             denied ? "DENEGADO" : "OK",
             uid_preview);

    ble_send_string(message);
}

static void ble_rx_callback(const uint8_t *data, uint16_t len, void *ctx)
{
    (void)ctx;

    if (try_update_rtc_time_from_ble(data, len)) {
        return;
    }

    char incoming[ACTIVE_MESSAGE_MAX_LEN + 1] = {0};
    bool should_persist = false;

    if (!normalize_message_payload(data, len, incoming, sizeof(incoming))) {
        return;
    }

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    if (system_state == SIS_ACTIVO) {
        snprintf(active_message, sizeof(active_message), "%s", incoming);
        active_message_received  = true;
        ui_dirty = true;
        should_persist = true;
    }
    xSemaphoreGive(g_state_mutex);

    if (should_persist) {
        persist_last_message(incoming);
    }
}

static void ble_tx_ready_callback(bool ready, void *ctx)
{
    (void)ctx;
    if (ready) send_ble_snapshot("BLE READY");
}

/* ──────────────────── LÓGICA DE ACCESO ──────────────────── */
static void update_access_state(const rfid_uid_t *uid)
{
    bool authorized = uid_checker(uid, llave_admin, sizeof(llave_admin));
    system_state_t prev;
    uint8_t copy_len = uid->length;

    if (copy_len > (uint8_t)sizeof(last_uid)) {
        copy_len = (uint8_t)sizeof(last_uid);
    }

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    prev = system_state;
    last_uid_length = copy_len;
    memcpy(last_uid, uid->uid, copy_len);

    if (prev == SIS_ACTIVO) {
        if (authorized) {
            system_state            = SIS_BLOQUEADO;
            acceso_denegado         = false;
            ui_dirty = true;
        }
        xSemaphoreGive(g_state_mutex);
        if (authorized) buzzer_beep(BUZZER_SHORT_FREQ_HZ, 500);
        return;
    }

    if (prev != SIS_BLOQUEADO) {
        xSemaphoreGive(g_state_mutex);
        return;
    }

    if (authorized) {
        system_state    = SIS_CONCEDIDO;
        acceso_denegado = false;

        state_deadline  = xTaskGetTickCount() + pdMS_TO_TICKS(1000);

        /* Ignorar nuevas lecturas por 2 segundos */
        ignore_reads_until = xTaskGetTickCount() + pdMS_TO_TICKS(2000);

        ui_dirty = true;

        xSemaphoreGive(g_state_mutex);

        buzzer_beep(BUZZER_SHORT_FREQ_HZ, 500);
        return;
    }

    /* No autorizado */
    system_state       = SIS_DENEGADO;
    acceso_denegado    = true;
    denied_event_start = xTaskGetTickCount();
    state_deadline     = denied_event_start + pdMS_TO_TICKS(2000);
    ui_dirty           = true;
    xSemaphoreGive(g_state_mutex);
    buzzer_beep(BUZZER_LONG_FREQ_HZ, 2000);
}

static void reset_access_state(void)
{
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    system_state            = SIS_BLOQUEADO;
    acceso_denegado         = false;
    state_deadline          = 0;
    last_uid_length         = 0;
    memset(last_uid, 0, sizeof(last_uid));
    ui_dirty = true;
    xSemaphoreGive(g_state_mutex);
}

/* ──────────────────── CONFIGURACIÓN HW ──────────────────── */
static void configurar_leds(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_RED_PIN) |
                        (1ULL << LED_GREEN_PIN) |
                        (1ULL << LED_BLUE_PIN),
        .mode           = GPIO_MODE_OUTPUT,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(LED_RED_PIN,   0);
    gpio_set_level(LED_GREEN_PIN, 0);
    gpio_set_level(LED_BLUE_PIN,  0);
}

static void configurar_reset_pin(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << RESET_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

static void configurar_buzzer(void)
{
    buzzer_init();
    buzzer_stop();
}

/* ──────────────────── ACTUALIZAR LEDs ──────────────────── */
static void actualizar_leds(void)
{
    TickType_t now = xTaskGetTickCount();

    if (system_state == SIS_DENEGADO) {
        TickType_t elapsed = now - denied_event_start;
        gpio_set_level(LED_RED_PIN,
            ((elapsed / pdMS_TO_TICKS(250)) % 2U) == 0U ? 1 : 0);
    } else {
        gpio_set_level(LED_RED_PIN,
            system_state == SIS_BLOQUEADO ? 1 : 0);
    }

    gpio_set_level(LED_GREEN_PIN, system_state == SIS_CONCEDIDO ? 1 : 0);
    gpio_set_level(LED_BLUE_PIN,  system_state == SIS_ACTIVO    ? 1 : 0);
}

/* ──────────────────── TAREAS FREERTOS ──────────────────── */
static void control_state(void *arg)
{
    (void)arg;
    while (true) {
        TickType_t now = xTaskGetTickCount();
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);

        if (system_state == SIS_CONCEDIDO && now >= state_deadline) {
            system_state            = SIS_ACTIVO;
            ui_dirty = true;
        }
        if (system_state == SIS_DENEGADO && now >= state_deadline) {
            system_state    = SIS_BLOQUEADO;
            acceso_denegado = false;
            ui_dirty        = true;
        }

        xSemaphoreGive(g_state_mutex);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void control_rtc(void *arg)
{
    (void)arg;
    while (true) {
        rtc_datetime_t dt;
        if (rtc_read_datetime(&dt)) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            rtc_current_time = dt;
            rtc_print_debug(&rtc_current_time);
            ui_dirty = true;
            xSemaphoreGive(g_state_mutex);
        } else {
            rtc_bootstrap_if_needed();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void control_acceso(void *arg)
{
    (void)arg;
    rfid_uid_t uid;

    static uint8_t last_uid[10] = {0};
    static uint8_t last_len = 0;

    bool tarjeta_presente = false;

    while (true) {

        if (rfid_get_uid(&uid)) {

            bool misma_tarjeta =
                tarjeta_presente &&
                uid.length == last_len &&
                memcmp(uid.uid, last_uid, uid.length) == 0;

            /* SOLO actuar cuando la tarjeta aparece nuevamente */
            if (!misma_tarjeta) {

            if (xTaskGetTickCount() < ignore_reads_until) {
                vTaskDelay(pdMS_TO_TICKS(RFID_POLL_DELAY_MS));
                continue;
            }

                uint8_t copy_len = uid.length;
                if (copy_len > (uint8_t)sizeof(last_uid)) {
                    copy_len = (uint8_t)sizeof(last_uid);
                }

                memcpy(last_uid, uid.uid, copy_len);
                last_len = copy_len;

                tarjeta_presente = true;

                update_access_state(&uid);
            }

        } else {

            /* La tarjeta fue retirada */
            tarjeta_presente = false;
            last_len = 0;
            memset(last_uid, 0, sizeof(last_uid));
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

static void control_leds(void *arg)
{
    (void)arg;
    while (true) {
        actualizar_leds();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void control_reset(void *arg)
{
    (void)arg;
    TickType_t last_trigger = 0;
    while (true) {
        if (gpio_get_level(RESET_PIN) == 0) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_trigger) > pdMS_TO_TICKS(250)) {
                last_trigger = now;
                reset_access_state();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void control_display(void *arg)
{
    (void)arg;
    char line1[21], line2[21];

    while (true) {
        bool needs_render = false;

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        if (ui_dirty) {
            set_status_screen_locked();   /* actualiza ui_line1 / ui_line2 */
            ui_dirty     = false;
            needs_render = true;
        }
        /* En estado activo o concedido, refrescar la hora cada ciclo */
        if (system_state == SIS_ACTIVO || system_state == SIS_CONCEDIDO) {
            snprintf(ui_line2, sizeof(ui_line2), "%02u:%02u:%02u",
                     rtc_current_time.hour,
                     rtc_current_time.minute,
                     rtc_current_time.second);
            needs_render = true;
        }
        snprintf(line1, sizeof(line1), "%s", ui_line1);
        snprintf(line2, sizeof(line2), "%s", ui_line2);
        xSemaphoreGive(g_state_mutex);

        if (needs_render) {
            lcd_write_line(0, line1);
            lcd_write_line(1, line2);
        }

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
    }
}

/* ──────────────────── app_main ──────────────────── */
void app_main(void)
{

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    g_i2c_mutex   = xSemaphoreCreateMutex();
    g_state_mutex = xSemaphoreCreateMutex();

    if (!g_i2c_mutex || !g_state_mutex) return;

    configurar_leds();
    configurar_reset_pin();
    configurar_buzzer();

    /* I2C_NUM_0: RTC DS1307 */
    configuracion_i2c(RTC_I2C_PORT,    RTC_SDA_PIN,     RTC_SCL_PIN);
    /* I2C_NUM_1: LCD 1602 vía PCF8574 */
    configuracion_i2c(DISPLAY_I2C_PORT, DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);

    /* Inicializar LCD directamente (no hay mutex necesario aquí, tarea única) */
    lcd_init();

    restore_last_message_from_nvs();
    rtc_bootstrap_if_needed();

    /* Pantalla inicial */
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    set_status_screen_locked();
    xSemaphoreGive(g_state_mutex);
    lcd_write_line(0, ui_line1);
    lcd_write_line(1, ui_line2);

    ble_register_rx_callback(ble_rx_callback, NULL);
    ble_register_tx_ready_callback(ble_tx_ready_callback, NULL);
    bluetooth_init();

    rfid_init(RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SCLK_PIN, RFID_CS_PIN);

    xTaskCreate(control_state,   "control_state",   2048, NULL, 4, NULL);
    xTaskCreate(control_acceso,  "control_acceso",  4096, NULL, 5, NULL);
    xTaskCreate(control_leds,    "control_leds",    2048, NULL, 3, NULL);
    xTaskCreate(control_reset,   "control_reset",   2048, NULL, 4, NULL);
    xTaskCreate(control_display, "control_display", 4096, NULL, 4, NULL);
    xTaskCreate(control_rtc,     "control_rtc",     4096, NULL, 4, NULL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
