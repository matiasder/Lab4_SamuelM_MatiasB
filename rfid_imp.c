#include "rfid.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "driver/spi_master.h"
#include "driver/rc522_spi.h"

#include "rc522.h"
#include "rc522_picc.h"

static rc522_driver_handle_t driver;
static rc522_handle_t scanner;

static rfid_uid_t last_uid;

enum { RFID_UID_MAX_LEN = sizeof(last_uid.uid) };

static bool new_card = false;

static bool card_present = false;

static SemaphoreHandle_t rfid_mutex;

static void rfid_callback(
    void *arg,
    esp_event_base_t base,
    int32_t event_id,
    void *data)
{
    rc522_picc_state_changed_event_t *event;
    event = (rc522_picc_state_changed_event_t *)data;

    rc522_picc_t *picc = event->picc;

    xSemaphoreTake(rfid_mutex, portMAX_DELAY);

    /*
     * Tarjeta detectada
     */
    if (picc->state == RC522_PICC_STATE_ACTIVE) {

        /*
         * Ya había una tarjeta presente,
         * ignorar lecturas repetidas
         */
        if (card_present) {

            xSemaphoreGive(rfid_mutex);
            return;
        }

        card_present = true;

        uint8_t copy_len = picc->uid.length;
        if (copy_len > RFID_UID_MAX_LEN) {
            copy_len = RFID_UID_MAX_LEN;
        }

        last_uid.length = copy_len;

        memcpy(
            last_uid.uid,
            picc->uid.value,
            copy_len);

        new_card = true;
    }

    /*
     * Tarjeta retirada
     */
    else {

        card_present = false;
    }

    xSemaphoreGive(rfid_mutex);
}


void rfid_init(
    int miso,
    int mosi,
    int sclk,
    int cs)
{
    rfid_mutex = xSemaphoreCreateMutex();

    spi_bus_config_t bus_config = {
        .mosi_io_num = mosi,
        .miso_io_num = miso,
        .sclk_io_num = sclk,

        .quadwp_io_num = -1,
        .quadhd_io_num = -1,

        .max_transfer_sz = 0
    };

    rc522_spi_config_t driver_config = {
        .host_id = SPI2_HOST,

        .bus_config = &bus_config,

        .dev_config = {
            .spics_io_num = cs,
            .clock_speed_hz = 1000000,
        },

        .rst_io_num = -1,
    };

    rc522_spi_create(&driver_config, &driver);

    rc522_driver_install(driver);

    rc522_config_t scanner_config = {
        .driver = driver,
    };

    rc522_create(&scanner_config, &scanner);

    rc522_register_events(
        scanner,
        RC522_EVENT_PICC_STATE_CHANGED,
        rfid_callback,
        NULL);

    rc522_start(scanner);
}

bool rfid_card_available(void)
{

    bool available;

    xSemaphoreTake(rfid_mutex, portMAX_DELAY);

    available = new_card;

    xSemaphoreGive(rfid_mutex);

    return available;
}
bool rfid_get_uid(rfid_uid_t *out_uid)
{
    bool success = false;

    xSemaphoreTake(rfid_mutex, portMAX_DELAY);

    if (new_card) {

        memcpy(out_uid, &last_uid, sizeof(rfid_uid_t));

        new_card = false;

        success = true;
    }

    xSemaphoreGive(rfid_mutex);

    return success;
}