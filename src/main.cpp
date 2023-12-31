#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "configuration.h"
#include "shiftOut.h"
#include "kinematics.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "ThetaRhoParser.h"
#include "LEDStrip.h"

#define MOUNT_POINT "/sd"

extern "C" {
    void app_main(void);
    void listFiles(bool &retFlag);
}

void mountSD() {
    esp_err_t err;

    // Options for mounting the filesystem.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(__FUNCTION__, "Initializing SD card");

    ESP_LOGI(__FUNCTION__, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.set_card_clk(host.slot, 1000);
    host.slot = VSPI_HOST;

    //Configure endstop GPIO
	gpio_config_t cs_conf = {
        .pin_bit_mask = ((uint64_t)1<<SD_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&cs_conf));
    gpio_set_level(SD_CS_PIN, 0);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    err = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGE(__FUNCTION__, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SDSPI_SLOT_NO_CS;
    slot_config.host_id = (spi_host_device_t)host.slot;

    ESP_LOGI(__FUNCTION__, "Mounting filesystem");
    err = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (err != ESP_OK) {
        if (err == ESP_FAIL) {
            ESP_LOGE(__FUNCTION__, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(__FUNCTION__, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(err));
        }
        return;
    }
    ESP_LOGI(__FUNCTION__, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}

void app_main() {
    vTaskDelay(2000/portTICK_PERIOD_MS);

    LEDColor_t fgColor = {
        .r = 255,
        .g = 0,
        .b = 0,
        .brightness = 0.5
    };

    LEDColor_t bgColor = {
        .r = 0,
        .g = 255,
        .b = 255,
        .brightness = 0.06
    };

    if (ledInit(LED_DATA_PIN, LED_COUNT) != ESP_OK) {
        ESP_LOGE(__FUNCTION__, "Failed to init LEDs. Restarting.");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        esp_restart();
    }
    //ESP_ERROR_CHECK(ledSetColor(color));
    ledFollowerStart(fgColor, bgColor, (25.0/LED_COUNT)*2*M_PI);

    mountSD();

    kinematicsSetup();
    home();

    ThetaRho_Direction_t currentDirection = THR_DIR_NORMAL;
    ThetaRhoParser* thr;
    float speed = 20.0;

    ESP_LOGI(__FUNCTION__, "Opening playlist");
    std::ifstream playlist;
    playlist.open("/sd/playlist.txt", std::ifstream::in);
    if (!playlist.is_open()) {
        ESP_LOGE(__FUNCTION__, "Failed to open playlist file. Restarting.");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        esp_restart();
    }

    char thrFileName[128];
    while (1) {
        if (!playlist.getline(thrFileName, 128)) {
            ESP_LOGE(__FUNCTION__, "End of playlist, returning to first file.");
            playlist.clear();
            playlist.seekg(0, playlist.beg);
        } else {

            ESP_LOGI(__FUNCTION__, "Opening file %s", thrFileName);
            char pathToFile[128] = "/sd/";
            strcat(pathToFile, thrFileName);
            if (currentDirection == THR_DIR_NORMAL) {
                thr = new ThetaRhoParser(pathToFile, currentDirection);
            } else {
                thr = new ThetaRhoParser(pathToFile, currentDirection);
            }

            if (thr == nullptr || thr->isOpen() != THR_OK) {
                ESP_LOGE(__FUNCTION__, "Failed to open .thr file");
            } else {
                ESP_LOGI(__FUNCTION__, "Total lines: %ld", thr->getTotalLines());

                float theta, rho;
                if (thr->getNextCommand(&theta, &rho) == THR_OK) {
                    while(!setTheta(theta)) {
                        vTaskDelay(1000/portTICK_PERIOD_MS);
                    }
                    move(theta, rho, speed);
                } else {
                    ESP_LOGE(__FUNCTION__, "Failed to get first command");
                    esp_restart();
                }

                while (thr->getNextCommand(&theta, &rho) == THR_OK) {
                    ESP_LOGI(__FUNCTION__, "Line: %ld/%ld", thr->getCurrentLine(), thr->getTotalLines());
                    move(theta, rho, speed);
                }

                delete thr;

                if (currentDirection == THR_DIR_NORMAL) {
                    currentDirection = THR_DIR_REVERSE;
                } else {
                    currentDirection = THR_DIR_NORMAL;
                }
            }
        }
    }
}

void listFiles()
{
    DIR *dir = opendir("/sd");
    if (dir == NULL)
    {
        return;
    }

    while (true)
    {
        struct dirent *de = readdir(dir);
        if (!de)
        {
            break;
        }

        printf("Found file: %s\n", de->d_name);
    }

    closedir(dir);
}
