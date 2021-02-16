/* i2c-tools example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_fat.h"
#include "cmd_system.h"
#include "cmd_i2ctools.h"

#include "argtable3/argtable3.h"
#include "driver/i2c.h"
#include "esp_console.h"
#include "esp_log.h"

#include "tinyusb.h"

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */
static i2c_port_t i2c_port = I2C_NUM_0;

static gpio_num_t i2c_gpio_sda = 33;
static gpio_num_t i2c_gpio_scl = 34;
static uint32_t i2c_frequency = 100000;
// static i2c_port_t i2c_port = I2C_NUM_0;

static const char *TAG = "i2c-tools";

#if CONFIG_EXAMPLE_STORE_HISTORY

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"

static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
#endif // CONFIG_EXAMPLE_STORE_HISTORY

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency
    };
    return i2c_param_config(i2c_port, &conf);
}

static void do_i2cdetect_cmd(void)
{
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();
    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_stop(cmd);
            esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    i2c_driver_delete(i2c_port);
}

void app_main(void)
{
#if CONFIG_EXAMPLE_STORE_HISTORY
    initialize_filesystem();
    // repl_config.history_save_path = HISTORY_PATH;
#endif

    do_i2cdetect_cmd();


//     esp_console_repl_t *repl = NULL;
//     esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
//     esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
// #if CONFIG_EXAMPLE_STORE_HISTORY
//     initialize_filesystem();
//     repl_config.history_save_path = HISTORY_PATH;
// #endif
//     repl_config.prompt = "i2c-tools>";
//     // init console REPL environment
//     ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

//     register_i2ctools();
//     register_system();

//     printf("\n ==============================================================\n");
//     printf(" |             Steps to Use i2c-tools                         |\n");
//     printf(" |                                                            |\n");
//     printf(" |  1. Try 'help', check all supported commands               |\n");
//     printf(" |  2. Try 'i2cconfig' to configure your I2C bus              |\n");
//     printf(" |  3. Try 'i2cdetect' to scan devices on the bus             |\n");
//     printf(" |  4. Try 'i2cget' to get the content of specific register   |\n");
//     printf(" |  5. Try 'i2cset' to set the value of specific register     |\n");
//     printf(" |  6. Try 'i2cdump' to dump all the register (Experiment)    |\n");
//     printf(" |                                                            |\n");
//     printf(" ==============================================================\n\n");

//     // start console REPL
//     ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
