/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_RMT

static led_strip_handle_t led_strip;

static void blink_led(void)
{
  /* If the addressable LED is enabled */
  if (s_led_state)
  {
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    led_strip_set_pixel(led_strip, 0, 16, 16, 16);
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
  }
  else
  {
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
  }
}

static void configure_led(void)
{
  ESP_LOGI(TAG, "Example configured to blink addressable LED!");
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
      .strip_gpio_num = BLINK_GPIO,
      .max_leds = 1, // at least one LED on board
  };
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000, // 10MHz
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  /* Set all LED off to clear all pixels */
  led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
  /* Set the GPIO level according to the state (LOW or HIGH)*/
  gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
  ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
  gpio_reset_pin(BLINK_GPIO);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#endif
void connect_direct_to_ads1115();
void app_main(void)
{

  /* Configure the peripheral according to the LED type */
  configure_led();

  while (1)
  {
    ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
    blink_led();
    connect_direct_to_ads1115();
    /* Toggle the LED state */
    s_led_state = !s_led_state;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

#include <driver/i2c.h>

static i2c_port_t i2c_port = I2C_NUM_0;

static esp_err_t i2c_master_driver_initialize(void)
{
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = 21,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = 22,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 100000,
      // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
  };
  return i2c_param_config(I2C_NUM_0, &conf);
}

void connect_direct_to_ads1115()
{
  uint8_t address = 0x90;
  // 0x91 read operation
  // 0x90 write operation

  i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
  i2c_master_driver_initialize();

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  esp_err_t ret_err = ESP_OK;

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, 0x90, 1);
  i2c_master_write_byte(cmd, 0x01, 1);
  // i2c_master_write_byte(cmd, 0xCD, 1);
  i2c_master_write_byte(cmd, 0xC3, 1);
  i2c_master_write_byte(cmd, 0x83, 1);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, 0x90, 1);
  i2c_master_write_byte(cmd, 0x00, 1);

  uint8_t read_byte_1 = 0;
  uint8_t read_byte_2 = 0;
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, 0x91, 0);
  i2c_master_read_byte(cmd, &read_byte_1, 0);
  i2c_master_read_byte(cmd, &read_byte_2, 0);

  i2c_master_stop(cmd);
  //  WRITE POINTERS TO CONVERSIOBN
  //  WRITE POINTERS TO CONVERSIOBN
  //  WRITE POINTERS TO CONVERSIOBN

  // // set config
  // ret_err += i2c_master_start(cmd);
  // ret_err += i2c_master_write_byte(cmd, 0x90, 0x1); // connect to addr
  // uint8_t data_to_send[3] = {0x01, 0xCD, 0x83};
  // ret_err += i2c_master_write(cmd, data_to_send, sizeof(data_to_send), 1);
  // printf("data to send size: %i\n", sizeof(data_to_send));

  // ret_err += i2c_master_write_byte(cmd, 0x91, 0x1); // connect to addr
  // uint8_t data_to_res = 0;
  // i2c_master_read(cmd, &data_to_res, 3, 1);

  // ret_err += i2c_master_start(cmd);
  // ret_err += i2c_master_write_byte(cmd, 0x90, 0x1); // connect to addr
  // ret_err += i2c_master_write_byte(cmd, 0x1, 0x1);  // connect to addr
  // ret_err += i2c_master_start(cmd);
  // ret_err += i2c_master_write_byte(cmd, 0x91, 0x1); // connect to addr
  // uint8_t data_to_res_2 = 0;
  // i2c_master_read(cmd, &data_to_res_2, 3, 0);
  // ret_err += i2c_master_write_byte(cmd, 0x01, 0x1); // point to config
  // ret_err += i2c_master_write_byte(cmd, 0xC1, 0x1);     // set first byte
  // ret_err += i2c_master_write_byte(cmd, 0x83, 0x1);     // set second byte

  // ret_err += i2c_master_start(cmd);
  // ret_err += i2c_master_write_byte(cmd, 0x91, 0x1);        // set second byte
  // ret_err += i2c_master_write_byte(cmd, 0x00, 0x1);        // set second byte
  // ret_err += i2c_master_read_byte(cmd, &data_2_conv, 0x1); // set second byte

  // ret_err += i2c_master_start(cmd);
  // ret_err += i2c_master_write_byte(cmd, 0x91, 0x1); // connect to addr
  // ret_err += i2c_master_read_byte(cmd, &data_2, 0x1); // point pointers config reg
  // ret_err += i2c_master_read_byte(cmd, &data_conv, 0x1);   // point pointers config reg
  // ret_err += i2c_master_read_byte(cmd, &data_2_conv, 0x1); // point pointers config reg
  // ret_err += i2c_master_start(cmd);
  // ret_err += i2c_master_write_byte(cmd, 0x90, 0x1); // connect to addr
  // ret_err += i2c_master_write_byte(cmd, 0, 0x1);    // connect to addr
  // ret_err += i2c_master_stop(cmd);

  // ret_err += i2c_master_start(cmd);
  // ret_err += i2c_master_write_byte(cmd, 0x91, 0x1); // connect to addr
  // ret_err += i2c_master_read_byte(cmd, &data_conv, 0x1);
  // ret_err += i2c_master_read_byte(cmd, &data_2_conv, 0x1);

  // ret_err += i2c_master_start(cmd);
  // ret_err += i2c_master_write_byte(cmd, 0x90, 0x1); // connect to addr
  // ret_err += i2c_master_write_byte(cmd, 0x0, 0x1);  // set pointer to conversion

  // ret_err += i2c_master_start(cmd);

  // ret_err += i2c_master_read_byte(cmd, &data_2, 0x1); // point pointers config reg

  // ret_err += i2c_master_start(cmd);
  // ret_err += i2c_master_write_byte(cmd, 0x90, 0x1); // connect to addr
  // ret_err += i2c_master_write_byte(cmd, 0x00, 0x1); // point pointers config reg

  // ret_err += i2c_master_start(cmd);
  // ret_err += i2c_master_write_byte(cmd, 0x91, 0x1);       // point pointers config reg
  // ret_err += i2c_master_read(cmd, &data, 1, 0x1);         // point pointers config reg
  // ret_err += i2c_master_read_byte(cmd, &data_2 + 1, 0x1); // point pointers config reg
  // ret_err += i2c_master_stop(cmd); // stop i2c commands

  esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_PERIOD_MS); // send byte every 50ms
  uint16_t total = read_byte_1 << 8 | read_byte_2;
  if (ret == ESP_OK)
  {
    printf("Connected to ads1115\n");
    printf("Total %i \n", total);
    printf("Total Volts %f \n", total * (4096.0 / 32768.0));
    // printf("read_byte_1 %i  0x%02x \n", read_byte_1, read_byte_1);
    // printf("read_byte_2 %i  0x%02x \n", read_byte_2, read_byte_2);
    // printf("read_byte_1_2 sum %i  0x%02x \n", read_byte_1 + read_byte_2, read_byte_1 + read_byte_2);
    // printf("read_byte_1_2 sum test %i   \n", read_byte_1 << 8 | read_byte_2);
    // 00000000 1 1 1 1 1 1 11
    //
  }
  else
  {
    printf("Cloud not connect to ads1115\n");
    printf("Cloud not connect to ads1115 ret %i\n", ret);
    printf("Cloud not connect to ads1115 ret %s\n", esp_err_to_name(ret));
  }
  i2c_master_stop(cmd);
  i2c_cmd_link_delete(cmd); // clear i2c
  i2c_driver_delete(i2c_port);
}
