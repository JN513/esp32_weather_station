#include "bh1750.h" // Biblioteca de BH1750
#include "bme680.h"
#include "bmp280.h" // Biblioteca de BMP280/BME280
#include "dht.h"
#include "driver/adc.h" // Biblioteca de ADC (Comunicação Analogica)
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ssd1306.h"
#include <esp_adc_cal.h> // Biblioteca de calibragem de ADC
#include <stdio.h>
#include <string.h>

#if CONFIG_IDF_TARGET_ESP32 // Se o target for ESP32

#define SDA_GPIO GPIO_NUM_21 // GPIO do SDA
#define SCL_GPIO GPIO_NUM_22 // GPIO do SCL
// static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

#elif CONFIG_IDF_TARGET_ESP32S2 // Se o target for ESP32S2

#define SDA_GPIO GPIO_NUM_33 // GPIO do SDA
#define SCL_GPIO GPIO_NUM_34 // GPIO do SCL
// static const adc_bits_width_t width = ADC_WIDTH_BIT_13; // Tamanho do ADC

#endif

#if defined(CONFIG_EXAMPLE_TYPE_DHT11)
#define SENSOR_TYPE DHT_TYPE_DHT11
#endif
#if defined(CONFIG_EXAMPLE_TYPE_AM2301)
#define SENSOR_TYPE DHT_TYPE_AM2301
#endif
#if defined(CONFIG_EXAMPLE_TYPE_SI7021)
#define SENSOR_TYPE DHT_TYPE_SI7021
#endif

#define I2C_PORT 0
#define ADDR_BH1750 BH1750_ADDR_LO // Endereço I2C padrão do sensor BH1750
#define ADDR_BMP280 BMP280_I2C_ADDRESS_0 // Endereço I2C padrão do sensor BMP280
#define ADDR_BME680 BME680_I2C_ADDR_1

#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  // Multisampling

// static const adc_atten_t atten = ADC_ATTEN_DB_11; // Valor de atenuação
// padrão static esp_adc_cal_characteristics_t *adc_chars;  // Estrutura de
// calibragem de ADC

void app_main(void) {
  SSD1306_t dev_d;
#ifdef CONFIG_IDF_TARGET_ESP32 // Se o target for ESP32
  i2c_master_init(&dev_d, 4, 15, 16);
#elif CONFIG_IDF_TARGET_ESP32S2 // Se o target for ESP32S2
  i2c_master_init(&dev_d, 21, 26, -1);
#endif

  ssd1306_init(&dev_d, 128, 64); // Inicializa o display

  ssd1306_clear_screen(&dev_d, false); // Limpa o display

  ssd1306_display_text(&dev_d, 0, "Hello Word:", 14, 0);

  ESP_ERROR_CHECK(i2cdev_init()); // Init library

  bme680_t sensor;
  memset(&sensor, 0, sizeof(bme680_t));

  i2c_dev_t dev;
  memset(&dev, 0, sizeof(i2c_dev_t)); // Zero descriptor

  ESP_ERROR_CHECK(
      bh1750_init_desc(&dev, ADDR_BH1750, I2C_PORT, SDA_GPIO, SCL_GPIO));
  ESP_ERROR_CHECK(bh1750_setup(&dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH));
  ESP_ERROR_CHECK(
      bme680_init_desc(&sensor, ADDR_BME680, I2C_PORT, SDA_GPIO, SCL_GPIO));

  // init the sensor
  ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

  // Changes the oversampling rates to 4x oversampling for temperature
  // and 2x oversampling for humidity. Pressure measurement is skipped.
  bme680_set_oversampling_rates(
      &sensor, BME680_OSR_4X, BME680_OSR_2X,
      BME680_OSR_2X); // BME680_OSR_4X, BME680_OSR_NONE, BME680_OSR_2X

  // Change the IIR filter size for temperature and pressure to 7.
  bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);

  // Change the heater profile 0 to 200 degree Celsius for 100 ms.
  bme680_set_heater_profile(&sensor, 0, 200, 100);
  bme680_use_heater_profile(&sensor, 0);

  // Set ambient temperature to 10 degree Celsius
  bme680_set_ambient_temperature(&sensor, 10);

  // as long as sensor configuration isn't changed, duration is constant
  uint32_t duration;
  bme680_get_measurement_duration(&sensor, &duration);

  TickType_t last_wakeup = xTaskGetTickCount();

  bme680_values_float_t values;

  gpio_set_pull_mode(3, GPIO_PULLUP_ONLY);

  while (1) {
    uint16_t lux;
    float temperature, humidity;

    if (dht_read_float_data(DHT_TYPE_AM2301, 3, &humidity, &temperature) !=
        ESP_OK)
      printf("Could not read data from sensor\n");

    if (bh1750_read(&dev, &lux) != ESP_OK)
      printf("Could not read lux data\n");

    // trigger the sensor to start one TPHG measurement cycle
    if (bme680_force_measurement(&sensor) == ESP_OK) {
      // passive waiting until measurement results are available
      vTaskDelay(duration);

      // get the results and do something with them
      if (bme680_get_results_float(&sensor, &values) == ESP_OK)
        printf("BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm, Lux: %d, "
               "DHT22 %.2lf °C, %.2f%% \n",
               values.temperature, values.humidity, values.pressure,
               values.gas_resistance, lux, temperature, humidity);
    }
    // passive waiting until 1 second is over
    vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(2000));
  }
}
