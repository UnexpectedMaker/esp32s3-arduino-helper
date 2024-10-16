// Updated for ESP32 Arduino Core 3.0.0 support
// Oct 16, 2024 - Seon Rozenblum (Unexpected Maker)

#ifndef _UMS3_H
#define _UMS3_H

#include <Arduino.h>
#include <esp_adc_cal.h>
#include <soc/adc_channel.h>

#if defined(ARDUINO_TINYS3) || defined(ARDUINO_PROS3) || defined(ARDUINO_NANOS3)
#define VBAT_ADC_CHANNEL ADC1_GPIO10_CHANNEL
#define VBAT_ADC_PIN 10
#elif defined(ARDUINO_FEATHERS3)
#define VBAT_ADC_CHANNEL ADC1_GPIO2_CHANNEL
#define ALS_ADC_CHANNEL ADC1_GPIO4_CHANNEL
#define VBAT_ADC_PIN 2
#define ALS_ADC_PIN 4
#else
#error                                                                         \
    "The board you have selected is not compatible with the UMS3 helper library"
#endif

class UMS3 {
public:
  UMS3() : brightness(255) {}

  void begin() {
    // RGB_PWR is LDO2 on boards that have it
    pinMode(RGB_PWR, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR < 3
    rmt = rmtInit(RGB_DATA, RMT_TX_MODE, RMT_MEM_64);
    rmtSetTick(rmt, 25);
#else
    rmtInit(RGB_DATA, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000);
#endif

#if defined(ARDUINO_FEATHERS3)
    pinMode(LED_BUILTIN, INPUT | OUTPUT);
#endif

#if defined(ARDUINO_FEATHERS3) || defined(ARDUINO_TINYS3) ||                   \
    defined(ARDUINO_PROS3)
#if ESP_ARDUINO_VERSION_MAJOR < 3
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 0,
                             &adc_cal);
    adc1_config_channel_atten(VBAT_ADC_CHANNEL, ADC_ATTEN_DB_2_5);
#else
    analogSetPinAttenuation(VBAT_ADC_PIN, ADC_2_5db);
#endif
#endif

#if defined(ARDUINO_FEATHERS3)
#if ESP_ARDUINO_VERSION_MAJOR < 3
    adc1_config_channel_atten(ALS_ADC_CHANNEL, ADC_ATTEN_DB_11);
#else
    analogSetPinAttenuation(ALS_ADC_PIN, ADC_11db);
#endif
#endif

#if defined(ARDUINO_FEATHERS3) || defined(ARDUINO_TINYS3) ||                   \
    defined(ARDUINO_PROS3)
    pinMode(VBUS_SENSE, INPUT);
#endif
  }

#if defined(ARDUINO_PROS3) || defined(ARDUINO_FEATHERS3)
  void setLDO2Power(bool on) { digitalWrite(LDO2, on); }
#endif

  void setPixelPower(bool on) { digitalWrite(RGB_PWR, on); }

  void setPixelColor(uint8_t r, uint8_t g, uint8_t b) {
    pixel_color[0] = g;
    pixel_color[1] = r;
    pixel_color[2] = b;
    writePixel();
  }

  void setPixelColor(uint32_t rgb) { setPixelColor(rgb >> 16, rgb >> 8, rgb); }

  void setPixelBrightness(uint8_t brightness) {
    this->brightness = brightness;
    writePixel();
  }

  void writePixel() {
    setPixelPower(true);
    while (micros() - next_rmt_write < 350) {
      yield();
    }
    int index = 0;
    for (auto chan : pixel_color) {
      uint8_t value = chan * (brightness + 1) >> 8;
      for (int bit = 7; bit >= 0; bit--) {
        if ((value >> bit) & 1) {
#if ESP_ARDUINO_VERSION_MAJOR < 3
          rmt_data[index].level0 = 1;
          rmt_data[index].duration0 = 32; // 800ns
          rmt_data[index].level1 = 0;
          rmt_data[index].duration1 = 18; // 450ns
        } else {
          rmt_data[index].level0 = 1;
          rmt_data[index].duration0 = 16; // 400ns
          rmt_data[index].level1 = 0;
          rmt_data[index].duration1 = 34; // 850ns
#else
          rmt_data[index].level0 = 1;
          rmt_data[index].duration0 = 8;
          rmt_data[index].level1 = 0;
          rmt_data[index].duration1 = 4;
        } else {
          rmt_data[index].level0 = 1;
          rmt_data[index].duration0 = 4;
          rmt_data[index].level1 = 0;
          rmt_data[index].duration1 = 8;
#endif
        }
        index++;
      }
    }
#if ESP_ARDUINO_VERSION_MAJOR < 3
    rmtWrite(rmt, rmt_data, 3 * 8);
#else
    rmtWrite(RGB_DATA, rmt_data, 3 * 8, RMT_WAIT_FOR_EVER);
#endif
    next_rmt_write = micros();
  }

  static uint32_t color(uint8_t r, uint8_t g, uint8_t b) {
    return (r << 16) | (g << 8) | b;
  }

  static uint32_t colorWheel(uint8_t pos) {
    if (pos < 85) {
      return color(255 - pos * 3, pos * 3, 0);
    } else if (pos < 170) {
      pos -= 85;
      return color(0, 255 - pos * 3, pos * 3);
    } else {
      pos -= 170;
      return color(pos * 3, 0, 255 - pos * 3);
    }
  }

#if defined(ARDUINO_FEATHERS3)
  void setBlueLED(bool on) { digitalWrite(LED_BUILTIN, on); }

  void toggleBlueLED() { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); }
#endif

#if defined(ARDUINO_FEATHERS3) || defined(ARDUINO_TINYS3) ||                   \
    defined(ARDUINO_PROS3)
  float getBatteryVoltage() {
#if ESP_ARDUINO_VERSION_MAJOR < 3
    uint32_t raw = adc1_get_raw(VBAT_ADC_CHANNEL);
    uint32_t millivolts = esp_adc_cal_raw_to_voltage(raw, &adc_cal);
#else
    uint32_t millivolts = analogReadMilliVolts(VBAT_ADC_PIN);
#endif
    const uint32_t upper_divider = 442;
    const uint32_t lower_divider = 160;
    return (float)(upper_divider + lower_divider) / lower_divider / 1000 *
           millivolts;
  }

  bool getVbusPresent() { return digitalRead(VBUS_SENSE); }
#endif

#if defined(ARDUINO_FEATHERS3)
  float getLightSensorVoltage() {
#if ESP_ARDUINO_VERSION_MAJOR < 3
    uint32_t raw = adc1_get_raw(ALS_ADC_CHANNEL);
    uint32_t millivolts = esp_adc_cal_raw_to_voltage(raw, &adc_cal);
    return (float)millivolts / 1000.0f;
#else
    uint32_t millivolts = analogReadMilliVolts(ALS_ADC_PIN);
    return (float)(millivolts);
#endif
  }
#endif

private:
#if ESP_ARDUINO_VERSION_MAJOR < 3
  rmt_obj_t *rmt;
#endif

  rmt_data_t rmt_data[3 * 8];
  unsigned long next_rmt_write;
  uint8_t pixel_color[3];
  uint8_t brightness;
  esp_adc_cal_characteristics_t adc_cal;
};

#endif
