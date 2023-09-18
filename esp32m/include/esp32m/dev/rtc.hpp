#pragma once

#include "esp32m/bus/i2c.hpp"
#include "esp32m/device.hpp"

namespace esp32m {
  class Rtc;

  namespace rtc {

    const uint8_t DefaultAddress = 0x68;

    const int days_per_month[] = {31, 28, 31, 30, 31, 30,
                                  31, 31, 30, 31, 30, 31};
    const int days_per_month_leap_year[] = {31, 29, 31, 30, 31, 30,
                                            31, 31, 30, 31, 30, 31};

    /**
     * Alarms
     */
    enum AlarmFlag { DS3231_SET = 0, DS3231_CLEAR, DS3231_REPLACE };

    /**
     * Alarms
     */
    enum Alarm {
      DS3231_ALARM_NONE = 0,  //!< No alarms
      DS3231_ALARM_1,         //!< First alarm
      DS3231_ALARM_2,         //!< Second alarm
      DS3231_ALARM_BOTH       //!< Both alarms
    };

    /**
     * First alarm rate
     */
    enum Alarm1Rate {
      DS3231_ALARM1_EVERY_SECOND = 0,
      DS3231_ALARM1_MATCH_SEC,
      DS3231_ALARM1_MATCH_SECMIN,
      DS3231_ALARM1_MATCH_SECMINHOUR,
      DS3231_ALARM1_MATCH_SECMINHOURDAY,
      DS3231_ALARM1_MATCH_SECMINHOURDATE
    };

    /**
     * Second alarm rate
     */
    enum Alarm2Rate {
      DS3231_ALARM2_EVERY_MIN = 0,
      DS3231_ALARM2_MATCH_MIN,
      DS3231_ALARM2_MATCH_MINHOUR,
      DS3231_ALARM2_MATCH_MINHOURDAY,
      DS3231_ALARM2_MATCH_MINHOURDATE
    };

    /**
     * Squarewave frequency
     */
    enum SquarewaveFrequency {
      DS3231_SQWAVE_1HZ = 0x00,
      DS3231_SQWAVE_1024HZ = 0x08,
      DS3231_SQWAVE_4096HZ = 0x10,
      DS3231_SQWAVE_8192HZ = 0x18
    };

    enum Register {
      DS3231_STAT_OSCILLATOR = 0x80,
      DS3231_STAT_32KHZ = 0x08,
      DS3231_STAT_ALARM_2 = 0x02,
      DS3231_STAT_ALARM_1 = 0x01,
      DS3231_CTRL_OSCILLATOR = 0x80,
      DS3231_CTRL_TEMPCONV = 0x20,
      DS3231_CTRL_ALARM_INTS = 0x04,
      DS3231_CTRL_ALARM2_INT = 0x02,
      DS3231_CTRL_ALARM1_INT = 0x01,
      DS3231_ALARM_WDAY = 0x40,
      DS3231_ALARM_NOTSET = 0x80,
      DS3231_ADDR_TIME = 0x00,
      DS3231_ADDR_ALARM1 = 0x07,
      DS3231_ADDR_ALARM2 = 0x0b,
      DS3231_ADDR_CONTROL = 0x0e,
      DS3231_ADDR_STATUS = 0x0f,
      DS3231_ADDR_AGING = 0x10,
      DS3231_ADDR_TEMP = 0x11,
      DS3231_12HOUR_FLAG = 0x40,
      DS3231_12HOUR_MASK = 0x1f,
      DS3231_PM_FLAG = 0x20,
      DS3231_MONTH_MASK = 0x1f
    };

    struct Settings {
     public:
      uint8_t getInt() const;
      void setInt(uint8_t value);

     private:
      uint8_t _int = 0;
      friend class Core;
    };

    class Core : public virtual log::Loggable {
     public:
      Core(I2C *i2c, const char *name);
      Core(const Core &) = delete;
      const char *name() const override {
        return _name;
      }
      Settings &settings() {
        return _settings;
      }

      /**
       * @brief Set the time on the RTC
       *
       * Timezone agnostic, pass whatever you like.
       * I suggest using GMT and applying timezone and DST when read back.
       *
       * @return ESP_OK to indicate success
       */
      esp_err_t setTime(struct tm *tv);

      /**
       * @brief Get the time from the RTC, populates a supplied tm struct
       *
       * @param dev Device descriptor
       * @param[out] time RTC time
       * @return ESP_OK to indicate success
       */
      esp_err_t getTime(struct tm *tv);

      /**
       * @brief Set alarms
       *
       * `alarm1` works with seconds, minutes, hours and day of week/month, or
       * fires every second. `alarm2` works with minutes, hours and day of
       * week/month, or fires every minute.
       *
       * Not all combinations are supported, see `DS3231_ALARM1_*` and
       * `DS3231_ALARM2_*` defines for valid options you only need to populate
       * the fields you are using in the `tm` struct, and you can set both
       * alarms at the same time (pass
       * `DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`).
       *
       * If only setting one alarm just pass 0 for `tm` struct and `option`
       * field for the other alarm. If using
       * ::DS3231_ALARM1_EVERY_SECOND/::DS3231_ALARM2_EVERY_MIN you can pass 0
       * for `tm` struct.
       *
       * If you want to enable interrupts for the alarms you need to do that
       * separately.
       *
       * @return ESP_OK to indicate success
       */
      esp_err_t ds3231_set_alarm(Alarm alarms, struct tm *time1,
                                 Alarm1Rate option1, struct tm *time2,
                                 Alarm2Rate option2);

      /**
       * @brief Check which alarm(s) have past
       *
       * Sets alarms to
       * `DS3231_ALARM_NONE`/`DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`
       *
       * @param dev Device descriptor
       * @param[out] alarms Alarms
       * @return ESP_OK to indicate success
       */
      esp_err_t ds3231_get_alarm_flags(Alarm *alarms);

      /**
       * @brief Clear alarm past flag(s)
       *
       * Pass `DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`
       *
       * @param dev Device descriptor
       * @param alarms Alarms
       * @return ESP_OK to indicate success
       */
      esp_err_t ds3231_clear_alarm_flags(Alarm alarms);

      /**
       * @brief enable alarm interrupts (and disables squarewave)
       *
       * Pass `DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`.
       *
       * If you set only one alarm the status of the other is not changed
       * you must also clear any alarm past flag(s) for alarms with
       * interrupt enabled, else it will trigger immediately.
       *
       * @param dev Device descriptor
       * @param alarms Alarms
       * @return ESP_OK to indicate success
       */
      esp_err_t ds3231_enable_alarm_ints(Alarm alarms);

      /**
       * @brief Disable alarm interrupts
       *
       * Does not (re-)enable squarewave
       *
       * @param dev Device descriptor
       * @param alarms Alarm
       * @return ESP_OK to indicate success
       */
      esp_err_t ds3231_disable_alarm_ints(Alarm alarms);

      /**
       * @brief Get the raw temperature value
       *
       * **Supported only by DS3231**
       *
       * @param dev Device descriptor
       * @param[out] temp Raw temperature value
       * @return ESP_OK to indicate success
       */
      esp_err_t ds3231_get_raw_temp(int16_t *temp);

      /**
       * @brief Get the temperature as an integer
       *
       * **Supported only by DS3231**
       *
       * @param dev Device descriptor
       * @param[out] temp Temperature, degrees Celsius
       * @return ESP_OK to indicate success
       */
      esp_err_t ds3231_get_temp_integer(int8_t *temp);

      /**
       * @brief Get the temperature as a float
       *
       * **Supported only by DS3231**
       *
       * @param dev Device descriptor
       * @param[out] temp Temperature, degrees Celsius
       * @return ESP_OK to indicate success
       */
      esp_err_t ds3231_get_temp_float(float *temp);

     protected:
      std::unique_ptr<I2C> _i2c;

     private:
      const char *_name;
      bool _inited = false;
      Settings _settings;
      friend class esp32m::Rtc;

      /* Set/clear bits in a byte register, or replace the byte altogether
       * pass the register address to modify, a byte to replace the existing
       * value with or containing the bits to set/clear and one of
       * DS3231_SET/DS3231_CLEAR/DS3231_REPLACE
       * returns true to indicate success
       */
      esp_err_t ds3231_set_flag(Register addr, uint8_t bits, AlarmFlag mode);
    };
  }  // namespace rtc

  namespace dev {
    class Rtc : public virtual Device, public virtual rtc::Core {
     public:
      Rtc(I2C *i2c, const char *name);
      Rtc(const Rtc &) = delete;

     protected:
      void handleEvent(Event &ev) override;
      DynamicJsonDocument *getState(const JsonVariantConst args) override;
      bool initSensors() override;
      bool pollSensors() override;

     private:
      Sensor _currentTime, _alarm1, _alarm2, _temperature;
    };

    void useRtc(const char *name = nullptr, uint8_t addr = rtc::DefaultAddress);
  }  // namespace dev
}  // namespace esp32m