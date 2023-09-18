#pragma once

#include "esp32m/bus/i2c.hpp"
#include "esp32m/device.hpp"

namespace esp32m
{
  class Rtc;

  namespace rtc
  {

    const uint8_t DefaultAddress = 0x68;

    /**
     * Alarms
     */
    enum Alarm
    {
      DS3231_ALARM_NONE = 0, //!< No alarms
      DS3231_ALARM_1,        //!< First alarm
      DS3231_ALARM_2,        //!< Second alarm
      DS3231_ALARM_BOTH      //!< Both alarms
    };

    /**
     * First alarm rate
     */
    enum Alarm1Rate
    {
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
    enum Alarm2Rate
    {
      DS3231_ALARM2_EVERY_MIN = 0,
      DS3231_ALARM2_MATCH_MIN,
      DS3231_ALARM2_MATCH_MINHOUR,
      DS3231_ALARM2_MATCH_MINHOURDAY,
      DS3231_ALARM2_MATCH_MINHOURDATE
    };

    /**
     * Squarewave frequency
     */
    enum SquarewaveFrequency
    {
      DS3231_SQWAVE_1HZ = 0x00,
      DS3231_SQWAVE_1024HZ = 0x08,
      DS3231_SQWAVE_4096HZ = 0x10,
      DS3231_SQWAVE_8192HZ = 0x18
    };

    enum Register
    {
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

    struct Settings
    {
    public:
      uint8_t getInt() const;
      void setInt(uint8_t value);

    private:
      uint8_t _int = 0;
      friend class Core;
    };

    class Core : public virtual log::Loggable
    {
    public:
      Core(I2C *i2c, const char *name);
      Core(const Core &) = delete;
      const char *name() const override
      {
        return _name;
      }
      Settings &settings()
      {
        return _settings;
      }
      esp_err_t setTime(struct timeval *tv);

    protected:
      uint8_t dec2bcd(uint8_t val);
      std::unique_ptr<I2C> _i2c;

    private:
      const char *_name;
      bool _inited = false;
      Settings _settings;
      friend class esp32m::Rtc;
    };
  } // namespace rtc

  namespace dev
  {
    class Rtc : public virtual Device, public virtual rtc::Core
    {
    public:
      Rtc(I2C *i2c, const char *name);
      Rtc(const Rtc &) = delete;

    protected:
      DynamicJsonDocument *getState(const JsonVariantConst args) override;
      bool initSensors() override;
      bool pollSensors() override;

    private:
      Sensor _currentTime, _alarm1, _alarm2, _temperature;
    };

    void useRtc(const char *name = nullptr, uint8_t addr = rtc::DefaultAddress);
  } // namespace dev
} // namespace esp32m