#include "apps/esp_sntp.h"
#include "esp32m/defs.hpp"

#include "esp32m/dev/rtc.hpp"
#include "esp32m/net/sntp.hpp"
/**
 * Use this library as template:
 * https://github.com/UncleRus/esp-idf-lib/tree/master/examples/ds3231/default
 */

namespace esp32m {

  namespace rtc {

    uint8_t Settings::getInt() const {
      return _int;
    }
    void Settings::setInt(uint8_t value) {
      if (_int == value)
        return;
      _int = value;
    }

    Core::Core(I2C *i2c, const char *name) : _i2c(i2c) {
      _name = name;
      // _i2c->setEndianness(Endian::Little);
      // _i2c->setErrSnooze(30000);
      // uint8_t id = 0;
      // if (_i2c->readSafe(Register::Id, id) == ESP_OK)
      //   _id = (ChipId)id;
      // if (!_name)
      //   switch (_id) {
      //     case ChipId::Bmp280:
      //       _name = "BMP280";
      //       break;
      //     case ChipId::Bme280:
      //       _name = "BME280";
      //       break;
      //     default:
      //       _name = "BMx280";
      //       break;
      //   }
    }

    // TODO: Find a better way to organize this
    static uint8_t dec2bcd(uint8_t val) {
      return ((val / 10) << 4) + (val % 10);
    }

    // TODO: Find a better way to organize this
    static uint8_t bcd2dec(uint8_t val) {
      return (val >> 4) * 10 + (val & 0x0f);
    }

    // TODO: Find a better way to organize this
    // Function to convert year, month, and day to days since January 1st
    static inline int days_since_january_1st(int year, int month, int day) {
      int days = day - 1;
      const int *ptr = days_per_month;

      // Handle leap year
      if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
        ptr = days_per_month_leap_year;

      // Add days from previous months
      for (int i = 0; i < month; i++) {
        days += ptr[i];
      }

      return days;
    }

    esp_err_t Core::setTime(struct tm *time) {
      // if (!_inited)
      //   return ESP_ERR_INVALID_STATE;
      uint8_t data[7];

      /* time/date data */
      data[0] = rtc::dec2bcd(time->tm_sec);
      data[1] = rtc::dec2bcd(time->tm_min);
      data[2] = rtc::dec2bcd(time->tm_hour);
      /* The week data must be in the range 1 to 7, and to keep the start on the
       * same day as for tm_wday have it start at 1 on Sunday. */
      data[3] = rtc::dec2bcd(time->tm_wday + 1);
      data[4] = rtc::dec2bcd(time->tm_mday);
      data[5] = rtc::dec2bcd(time->tm_mon + 1);
      data[6] = rtc::dec2bcd(time->tm_year - 100);

      ESP_CHECK_LOGW_RETURN(_i2c->write(Register::DS3231_ADDR_TIME, data, 7),
                            "failed to set time");

      return ESP_OK;
    }

    esp_err_t Core::getTime(struct tm *time) {
      // if (!_inited)
      //   return ESP_ERR_INVALID_STATE;
      uint8_t data[7];

      ESP_CHECK_LOGW_RETURN(_i2c->read(Register::DS3231_ADDR_TIME, data, 7),
                            "failed to get time");

      /* convert to unix time structure */
      time->tm_sec = rtc::bcd2dec(data[0]);
      time->tm_min = rtc::bcd2dec(data[1]);
      if (data[2] & DS3231_12HOUR_FLAG) {
        /* 12H */
        time->tm_hour = rtc::bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
        /* AM/PM? */
        if (data[2] & DS3231_PM_FLAG)
          time->tm_hour += 12;
      } else
        time->tm_hour = rtc::bcd2dec(data[2]); /* 24H */
      time->tm_wday = rtc::bcd2dec(data[3]) - 1;
      time->tm_mday = rtc::bcd2dec(data[4]);
      time->tm_mon = rtc::bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
      time->tm_year = rtc::bcd2dec(data[6]) + 100;
      time->tm_isdst = 0;
      time->tm_yday = rtc::days_since_january_1st(time->tm_year, time->tm_mon,
                                                  time->tm_mday);

      // apply a time zone (if you are not using localtime on the rtc or you
      // want to check/apply DST) applyTZ(time);

      // Log time
      // char buf[32];
      // strftime(buf, sizeof(buf), "%F %T", time);

      return ESP_OK;
    }

    esp_err_t Core::ds3231_set_alarm(Alarm alarms, struct tm *time1,
                                     Alarm1Rate option1, struct tm *time2,
                                     Alarm2Rate option2) {
      int i = 0;
      uint8_t data[7];

      /* alarm 1 data */
      if (alarms != DS3231_ALARM_2) {
        data[i++] =
            (option1 >= DS3231_ALARM1_MATCH_SEC ? rtc::dec2bcd(time1->tm_sec)
                                                : DS3231_ALARM_NOTSET);
        data[i++] =
            (option1 >= DS3231_ALARM1_MATCH_SECMIN ? rtc::dec2bcd(time1->tm_min)
                                                   : DS3231_ALARM_NOTSET);
        data[i++] = (option1 >= DS3231_ALARM1_MATCH_SECMINHOUR
                         ? rtc::dec2bcd(time1->tm_hour)
                         : DS3231_ALARM_NOTSET);
        data[i++] =
            (option1 == DS3231_ALARM1_MATCH_SECMINHOURDAY
                 ? (rtc::dec2bcd(time1->tm_wday + 1) & DS3231_ALARM_WDAY)
                 : (option1 == DS3231_ALARM1_MATCH_SECMINHOURDATE
                        ? rtc::dec2bcd(time1->tm_mday)
                        : DS3231_ALARM_NOTSET));
      }

      /* alarm 2 data */
      if (alarms != DS3231_ALARM_1) {
        data[i++] =
            (option2 >= DS3231_ALARM2_MATCH_MIN ? rtc::dec2bcd(time2->tm_min)
                                                : DS3231_ALARM_NOTSET);
        data[i++] = (option2 >= DS3231_ALARM2_MATCH_MINHOUR
                         ? rtc::dec2bcd(time2->tm_hour)
                         : DS3231_ALARM_NOTSET);
        data[i++] =
            (option2 == DS3231_ALARM2_MATCH_MINHOURDAY
                 ? (rtc::dec2bcd(time2->tm_wday + 1) & DS3231_ALARM_WDAY)
                 : (option2 == DS3231_ALARM2_MATCH_MINHOURDATE
                        ? rtc::dec2bcd(time2->tm_mday)
                        : DS3231_ALARM_NOTSET));
      }

      ESP_CHECK_LOGW_RETURN(_i2c->write((alarms == Alarm::DS3231_ALARM_2
                                             ? Register::DS3231_ADDR_ALARM2
                                             : Register::DS3231_ADDR_ALARM1),
                                        data, i),
                            "failed to set time");

      return ESP_OK;
    }

    esp_err_t Core::ds3231_get_alarm_flags(Alarm *alarms) {
      uint8_t data;

      /* get register */
      /* get status register */
      ESP_CHECK_LOGW_RETURN(_i2c->read(DS3231_ADDR_STATUS, &data, 1),
                            "failed to get alarm flags");

      /* return only requested flag */
      *alarms = (Alarm)(data & DS3231_ALARM_BOTH);
      return ESP_OK;
    }

    esp_err_t Core::ds3231_clear_alarm_flags(Alarm alarms) {
      ds3231_set_flag(DS3231_ADDR_STATUS, alarms, DS3231_CLEAR);

      return ESP_OK;
    }

    esp_err_t Core::ds3231_enable_alarm_ints(Alarm alarms) {
      ds3231_set_flag(DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM_INTS | alarms,
                      DS3231_SET);

      return ESP_OK;
    }

    esp_err_t Core::ds3231_disable_alarm_ints(Alarm alarms) {
      ds3231_set_flag(DS3231_ADDR_CONTROL, alarms, DS3231_CLEAR);

      return ESP_OK;
    }

    esp_err_t Core::ds3231_get_raw_temp(int16_t *temp) {
      uint8_t data[2];

      ESP_CHECK_LOGW_RETURN(_i2c->read(DS3231_ADDR_TEMP, &data, sizeof(data)),
                            "failed to get raw temp");

      *temp = (int16_t)(int8_t)data[0] << 2 | data[1] >> 6;

      return ESP_OK;
    }

    esp_err_t Core::ds3231_get_temp_integer(int8_t *temp) {
      int16_t t_int;

      esp_err_t res = ds3231_get_raw_temp(&t_int);
      if (res == ESP_OK)
        *temp = t_int >> 2;

      return res;
    }

    esp_err_t Core::ds3231_get_temp_float(float *temp) {
      int16_t t_int;

      esp_err_t res = ds3231_get_raw_temp(&t_int);
      if (res == ESP_OK)
        *temp = t_int * 0.25;

      return res;
    }

    esp_err_t Core::ds3231_set_flag(Register addr, uint8_t bits,
                                    AlarmFlag mode) {
      uint8_t data;

      /* get status register */
      ESP_CHECK_LOGW_RETURN(_i2c->read(addr, &data, 1), "failed to set flag");

      /* clear the flag */
      if (mode == DS3231_REPLACE)
        data = bits;
      else if (mode == DS3231_SET)
        data |= bits;
      else
        data &= ~bits;

      ESP_CHECK_LOGW_RETURN(_i2c->write(addr, &data, 1), "failed to set flag");

      return ESP_OK;
    }

  }  // namespace rtc

  namespace dev {

    Rtc::Rtc(I2C *i2c, const char *name)
        : rtc::Core(i2c, name),
          _currentTime(this, "currentTime"),
          _alarm1(this, "alarm1"),
          _alarm2(this, "alarm2"),
          _temperature(this, "temperature") {
      auto group = sensor::nextGroup();
      _currentTime.group = group;
      _currentTime.precision = 2;
      _alarm1.group = group;
      _alarm1.precision = 0;
      _alarm1.unit = "mmHg";
      _alarm2.group = group;
      _alarm2.precision = 0;
      _temperature.group = group;
      _temperature.precision = 0;
      Device::init(Flags::HasSensors);

      // Check if we have time in RTC
      // get time and print it
      struct tm timeinfo;
      if (getTime(&timeinfo) == ESP_OK) {
        auto timer = mktime(&timeinfo);
        struct timeval tv;
        tv.tv_sec = timer;
        tv.tv_usec = 0;
        settimeofday(&tv, nullptr);
      };
    }

    bool Rtc::initSensors() {
      return true;  // sync(true) == ESP_OK;
    }

    void Rtc::handleEvent(Event &ev) {
      if (EventStateChanged::is(ev, &net::Sntp::instance())) {
        if (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {
          struct tm timeinfo;
          time_t now;
          time(&now);
          localtime_r(&now, &timeinfo);
          setTime(&timeinfo);
        }
      }
    }

    bool Rtc::pollSensors() {
      float t = 0, p = 0, h = 0, l = 0;
      // ESP_CHECK_RETURN_BOOL(read(&t, &p, &h));
      bool changed = false;
      _temperature.set(t, &changed);
      _alarm1.set(p, &changed);
      // if (chipId() == bme280::ChipId::Bme280)
      _alarm2.set(h, &changed);
      // get time and print it
      struct tm timeinfo;
      getTime(&timeinfo);
      char buf[32];
      strftime(buf, sizeof(buf), "%F %T", &timeinfo);
      _currentTime.set(buf, &changed);
      logI("local time/date is %s", buf);
      if (changed)
        sensor::GroupChanged::publish(_currentTime.group);
      return true;
    }

    DynamicJsonDocument *Rtc::getState(const JsonVariantConst args) {
      float t = 0, p = 0, h = 0, l = 0;
      // if (read(&t, &p, &h) != ESP_OK)
      // return nullptr;
      DynamicJsonDocument *doc = new DynamicJsonDocument(JSON_OBJECT_SIZE(7));
      JsonObject root = doc->to<JsonObject>();
      root["addr"] = _i2c->addr();
      int8_t temp;
      ds3231_get_temp_integer(&temp);
      root["temperature"] = temp;
      root["alarm1"] = p;
      // if (chipId() == bme280::ChipId::Bme280)
      root["alarm2"] = h;
      // get time and print it
      struct tm timeinfo;
      getTime(&timeinfo);
      char buf[32];
      strftime(buf, sizeof(buf), "%F %T", &timeinfo);
      logI("local time/date is %s", buf);
      root["currentTime"] = buf;
      return doc;
    }

    void useRtc(const char *name, uint8_t addr) {
      new Rtc(new I2C(addr), name);
    }

  }  // namespace dev

}  // namespace esp32m