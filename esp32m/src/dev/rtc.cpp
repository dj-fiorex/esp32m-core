#include "esp32m/defs.hpp"

#include "esp32m/dev/rtc.hpp"
#include "esp32m/ha/ha.hpp"
/**
 * Use this library as template: https://github.com/UncleRus/esp-idf-lib/tree/master/examples/ds3231/default
*/

namespace esp32m
{

  namespace rtc
  {

    uint8_t Settings::getInt() const
    {
      return _int;
    }
    void Settings::setInt(uint8_t value)
    {
      if (_int == value)
        return;
      _int = value;
    }

    Core::Core(I2C *i2c, const char *name) : _i2c(i2c)
    {
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

    uint8_t dec2bcd(uint8_t val)
    {
      return ((val / 10) << 4) + (val % 10);
    }

    esp_err_t Core::setTime(struct timeval *tv)
    {
      // if (!_inited)
      //   return ESP_ERR_INVALID_STATE;
      uint8_t data[7];

      /* time/date data */
      data[0] = dec2bcd(tv->tv_sec % 60);                // seconds
      data[1] = dec2bcd(tv->tv_sec / 60 % 60);           // minutes
      data[2] = dec2bcd(tv->tv_sec / 3600 % 24);         // hours
      data[3] = dec2bcd(tv->tv_sec / 86400 % 7);         // week day
      data[4] = dec2bcd(tv->tv_sec / 86400 / 7);         // month day
      data[5] = dec2bcd(tv->tv_sec / 86400 / 365);       // month
      data[6] = dec2bcd(tv->tv_sec / 86400 / 365 / 100); // year

      ESP_CHECK_LOGW_RETURN(_i2c->write(Register::DS3231_ADDR_TIME, data, 7),
                            "failed to set time");

      return ESP_OK;
    }

  } // namespace rtc

  namespace dev
  {

    Rtc::Rtc(I2C *i2c, const char *name)
        : rtc::Core(i2c, name),
          _currentTime(this, "currentTime"),
          _alarm1(this, "alarm1"),
          _alarm2(this, "alarm2"),
          _temperature(this, "temperature")
    {
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
    }

    bool Rtc::initSensors()
    {
      return true; // sync(true) == ESP_OK;
    }

    void Rtc::handleEvent(Event &ev)
    {
      EventStateChanged *stc;
      if (EventStateChanged::is(ev, &stc)) {
          auto state = stc->state();
          auto obj = stc->object();
          if (!state.isUnbound() && obj)
            publish(obj->name(), state, false);
        }
      // if (IpEvent::is(ev, IP_EVENT_STA_GOT_IP, nullptr))
      //   xTimerPendFunctionCall(
      //       [](void *self, uint32_t a)
      //       { ((Sntp *)self)->update(); },
      //       this, 0,
      //       pdMS_TO_TICKS(1000));
    }

    bool Rtc::pollSensors()
    {
      float t = 0, p = 0, h = 0, l = 0;
      // ESP_CHECK_RETURN_BOOL(read(&t, &p, &h));
      sensor("temperature", t);
      sensor("alarm1", p);
      sensor("alarm2", h);
      // if (chipId() == bme280::ChipId::Bme280)
      sensor("currentTime", l);

      bool changed = false;
      _temperature.set(t, &changed);
      _alarm1.set(p, &changed);
      // if (chipId() == bme280::ChipId::Bme280)
      _alarm2.set(h, &changed);
      _currentTime.set(l, &changed);
      if (changed)
        sensor::GroupChanged::publish(_currentTime.group);
      return true;
    }

    DynamicJsonDocument *Rtc::getState(const JsonVariantConst args)
    {
      float t = 0, p = 0, h = 0, l = 0;
      // if (read(&t, &p, &h) != ESP_OK)
      // return nullptr;
      DynamicJsonDocument *doc = new DynamicJsonDocument(JSON_OBJECT_SIZE(5));
      JsonObject root = doc->to<JsonObject>();
      root["addr"] = _i2c->addr();
      root["temperature"] = t;
      root["alarm1"] = p;
      // if (chipId() == bme280::ChipId::Bme280)
      root["alarm2"] = h;
      root["currentTime"] = l;
      return doc;
    }

    void useRtc(const char *name, uint8_t addr)
    {
      new Rtc(new I2C(addr), name);
    }

  } // namespace dev

} // namespace esp32m