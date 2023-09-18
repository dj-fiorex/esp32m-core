#include "esp32m/device.hpp"
#include "esp32m/app.hpp"
#include "esp32m/base.hpp"
#include "esp32m/net/ota.hpp"
#include "esp32m/sleep.hpp"

#include <esp_task_wdt.h>
#include <math.h>

namespace esp32m {
  const char *EventSensor::NAME = "sensor";

  class EventPollSensors : public Event {
   public:
    EventPollSensors() : Event(Type) {}
    static bool is(Event &ev) {
      return ev.is(Type);
    }

   private:
    constexpr static const char *Type = "poll-sensors";
  };

  class EventPollSensorsTime : public Event {
   public:
    EventPollSensorsTime() : Event(Type) {}
    static bool is(Event &ev, EventPollSensorsTime **pste) {
      bool result = ev.is(Type);
      if (result && pste)
        *pste = (EventPollSensorsTime *)&ev;
      return result;
    }
    void record(unsigned long next) {
      if (next < _next)
        _next = next;
    }
    unsigned long getNext() {
      return _next;
    }

   private:
    unsigned long _next = 0;
    constexpr static const char *Type = "poll-sensors-time";
  };

  void Device::init(Flags flags) {
    _flags = flags;
    if (flags & Flags::HasSensors)
      setupSensorPollTask();
  }

  void Device::handleEvent(Event &ev) {
    if ((_flags & Flags::HasSensors) == 0)
      return;
    EventPollSensorsTime *pste;
    if (EventPollSensorsTime::is(ev, &pste))
      pste->record(nextSensorsPollTime());
    else if (EventPollSensors::is(ev) && shouldPollSensors()) {
      if (sensorsReady() && !pollSensors())
        resetSensors();
      _sensorsPolledAt = millis();
    }
  }

  void Device::sensor(const char *sensor, const float value) {
    if (!isnan(value))
      EventSensor::publish(*this, sensor, value, json::null<JsonObjectConst>());
  };

  void Device::sensor(const char *sensor, const float value,
                      const JsonObjectConst props) {
    if (!isnan(value))
      EventSensor::publish(*this, sensor, value, props);
  };

  void Device::setupSensorPollTask() {
    static TaskHandle_t task = nullptr;
    // static Sleeper sleeper;
    static EventPollSensors ev;
    if (task)
      xTaskNotifyGive(task);
    else
      xTaskCreate(
          [](void *) {
            EventManager::instance().subscribe([](Event &ev) {
              if (EventInited::is(ev) && task)
                xTaskNotifyGive(task);
            });
            esp_task_wdt_add(NULL);
            for (;;) {
              int sleepTime = 0;
              esp_task_wdt_reset();
              if (App::initialized()) {
                EventPollSensorsTime pste;
                pste.publish();
                auto next = pste.getNext();
                auto current = millis();
                if (next < current) {
                  locks::Guard guard(net::ota::Name);
                  ev.publish();
                } else
                  sleepTime = current - next;  // sleeper.sleep();
              } else
                sleepTime = 1000;
              if (sleepTime > 0)
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(sleepTime));
            }
          },
          "m/sensors", 4096, nullptr, 1, &task);
  }

  bool Device::sensorsReady() {
    if (_sensorsReady)
      return true;
    unsigned int ms = millis();
    if (_sensorsInitAt && ms - _sensorsInitAt < _reinitDelay)
      return false;
    _sensorsInitAt = ms;
    _sensorsReady = initSensors();
    if (_sensorsReady)
      logI("device initialized");
    return _sensorsReady;
  }

  namespace sensor {

    std::mutex _sensorsMutex;
    std::map<std::string, Sensor *> _sensors;
    int _groupCounter = 0;

    Sensor *find(std::string uid) {
      std::lock_guard lock(_sensorsMutex);
      auto it = _sensors.find(uid);
      return it == _sensors.end() ? nullptr : it->second;
    }

    Sensor *find(Device *device, const char *id) {
      auto uid = string_printf("%s_%s", device->name(), id);
      return find(uid);
    }

    int nextGroup() {
      std::lock_guard lock(_sensorsMutex);
      return ++_groupCounter;
    }

    Group::Iterator::Iterator(int id) : _id(id) {
      if (id < 0)
        _inner = _sensors.end();
      else {
        _inner = _sensors.begin();
        if (_inner->second->group != _id)
          next();
      }
    }
    void Group::Iterator::next() {
      for (;;) {
        _inner++;
        if (_inner == _sensors.end() || _inner->second->group == _id)
          break;
      }
    }

    All::Iterator All::begin() const {
      return All::Iterator(_sensors.begin());
    }
    All::Iterator All::end() const {
      return All::Iterator(_sensors.end());
    }

  }  // namespace sensor

  Sensor::Sensor(Device *device, const char *type, const char *id, size_t size)
      : _device(device), _type(type), _value(size) {
    std::lock_guard lock(sensor::_sensorsMutex);
    if (id)
      _id = id;
    sensor::_sensors[uid()] = this;
  }

}  // namespace esp32m
