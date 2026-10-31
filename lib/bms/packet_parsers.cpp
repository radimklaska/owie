#include <algorithm>
#include <cmath>
#include <cstdint>

#include "bms_relay.h"
#include "packet.h"

namespace {

inline int16_t int16FromNetworkOrder(const void* const p) {
  uint8_t* const charPointer = (uint8_t* const)p;
  return ((uint16_t)(*charPointer)) << 8 | *(charPointer + 1);
}

}  // namespace

void BmsRelay::batteryPercentageParser(Packet& p) {
  if (p.getType() != 3) {
    return;
  }
  // 0x3 message is just one byte containing battery percentage.
  bms_soc_percent_ = *(int8_t*)p.data();
  overridden_soc_percent_ = battery_fuel_gauge_.getSoc();
  if (overridden_soc_percent_ < 0) {
    p.setShouldForward(false);
    return;
  }
  p.data()[0] = overridden_soc_percent_;
}

void BmsRelay::currentParser(Packet& p) {
  if (p.getType() != 5) {
    return;
  }
  p.setShouldForward(isCharging());

  // 0x5 message encodes current as signed int16.
  // The scaling factor (tested on a Pint) seems to be 0.055
  // i.e. 1 in the data message below corresponds to 55 milliamps.
  current_milliamps_ = int16FromNetworkOrder(p.data()) * CURRENT_SCALER;
  battery_fuel_gauge_.updateCurrent(current_milliamps_, now_millis_);
}

void BmsRelay::bmsSerialParser(Packet& p) {
  if (p.getType() != 6) {
    return;
  }
  // 0x6 message has the BMS serial number encoded inside of it.
  // It is the last seven digits from the sticker on the back of the BMS.
  if (captured_serial_ == 0) {
    for (int i = 0; i < 4; i++) {
      captured_serial_ |= p.data()[i] << (8 * (3 - i));
    }
  }
  if (serial_override_ == 0) {
    return;
  }
  uint32_t serial_override_copy = serial_override_;
  for (int i = 3; i >= 0; i--) {
    p.data()[i] = serial_override_copy & 0xFF;
    serial_override_copy >>= 8;
  }
}

void BmsRelay::cellVoltageParser(Packet& p) {
  if (p.getType() != 2) {
    return;
  }
  // The data in this packet is 16 int16-s. First 15 of them is
  // individual cell voltages in millivolts. The last value is mysterious.
  const uint8_t* const data = p.data();
  uint16_t total_voltage = 0;
  uint16_t min_voltage = 0xFFFF;
  for (int i = 0; i < 15; i++) {
    uint16_t cellVoltage = int16FromNetworkOrder(data + (i << 1));
    total_voltage += cellVoltage;
    if (cellVoltage < min_voltage) {
      min_voltage = cellVoltage;
    }
    cell_millivolts_[i] = cellVoltage;
  }
  total_voltage_millivolts_ = total_voltage;
  battery_fuel_gauge_.updateVoltage(min_voltage, now_millis_);
}

void BmsRelay::temperatureParser(Packet& p) {
  if (p.getType() != 4) {
    return;
  }
  // 0x4 message: five unsigned bytes in degrees Celsius. The BMS clamps at
  // 0 C from below and reports 0xFF for a reading above the clamp (its
  // "sensor error" case, e.g. a shorted thermistor). The
  // first four are the pack thermistors; the fifth sits on the BMS board
  // (different NTC curve, 175 C clamp) and the BMS itself leaves it out of
  // its own min/max logic, so it is left out of the average here too.
  const uint8_t* const temperatures = p.data();
  uint16_t temperature_sum = 0;
  uint8_t valid_count = 0;
  for (int i = 0; i < 5; i++) {
    const uint8_t temp = temperatures[i];
    temperatures_celsius_[i] = temp;
    if (i == BOARD_TEMPERATURE_INDEX || temp == TEMPERATURE_INVALID) {
      continue;
    }
    temperature_sum += temp;
    valid_count++;
  }
  avg_temperature_celsius_ =
      valid_count == 0 ? 0 : temperature_sum / valid_count;
}

void BmsRelay::bmsStatusParser(Packet& p) {
  if (p.getType() != 0) {
    return;
  }
  last_status_byte_ = p.data()[0];

  // Forwarding the status packet during normal operation seems to drive
  // an event loop in the controller that tallies used Ah and eventually
  // makes the board throw Error 23. Swallowing the packet does the trick.
  p.setShouldForward(isCharging() || isBatteryEmpty() ||
                     isBatteryTempOutOfRange() || isBatteryOvercharged());
  battery_fuel_gauge_.updateChargingStatus(isCharging());
}
