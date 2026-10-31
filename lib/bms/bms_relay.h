#ifndef BMS_RELAY_H
#define BMS_RELAY_H

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include "battery_fuel_gauge.h"
#include "packet_tracker.h"

class Packet;

class BmsRelay {
 public:
  /**
   * @brief Function polled for new byte from BMS. Expected to return negative
   * value when there's no data available on the wire.
   */
  typedef std::function<int()> Source;
  /**
   * @brief Function called to send data to the MB.
   */
  typedef std::function<void(uint8_t)> Sink;

  /**
   * @brief Packet callback.
   */
  typedef std::function<void(BmsRelay*, Packet*)> PacketCallback;

  typedef std::function<unsigned long()> MillisProvider;

  /**
   * @brief Called once when the BMS announces that it is powering off.
   * Right before shutting down the BMS sends a bare ASCII fault code without
   * any frame header ("BU" = power button, "DC" = done charging, "CD" =
   * charger disconnected, "ZC" = idle timeout, "UV"/"CUV"/"OV"/"OT"/"OC"/
   * "QC"/"PR"/"CS"/"CC" = protections), then drains its TX buffer and waits
   * about a second before cutting power. The argument is that code.
   */
  typedef std::function<void(const char*)> ShutdownCallback;

  BmsRelay(const Source& source, const Sink& sink,
           const MillisProvider& millisProvider);

  /**
   * @brief All of the data ingestion, processing and forwarding is done here.
   * Must be called continuously from arduino loop.
   */
  void loop();

  void addReceivedPacketCallback(const PacketCallback& callback) {
    receivedPacketCallbacks_.push_back(callback);
  }

  void addForwardedPacketCallback(const PacketCallback& callback) {
    forwardedPacketCallbacks_.push_back(callback);
  }

  void setUnknownDataCallback(const Sink& c) { unknownDataCallback_ = c; }

  void setShutdownCallback(const ShutdownCallback& c) { shutdownCallback_ = c; }

  /**
   * @brief Seconds until the BMS shuts itself down, as last announced in
   * message 0x01, or -1 when no warning was received in the last 2 seconds.
   */
  int getShutdownCountdownSeconds();

  /**
   * @brief Reason bit mask of the last 0x01 warning: 1 = over-voltage,
   * 2 = under-voltage, 4 = over-temperature, 8 = idle (zero current) timer.
   */
  uint8_t getShutdownReasonMask() { return shutdown_reason_mask_; }

  /**
   * @brief Fault code text the BMS sent before powering off ("BU", "DC",
   * ...), empty string if none was seen yet.
   */
  const char* getShutdownCode() { return shutdown_code_; }

  /**
   * @brief If set to non-zero value, spoofs captured BMS serial
   * with the number provided here. The serial number can be found
   * on the sticker on the bottom side of BMS. The lower number on
   * the sticker without the 4 leading (most significant) digits
   * goes here.
   */
  void setBMSSerialOverride(uint32_t serial) { serial_override_ = serial; }

  /**
   * @brief Get the captured BMS serial number.
   *
   * @return non-zero serial, if it's already been captured.
   */
  uint32_t getCapturedBMSSerial() { return captured_serial_; }

  /**
   * @brief Get the BMS serial override value.
   *
   * @return The serial override value, or 0 if not set.
   */
  uint32_t getBMSSerialOverride() { return serial_override_; }

  /**
   * @brief Battery percentage as reported by the BMS.
   */
  int8_t getBmsReportedSOC() { return bms_soc_percent_; }

  /**
   * @brief Spoofed battery percentage sent to the controller.
   */
  int8_t getOverriddenSOC() { return overridden_soc_percent_; }
  /**
   * @brief Cell voltages in millivolts.
   * @return pointer to a 15 element array.
   */
  uint16_t* const getCellMillivolts() { return cell_millivolts_; }

  uint16_t getTotalVoltageMillivolts() { return total_voltage_millivolts_; }

  /**
   * @brief Cell voltages in millivolts.
   * @return pointer to a 5 element array.
   */
  int8_t* const getTemperaturesCelsius() { return temperatures_celsius_; }

  uint8_t getAverageTemperatureCelsius() { return avg_temperature_celsius_; }

  /**
   * @brief Current In Amps.
   */
  int32_t getCurrentMilliamps() { return current_milliamps_; }

  int32_t getUsedChargeMah() {
    return battery_fuel_gauge_.getMilliampSecondsDischarged() / 3600;
  }
  int32_t getRegeneratedChargeMah() {
    return battery_fuel_gauge_.getMilliampSecondsRecharged() / 3600;
  }

  bool isCharging() { return last_status_byte_ & 0x20; }
  bool isBatteryEmpty() { return last_status_byte_ & 0x4; }
  bool isBatteryTempOutOfRange() {
    // Both these bits throw the same error, one is prob hot and the other is
    // cold.
    return last_status_byte_ & 3;
  }
  bool isBatteryOvercharged() { return last_status_byte_ & 8; }

  const PacketTracker& getPacketTracker() const { return packet_tracker_; }

  BatteryFuelGauge& getBatteryFuelGauge() { return battery_fuel_gauge_; }

 private:
  // BMS current units to milliamps.
  static constexpr int CURRENT_SCALER = 55;
  void processNextByte();
  void purgeUnknownData();
  void trackUnknownByte(uint8_t b);
  void maybeReplayPackets();
  void ingestPacket(Packet& p);

  std::vector<PacketCallback> receivedPacketCallbacks_;
  std::vector<PacketCallback> forwardedPacketCallbacks_;
  Sink unknownDataCallback_;
  ShutdownCallback shutdownCallback_;

  // Detection of the header-less fault code that precedes a BMS power-off:
  // the first bytes of an unknown-data run that directly follows a valid
  // packet.
  bool seen_valid_packet_ = false;
  uint8_t unknown_run_[3] = {0};
  uint8_t unknown_run_len_ = 0;
  bool unknown_run_fired_ = false;
  char shutdown_code_[4] = {0};

  int8_t shutdown_countdown_seconds_ = -1;
  uint8_t shutdown_reason_mask_ = 0;
  int32_t shutdown_warning_millis_ = 0;

  std::vector<uint8_t> sourceBuffer_;
  uint32_t serial_override_ = 0;
  uint32_t captured_serial_ = 0;
  int16_t current_milliamps_ = 0;

  int8_t bms_soc_percent_ = -1;
  int8_t overridden_soc_percent_ = -1;
  uint16_t cell_millivolts_[15] = {0};
  uint16_t total_voltage_millivolts_ = 0;

  int8_t temperatures_celsius_[5] = {0};
  int8_t avg_temperature_celsius_ = 0;
  uint8_t last_status_byte_ = 0;
  const Source source_;
  const Sink sink_;
  const MillisProvider millis_provider_;
  int32_t now_millis_;
  PacketTracker packet_tracker_;
  BatteryFuelGauge battery_fuel_gauge_;

  void bmsStatusParser(Packet& p);
  void shutdownWarningParser(Packet& p);
  void bmsSerialParser(Packet& p);
  void currentParser(Packet& p);
  void batteryPercentageParser(Packet& p);
  void cellVoltageParser(Packet& p);
  void temperatureParser(Packet& p);
};

#endif  // BMS_RELAY_H