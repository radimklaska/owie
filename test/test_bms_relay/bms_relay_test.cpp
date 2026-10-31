#include "bms_relay.h"

#include <unity.h>

#include <cstdio>
#include <deque>
#include <memory>
#include <string>

#include "packet.h"

std::unique_ptr<BmsRelay> relay;
std::deque<int> mockBmsData;
std::vector<uint8_t> mockDataOut;
unsigned long timeMillis = 0;

void setUp(void) {
  relay.reset(new BmsRelay(
      [&]() {
        if (mockBmsData.empty()) {
          return -1;
        }
        int data = mockBmsData.front();
        mockBmsData.pop_front();
        return data;
      },
      [&](uint8_t b) {
        mockDataOut.push_back(b);
        return 1;
      },
      [&]() { return timeMillis; }));
  mockBmsData.clear();
  mockDataOut.clear();
}

void addMockData(const std::vector<uint8_t>& data) {
  mockBmsData.insert(mockBmsData.end(), data.begin(), data.end());
}

void expectDataOut(const std::vector<uint8_t>& expected) {
  TEST_ASSERT_EQUAL(expected.size(), mockDataOut.size());
  if (expected.size() == 0) {
    return;
  }
  TEST_ASSERT_EQUAL_UINT8_ARRAY(&expected[0], &mockDataOut[0], expected.size());
  mockDataOut.clear();
}

void testUnknownBytesGetsForwardedImmediately(void) {
  addMockData({0x1, 0x2, 0x3});
  relay->loop();
  TEST_ASSERT_TRUE(mockBmsData.empty());
  expectDataOut({0x1, 0x2, 0x3});
}

void testUnknownDataAfterKnownPacketGetsFlushedImmediately(void) {
  addMockData({0xFF, 0x55, 0xAA, 0x6, 0x1, 0x2, 0x3, 0x4, 0x2, 0xE, 0x1});
  relay->loop();
  TEST_ASSERT_TRUE(mockBmsData.empty());
  expectDataOut({0xFF, 0x55, 0xAA, 0x6, 0x1, 0x2, 0x3, 0x4, 0x2, 0xE, 0x1});
}

void testSerialGetsRecordedAndIntercepted(void) {
  addMockData(
      {0x1, 0x2, 0x3, 0xFF, 0x55, 0xAA, 0x6, 0x1, 0x2, 0x3, 0x4, 0x2, 0xE});
  relay->setBMSSerialOverride(0x8040201);
  relay->loop();
  TEST_ASSERT_TRUE(mockBmsData.empty());
  TEST_ASSERT_EQUAL(0x1020304, relay->getCapturedBMSSerial());
  // Expect no data so far because we identify packets only when we see the
  // header of the next one.
  expectDataOut(
      {0x1, 0x2, 0x3, 0xFF, 0x55, 0xAA, 0x6, 0x8, 0x4, 0x2, 0x1, 0x2, 0x13});
}

void testPacketLengths() {
  std::vector<std::vector<uint8_t>> packets = {
      {0xff, 0x55, 0xaa, 0x00, 0x80, 0x02, 0x7e},
      {0xff, 0x55, 0xaa, 0x01, 0x1c, 0x02, 0x02, 0x1d},
      {0xff, 0x55, 0xaa, 0x02, 0x0f, 0x28, 0x0f, 0x2c, 0x0f, 0x2b,
       0x0f, 0x29, 0x0f, 0x2a, 0x0f, 0x2b, 0x0f, 0x2a, 0x0f, 0x2c,
       0x0f, 0x29, 0x0f, 0x2b, 0x0f, 0x29, 0x0f, 0x2a, 0x0f, 0x22,
       0x0f, 0x2a, 0x0f, 0x2a, 0x00, 0x2a, 0x05, 0x7b},
      {0xff, 0x55, 0xaa, 0x03, 0x29, 0x02, 0x2a},
      {0xff, 0x55, 0xaa, 0x04, 0x16, 0x17, 0x17, 0x17, 0x18, 0x02, 0x75},
      {0xff, 0x55, 0xaa, 0x05, 0x00, 0x01, 0x02, 0x04},
      {0xFF, 0x55, 0xAA, 0x6, 0x8, 0x4, 0x2, 0x1, 0x2, 0x13},
      {0xff, 0x55, 0xaa, 0x07, 0x10, 0xcc, 0x10, 0x57, 0x09, 0xc4, 0x50, 0x04,
       0x65},
      {0xff, 0x55, 0xaa, 0x08, 0x06, 0x02, 0x0c},
      {0xff, 0x55, 0xaa, 0x09, 0x03, 0x02, 0x0a},
      {0xff, 0x55, 0xaa, 0x0b, 0x0b, 0xc0, 0x02, 0xd4},
      {0xff, 0x55, 0xaa, 0x0c, 0x00, 0x00, 0x02, 0x0a},
      {0xff, 0x55, 0xaa, 0x0d, 0x02, 0xda, 0x47, 0x03, 0x2e},
      {0xff, 0x55, 0xaa, 0x0e, 0x01, 0x02, 0x0d},
      {0xff, 0x55, 0xaa, 0x0f, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0f},
      {0xff, 0x55, 0xaa, 0x10, 0x03, 0x03, 0x0b, 0x03, 0x03, 0x03, 0x03, 0x03,
       0x03, 0x03, 0x02, 0x34},
      {0xff, 0x55, 0xaa, 0x11, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0f}};
  std::vector<uint8_t> receivedPacket;
  relay->addReceivedPacketCallback([&](BmsRelay*, Packet* p) {
    TEST_ASSERT_TRUE(p->isValid());
    receivedPacket = std::vector<uint8_t>(p->start(), p->start() + p->len());
  });
  for (const auto& packet : packets) {
    addMockData(packet);
    relay->loop();
    TEST_ASSERT_EQUAL(packet.size(), receivedPacket.size());
    TEST_ASSERT_EQUAL_UINT8_ARRAY(&packet[0], &receivedPacket[0],
                                  packet.size());
  }
}

void testPacketCallback() {
  addMockData({0x1, 0x2, 0x3, 0xFF, 0x55, 0xAA, 0x6, 0x1, 0x2, 0x3, 0x4, 0x2,
               0xE, 0xFF, 0x55, 0xAA});
  std::vector<uint8_t> receivedPacket;
  relay->addReceivedPacketCallback([&](BmsRelay*, Packet* p) {
    const uint8_t* start = p->start();
    receivedPacket.assign(start, start + p->len());
  });
  relay->loop();
  std::vector<uint8_t> expected(
      {0xFF, 0x55, 0xAA, 0x6, 0x1, 0x2, 0x3, 0x4, 0x2, 0xE});
  TEST_ASSERT_EQUAL(expected.size(), receivedPacket.size());
  for (int i = 0; i < expected.size(); i++) {
    TEST_ASSERT_EQUAL(expected[i], receivedPacket[i]);
  }
}

void testBatterySocParsing() {
  addMockData({0xFF, 0x55, 0xAA, 0x3, 0x2B, 0x02, 0x2C, 0xFF, 0x55, 0xAA});
  relay->loop();
  TEST_ASSERT_EQUAL(43, relay->getBmsReportedSOC());
}

void testCurrentParsing() {
  addMockData({0xff, 0x55, 0xaa, 0x5, 0xff, 0xe8, 0x3, 0xea, 0xFF, 0x55, 0xAA});
  relay->loop();
  TEST_ASSERT_EQUAL(-1320, relay->getCurrentMilliamps());
}

void testCellVoltageParsing() {
  addMockData({0xff, 0x55, 0xaa, 0x02, 0x0f, 0x14, 0x0f, 0x14, 0x0f, 0x14, 0x0f,
               0x13, 0x0f, 0x14, 0x0f, 0x14, 0x0f, 0x14, 0x0f, 0x13, 0x0f, 0x14,
               0x0f, 0x13, 0x0f, 0x13, 0x0f, 0x13, 0x0f, 0x13, 0x0f, 0x14, 0x0f,
               0x14, 0x00, 0x2a, 0x04, 0x31, 0xFF, 0x55, 0xAA});
  relay->loop();
  uint16_t expected[15] = {3860, 3860, 3860, 3859, 3860, 3860, 3860, 3859,
                           3860, 3859, 3859, 3859, 3859, 3860, 3860};
  TEST_ASSERT_EQUAL_INT16_ARRAY(expected, relay->getCellMillivolts(), 15);
  TEST_ASSERT_EQUAL(57894, relay->getTotalVoltageMillivolts());
}

void testTemperatureParsing() {
  addMockData({0xff, 0x55, 0xaa, 0x04, 0x13, 0x14, 0x14, 0x14, 0x16, 0x02, 0xFF,
               0x55, 0xAA});
  relay->loop();
  int8_t expected[5] = {19, 20, 20, 20, 22};
  TEST_ASSERT_EQUAL_INT8_ARRAY(expected, relay->getTemperaturesCelsius(), 5);
  TEST_ASSERT_EQUAL(20, relay->getAverageTemperatureCelsius());
}

void testBlocksStatusPacketsUnlessWarning() {
  addMockData({0xff, 0x55, 0xaa, 0x0, 0x0, 0x1, 0xFE, 0x1});
  relay->loop();
  expectDataOut({0x1});
  // Charging bit is present, expect all data to go through.
  addMockData({0xff, 0x55, 0xaa, 0x0, 0x21, 0x2, 0x1F, 0x1});
  relay->loop();
  expectDataOut({0xff, 0x55, 0xaa, 0x0, 0x21, 0x2, 0x1F, 0x1});
}

void testPacketReplay() {
  addMockData({0xff, 0x55, 0xaa, 0x0, 0x21, 0x2, 0x1F, 0x1});
  addMockData({0xff, 0x55, 0xaa, 0x08, 0x06, 0x02, 0x0c});
  timeMillis = 0;
  relay->loop();
  expectDataOut({0xff, 0x55, 0xaa, 0x0, 0x21, 0x2, 0x1F, 0x1, 0xff, 0x55, 0xaa,
                 0x08, 0x06, 0x02, 0x0c});
  // Under the status packet timeout limit
  timeMillis = 499;
  relay->loop();
  expectDataOut({});
  // Over the timeout, trigger replay
  timeMillis = 501;
  relay->loop();

  // Make sure that replayed packet reset the stats so we don't
  // end up spamming the controller
  expectDataOut({0xff, 0x55, 0xaa, 0x0, 0x21, 0x2, 0x1F});
  timeMillis = 601;
  relay->loop();
  expectDataOut({});

  // Rewind time to far future to trigger both slow and fast packet
  // replay.
  timeMillis = 10000;
  relay->loop();
  expectDataOut({0xff, 0x55, 0xaa, 0x0, 0x21, 0x2, 0x1F, 0xff, 0x55, 0xaa, 0x08,
                 0x06, 0x02, 0x0c});
}

void testShutdownWarningParsedAndNeverReplayed() {
  timeMillis = 1000;
  // 28 s left, reason 2 = under-voltage.
  addMockData({0xff, 0x55, 0xaa, 0x01, 0x1c, 0x02, 0x02, 0x1d});
  relay->loop();
  expectDataOut({0xff, 0x55, 0xaa, 0x01, 0x1c, 0x02, 0x02, 0x1d});
  TEST_ASSERT_EQUAL(28, relay->getShutdownCountdownSeconds());
  TEST_ASSERT_EQUAL(2, relay->getShutdownReasonMask());
  // Still valid just under 2 s later, gone after that.
  timeMillis = 2999;
  relay->loop();
  TEST_ASSERT_EQUAL(28, relay->getShutdownCountdownSeconds());
  timeMillis = 3001;
  relay->loop();
  TEST_ASSERT_EQUAL(-1, relay->getShutdownCountdownSeconds());
  // Far in the future: no replay of the warning, unlike regular packets.
  timeMillis = 100000;
  relay->loop();
  expectDataOut({});
}

void testShutdownCodeTriggersCallbackOnce() {
  std::string code;
  int fires = 0;
  relay->setShutdownCallback([&](const char* c) {
    code = c;
    fires++;
  });
  const std::vector<uint8_t> validPacket = {0xff, 0x55, 0xaa, 0x08,
                                            0x06, 0x02, 0x0c};
  // Garbage before the first valid packet is ignored.
  addMockData({'B', 'U'});
  relay->loop();
  TEST_ASSERT_EQUAL(0, fires);
  TEST_ASSERT_EQUAL_STRING("", relay->getShutdownCode());

  // Power button: "BU" right after a frame, followed by the 10-byte record
  // that 4210/Pint boards append. Exactly one callback.
  addMockData(validPacket);
  relay->loop();
  addMockData({'B', 'U'});
  relay->loop();
  TEST_ASSERT_EQUAL(1, fires);
  TEST_ASSERT_EQUAL_STRING("BU", code.c_str());
  TEST_ASSERT_EQUAL_STRING("BU", relay->getShutdownCode());
  addMockData({0x07, 0xd7, 0xff, 0xab, 0xcd, 0xef, 0x03, 0x00, 0x10, 0x62});
  relay->loop();
  TEST_ASSERT_EQUAL(1, fires);

  // Three-letter code.
  addMockData(validPacket);
  relay->loop();
  addMockData({'C', 'U', 'V'});
  relay->loop();
  TEST_ASSERT_EQUAL(2, fires);
  TEST_ASSERT_EQUAL_STRING("CUV", code.c_str());

  // "PR" carries three binary bytes after the text; still one callback.
  addMockData(validPacket);
  relay->loop();
  addMockData({'P', 'R', 0x50, 0x08, 0x00});
  relay->loop();
  TEST_ASSERT_EQUAL(3, fires);
  TEST_ASSERT_EQUAL_STRING("PR", code.c_str());

  // A run that does not start with a code never fires, even if a code
  // appears later in it.
  addMockData(validPacket);
  relay->loop();
  addMockData({'Z', 'Z', 'B', 'U'});
  relay->loop();
  TEST_ASSERT_EQUAL(3, fires);

  // Unknown bytes are still forwarded unchanged.
  TEST_ASSERT_TRUE(mockDataOut.size() > 0);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();
  RUN_TEST(testUnknownDataAfterKnownPacketGetsFlushedImmediately);
  RUN_TEST(testPacketLengths);
  RUN_TEST(testSerialGetsRecordedAndIntercepted);
  RUN_TEST(testUnknownBytesGetsForwardedImmediately);
  RUN_TEST(testPacketCallback);
  RUN_TEST(testBatterySocParsing);
  RUN_TEST(testCurrentParsing);
  RUN_TEST(testCellVoltageParsing);
  RUN_TEST(testBlocksStatusPacketsUnlessWarning);
  RUN_TEST(testPacketReplay);
  RUN_TEST(testShutdownWarningParsedAndNeverReplayed);
  RUN_TEST(testShutdownCodeTriggersCallbackOnce);
  UNITY_END();

  return 0;
}