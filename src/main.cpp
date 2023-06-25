#include "main.h"
#include "pros/serial.hpp"
#include <atomic>
#include <bitset>
#include <numeric>

std::atomic_bool tareFlag{false};
std::atomic_bool testFlag{false};
std::atomic<uint32_t> testStartTime{0};

// std::optional<std::array<uint8_t, 8>> readNibbles(pros::Serial serial) {
//   std::array<uint8_t, 8> outArray;
//   for (size_t i = 0; i < outArray.size(); i++) {
//     while (!serial.get_read_avail()) {
//       pros::delay(1);
//     };

//     uint8_t rawByte = static_cast<uint8_t>(serial.peek_byte());
//     uint8_t data = rawByte & 0B00001111;
//     uint8_t position = rawByte >> 4;

//     if (position != i) {
//       return {}; // Has to be in order
//     }

//     serial.read_byte(); // Clean from buf
//     outArray[i] = data;
//   }
//   return outArray;
// }

// uint32_t packLeastSigBits(std::array<uint8_t, 8> array) {
//   uint8_t byteArray[4] = {0};
//   for (size_t i = 0; i + 1 < array.size(); i += 2) {
//     uint8_t msNibble = array[i + 1] << 4;
//     uint8_t lsNibble = array[i] & 0B00001111;
//     byteArray[i / 2] |= msNibble |= lsNibble;
//   }
//   return *reinterpret_cast<uint32_t *>(&byteArray);
// }

lv_res_t tareCallback(lv_obj_t __attribute__((unused)) * obj) {
  tareFlag = true;
  return LV_RES_OK;
}

lv_res_t testCallback(lv_obj_t *obj) {
  testFlag = true;
  testStartTime = pros::millis();
  return LV_RES_OK;
}

void opcontrol() {
  std::vector<double> tareVector;
  double perNewton{1328.09681619};
  double tareValue{0};
  // uint32_t testStartTime = pros::millis();

  pros::MotorGroup motors{16, 2, 3, -4, -5, -6};

  tareVector.resize(100);
  tareVector.clear();
  std::cout << "Vec size: " << tareVector.size() << "\n";

  lv_obj_t *tareSwitch = lv_sw_create(lv_scr_act(), NULL);
  lv_obj_t *testSwitch = lv_sw_create(lv_scr_act(), NULL);

  lv_obj_t *tareLabel = lv_label_create(tareSwitch, NULL);
  lv_label_set_text(tareLabel, "Tare");
  lv_obj_align(tareLabel, lv_scr_act(), LV_ALIGN_IN_TOP_LEFT, 25, 25);
  lv_obj_align(tareSwitch, tareLabel, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

  lv_obj_t *testLabel = lv_label_create(testSwitch, NULL);
  lv_label_set_text(testLabel, "Test");
  lv_obj_align(testLabel, tareSwitch, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
  lv_obj_align(testSwitch, testLabel, LV_ALIGN_OUT_RIGHT_MID, 10, 0);

  lv_sw_set_action(tareSwitch, tareCallback);
  lv_sw_set_action(testSwitch, testCallback);

  pros::Serial serial{8, 115200};
  serial.flush();

  while (true) {
    uint32_t uValue = 0;
    // Reverse loop because MSB's are sent first
    //for (int i = 7; i >= 0; i--) {
    for(int i = 0; i < 8; i++){
      while (!serial.get_read_avail()) {
        pros::delay(1);
      };
      // Extract the position and data bits
      uint8_t rawByte = static_cast<uint8_t>(serial.peek_byte());
      //uint8_t data = rawByte & 0B00001111;
      uint32_t data = rawByte << 28;
      uint8_t position = rawByte >> 4;
      if (position != i) {
        // Reset loop and value if byte is out of order
        i = 0;
        uValue = 0;
      }
      serial.read_byte(); // Clean byte from serial buffer
      uValue >>= 4;        // Move previous byte's data out of the way
      uValue |= data;      // Prepend this byte's data
    }
    int32_t value = std::bit_cast<int32_t>(uValue);
    // From here value is guaranteed to be valid

    double newtons = value / perNewton - tareValue;
    //std::cout << "Newtons: " << newtons << " Value: " << value << "\n";

    if (tareFlag && tareVector.size() < 100) {
      tareVector.push_back(value / perNewton);
    }
    if (tareFlag && tareVector.size() >= 100) {
      tareValue =
          std::accumulate(tareVector.begin(), tareVector.end(), double{0}) /
          tareVector.size();
      tareVector.clear();
      tareFlag = false;
      lv_sw_off(tareSwitch);
    }

    if (testFlag && pros::millis() - testStartTime < 70'000) {
      motors.move_voltage((pros::millis() - testStartTime) / 5);
      std::cout << newtons << ", " << motors[0].get_actual_velocity() << ", " << motors[0].get_voltage() << "\n";
    }
    if (testFlag && pros::millis() - testStartTime >= 70'000) {
      motors.move(0);
      testFlag = false;
      lv_sw_off(testSwitch);
    }
  }
}
