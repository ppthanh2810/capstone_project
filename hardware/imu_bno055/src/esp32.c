#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t BNO055_ADDR = 0x29;

constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 9;

class Bno055
{
public:
  Bno055(uint8_t address) : address_(address) {}

  bool begin()
  {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeOut(50);

    delay(700);

    uint8_t id = 0;

    if (!readRegs(0x00, &id, 1) || id != 0xA0) return false;
    if (!configure()) return false;

    Serial.printf("BNO055 CHIP_ID: 0x%02X\n", id);
    Serial.println("BNO055 started");

    return true;
  }

  bool writeReg(uint8_t reg, uint8_t value)
  {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
  }

  bool readRegs(uint8_t reg, uint8_t *data, uint8_t length)
  {
    Wire.beginTransmission(address_);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) return false;

    int count = Wire.requestFrom(address_, length);

    if (count != length) {
      while (Wire.available()) Wire.read();
      return false;
    }

    for (int i = 0; i < length; ++i)
      data[i] = Wire.read();

    return true;
  }

  int16_t toInt16(uint8_t lsb, uint8_t msb)
  {
    return static_cast<int16_t>(
      static_cast<uint16_t>(lsb) |
      (static_cast<uint16_t>(msb) << 8));
  }

  bool configure()
  {
    // Page 0
    if (!writeReg(0x07, 0x00)) return false;

    // CONFIGMODE
    if (!writeReg(0x3D, 0x00)) return false;
    delay(100);

    // Normal power mode
    if (!writeReg(0x3E, 0x00)) return false;
    delay(100);

    // NDOF mode
    if (!writeReg(0x3D, 0x0C)) return false;
    delay(1000);

    uint8_t mode = 0;

    return readRegs(0x3D, &mode, 1) && mode == 0x0C;
  }

  bool readImu()
  {
    uint8_t q[8], g[6], a[6];

    if (!readRegs(0x20, q, 8)) return false;
    if (!readRegs(0x14, g, 6)) return false;
    if (!readRegs(0x28, a, 6)) return false;

    int16_t qw = toInt16(q[0], q[1]);
    int16_t qx = toInt16(q[2], q[3]);
    int16_t qy = toInt16(q[4], q[5]);
    int16_t qz = toInt16(q[6], q[7]);

    int16_t gx = toInt16(g[0], g[1]);
    int16_t gy = toInt16(g[2], g[3]);
    int16_t gz = toInt16(g[4], g[5]);

    int16_t ax = toInt16(a[0], a[1]);
    int16_t ay = toInt16(a[2], a[3]);
    int16_t az = toInt16(a[4], a[5]);

    Serial.printf(
      "IMU,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
      static_cast<unsigned long>(millis()),
      qw, qx, qy, qz,
      gx, gy, gz,
      ax, ay, az);

    return true;
  }

private:
  uint8_t address_;
};

Bno055 imu(BNO055_ADDR);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  if (!imu.begin()) {
    Serial.println("BNO055 initialization failed");

    while (true) delay(1000);
  }
}

void loop()
{
  static uint32_t last_time = 0;

  if (millis() - last_time < 20) return;

  last_time = millis();

  if (!imu.readImu())
    Serial.println("BNO055 read failed");
}