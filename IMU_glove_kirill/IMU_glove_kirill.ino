#include <WiFiSlotOTA.h>

#include <Wire.h>
// библиотека для работы с модулями IMU
#include <TroykaIMU.h>

#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCTiming.h>

#define PAULSSUGGESTION
#define EPS 0.001f
OSCBundle bndl;

WiFiUDP Udp;
//const IPAddress WiFi.localIP();        // remote IP (not needed for receive)
const unsigned int outPort = 9000;          // remote port ()
const unsigned int localPort = 8000;        // local port to listen for UDP packets (here's where we send the packets)

char packetBuffer[255]; //buffer to hold incoming packet
//char  ReplyBuffer[] = "acknowledged";       // a string to send back
IPAddress outIp;
//WiFiUDP Udp;

OSCErrorCode error;

//Barometer barometer;
int32_t startAltitudeMeter = 0;
// множитель фильтра
#define BETA 0.22f

// создаём объект для фильтра Madgwick
Madgwick filter;

// создаём объект для работы с акселерометром
Accelerometer accel(ACCEL_ADDRESS_V2);
// создаём объект для работы с гироскопом
Gyroscope gyro;

// переменные для данных с гироскопов, акселерометров
float gx, gy, gz, ax, ay, az;

// получаемые углы ориентации
float yaw, pitch, roll;
float lastYaw, lastRoll, lastPitch;
uint8_t lastMidiPitch = 0;
// переменная для хранения частоты выборок фильтра
float fps = 100;

unsigned long lastUpdate = 0;
unsigned long lastNoteOn = 0;

uint16_t getPot(byte pot) {
  static uint16_t lastVal = 0;

  uint16_t newVal = analogRead(pot);

  float val = (float)newVal * 0.15 + (float)lastVal * 0.85;
  lastVal = val;
  return lastVal;
};

void _setup()
{
_delay(500);
  // открываем последовательный порт
  Serial.begin(115200);
  Serial.println("Begin init...");
  // инициализация акселерометра
  accel.begin();
  // инициализация гироскопа
  gyro.begin();
  // выводим сообщение об удачной инициализации
  Serial.println("Initialization completed");

  uint32_t ip = WiFi.localIP() | (255 << 24);
  outIp = IPAddress(ip);

  lastYaw = lastRoll = lastYaw = 0;
  /*  barometer.begin();
    // создаём переменную и присваиваем ей значения абсолютного давления
    int32_t pressure = barometer.readPressureRaw();
    // создаём переменную и присваиваем ей значения высоты над уровнем море
    //  float altitude = barometer.pressureToAltitudeMeters(pressure);
    startAltitudeMeter = pressure;
  */
};

void sendNoteOn(uint8_t midiPitch, uint8_t velocity, uint8_t channel) {
  uint32_t noteOnMsg = channel & 0x0f;
  noteOnMsg <<= 8;
  noteOnMsg |= velocity & 0x7f;
  noteOnMsg <<= 8;
  noteOnMsg |= midiPitch & 0x7f;
  bndl.add("/midi/noteon").add((int32_t)noteOnMsg);
};

void sendCC(uint8_t ccNum, uint8_t value, uint8_t channel) {
  uint32_t noteOnMsg = channel;
  noteOnMsg <<= 8;
  noteOnMsg |= value;
  noteOnMsg <<= 8;
  noteOnMsg |= ccNum;
  bndl.add("/midi/cc").add((int32_t)noteOnMsg);
};

float getAcceleration() {
  static float lastG = 0;
  static float derivative = 0;
  float g = sqrt(ax * ax + ay * ay + az * az);
  float currDerivative = g - lastG;
  float val = fabs(derivative);
  float result = 0.0;
  if ((derivative < 0) && (currDerivative > 0) && (val > 0.1))
  {
    result = fabs(derivative);
  }

  if ((derivative > 0) && (currDerivative < 0) && (val > 0.1))
  {
    result = fabs(derivative);
  }
  derivative = currDerivative;
  lastG = g;

  return result;
}

void _loop() {

  //  uint64_t timetag;
  // запоминаем текущее время

  unsigned long startMillis = millis();

  // считываем данные с акселерометра в единицах G
  accel.readGXYZ(&ax, &ay, &az);
  // считываем данные с акселерометра в радианах в секунду
  gyro.readRadPerSecXYZ(&gx, &gy, &gz);
  // устанавливаем коэффициенты фильтра
  filter.setKoeff(fps, BETA);
  // обновляем входные данные в фильтр
  filter.update(gx, gy, gz, ax, ay, az);

  // получение углов yaw, pitch и roll из фильтра
  yaw =  0.5 + filter.getYawDeg() / 360.0;
  pitch = 0.5 + filter.getPitchDeg() / 360.0;
  roll = 0.5 + filter.getRollDeg() / 360.0;

  // вычисляем затраченное время на обработку данных
  unsigned long deltaMillis = millis() - startMillis;
  // вычисляем частоту обработки фильтра
  fps = 1000 / deltaMillis;


  if (millis() - lastUpdate >= 10) {
    lastUpdate = millis();

    static uint16_t lastPot = 0;

    uint16_t pot = getPot(A4);

    if (pot != lastPot) {
      uint8_t value = pot >> 3;
      sendCC(22, value, 1);
      bndl.add("glove/pot").add((float)pot / 1024.0);
      lastPot = pot;
    }

    float acceleration = getAcceleration();

    if (acceleration > 0.001) {
      //    uint32_t noteOnMsg = msg.getInt(0);
      uint8_t midiPitch = yaw * 127; //noteOnMsg & 0x7f;
      uint8_t velocity = constrain(acceleration * 127, 0, 127);
      uint8_t channel = 1;
      sendNoteOn(lastMidiPitch, 0, channel);
      sendNoteOn( midiPitch, velocity, channel);
      lastMidiPitch = midiPitch;
    }


    if (fabs(yaw - lastYaw) > EPS) {
      lastYaw = yaw;
      bndl.add("glove/yaw").add(yaw);

    }

    if (fabs(pitch - lastPitch) > EPS) {
      lastPitch = pitch;
      uint8_t value = pitch * 127;
      sendCC(19, value, 1);
      bndl.add("glove/pitch").add(pitch);
      //    bndl.add("1/fader2").add(pitch);
    }

    if (fabs(roll - lastRoll) > EPS) {
      lastRoll = roll;
      uint8_t value = roll * 127;
      sendCC(21, value, 1);
      bndl.add("glove/roll").add(roll);
      //bndl.add("1/fader3").add(roll);
    }

    bndl.setTimetag(oscTime());

    Udp.beginPacket(outIp, outPort);
    bndl.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    Udp.flush();
    bndl.empty(); // empty the bundle to free room for a new one
  }

};

