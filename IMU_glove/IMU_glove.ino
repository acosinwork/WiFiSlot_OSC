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

Barometer barometer;
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
  barometer.begin();
  // создаём переменную и присваиваем ей значения абсолютного давления
  int32_t pressure = barometer.readPressureRaw();
  // создаём переменную и присваиваем ей значения высоты над уровнем море
  //  float altitude = barometer.pressureToAltitudeMeters(pressure);
  startAltitudeMeter = pressure;
};

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

    if (fabs(yaw - lastYaw) > EPS) {
      lastYaw = yaw;
      //    bndl.add("glove/yaw").add(yaw);
      bndl.add("1/fader1").add(yaw);
    }
    if (fabs(pitch - lastPitch) > EPS) {
      lastPitch = pitch;
      //    bndl.add("glove/pitch").add(pitch);
      bndl.add("1/fader2").add(pitch);
    }
    if (fabs(roll - lastRoll) > EPS) {
      lastRoll = roll;
      //    bndl.add("glove/roll").add(roll);
      bndl.add("1/fader3").add(roll);
    }
    bndl.setTimetag(oscTime());

    Udp.beginPacket(outIp, outPort);
    bndl.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    Udp.flush();
    bndl.empty(); // empty the bundle to free room for a new one
  }
  if (millis() - lastNoteOn >= getPot(A4)) {
    lastNoteOn = millis();
    static bool isSend = false;
/*
    int32_t pressure = barometer.readPressureRaw();// - startAltitudeMeter;
    // создаём переменную и присваиваем ей значения высоты над уровнем море
    //    float altitude = (barometer.pressureToAltitudeMeters(pressure) - startAltitudeMeter);

    Serial.println(pressure);
    /*
        uint32_t noteOnMsg = msg.getInt(0);
        uint8_t pitch = noteOnMsg & 0x7f;
        uint8_t velocity = (noteOnMsg >> 8) & 0x7f;
        uint8_t channel = (noteOnMsg >> 16) & 0x7f;
    */
  }
};

