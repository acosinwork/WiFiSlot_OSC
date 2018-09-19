#include <WiFiSlotOTA.h>

#include <Wire.h>

#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCTiming.h>

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

void _setup()
{

  // открываем последовательный порт
  Serial.begin(115200);
  Serial.println("Begin init...");
  // инициализация акселерометра

  // выводим сообщение об удачной инициализации
  Serial.println("Initialization completed");

  uint32_t ip = WiFi.localIP() | (255 << 24);
  outIp = IPAddress(ip);

  /*  barometer.begin();
    // создаём переменную и присваиваем ей значения абсолютного давления
    int32_t pressure = barometer.readPressureRaw();
    // создаём переменную и присваиваем ей значения высоты над уровнем море
    //  float altitude = barometer.pressureToAltitudeMeters(pressure);
    startAltitudeMeter = pressure;
  */
};

void _loop() {
  Serial.println("send");
  _delay(500);
  uint32_t noteOnMsg = 0x01;
  noteOnMsg <<= 8;
  noteOnMsg |= 0x01;
  noteOnMsg <<= 8;
  noteOnMsg |= 0x01;
//  bndl.add("/midi/noteon").add((int32_t)noteOnMsg);
  bndl.add("/1/1").add((float)(analogRead(A4) / 1023.0));

  bndl.setTimetag(oscTime());

  Udp.beginPacket(outIp, outPort);
  bndl.send(Udp); // send the bytes to the SLIP stream
  Udp.endPacket(); // mark the end of the OSC Packet
  Udp.flush();
  bndl.empty(); // empty the bundle to free room for a new one
  /*
    if (millis() - lastNoteOn >= getPot(A4)) {
      lastNoteOn = millis();
      static bool isSend = false;
    }
  */
};

