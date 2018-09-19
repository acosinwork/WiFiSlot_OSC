#include <WiFiSlotOTA.h>
#include <Wire.h>
#include <I2cio.h>

#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCTiming.h>
#include <Ticker.h>

#include "controls.h"

#define PAULSSUGGESTION

OSCBundle bndl;

WiFiUDP Udp;
//const IPAddress WiFi.localIP();        // remote IP (not needed for receive)
const unsigned int outPort = 10000;          // remote port ()
const unsigned int localPort = 8000;        // local port to listen for UDP packets (here's where we send the packets)

char packetBuffer[255]; //buffer to hold incoming packet
//char  ReplyBuffer[] = "acknowledged";       // a string to send back
IPAddress outIp;
//WiFiUDP Udp;

OSCErrorCode error;

I2cio adio;
unsigned long lastUpdate = 0;

/*
  #define MAX_ADIO_PIN 49

  const byte slyder[] = {
  2
  , 0
  , 22
  };

  const byte pot[] = {
  40
  , 43
  , 12
  , 33
  , A2 + MAX_ADIO_PIN
  , A6 + MAX_ADIO_PIN
  };

  const byte button[] = {
  42
  , 45
  , 15
  , 35
  , A0 + MAX_ADIO_PIN
  , A4 + MAX_ADIO_PIN
  };

  const byte ledButton[] = {
  11
  , 31
  };

  const byte led[] = {
  10
  , 30
  };

  const byte touch[] = {
  13
  , 32
  };

  Ticker ticker;
  OSCBundle bndl;

  I2cio adio;

  int _analogRead(byte pin) {
  if ( pin > MAX_ADIO_PIN ) {
    return analogRead( pin - MAX_ADIO_PIN);
  }
  else {
    return adio.analogRead(pin);
  }
  };

  bool _digitalRead(byte pin) {
  if ( pin > MAX_ADIO_PIN ) {
    return !digitalRead( pin - MAX_ADIO_PIN);
  }
  else {
    return !adio.digitalRead(pin);
  }
  };

  void _digitalWrite(byte pin, bool value) {
  if ( pin > MAX_ADIO_PIN ) {
    digitalWrite( pin - MAX_ADIO_PIN, value);
  }
  else {
    adio.digitalWrite(pin, value);
  }
  };

  uint16_t potVal[] = {
  0
  , 0
  , 0
  , 0
  , 0
  , 0
  };
*/

Pot dec0EQHi;
Pot dec0EQMid;
Pot dec0EQLow;
Pot dec0Vol;

Pot dec1EQHi;
Pot dec1EQMid;
Pot dec1EQLow;
Pot dec1Vol;

Pot xFader;


void _setup()
{
  Wire.begin();
  adio.begin();
  adio.digitalWrite(14, HIGH); // touch HS
  adio.digitalWrite(37, HIGH); // touch HS

  dec0EQHi.attach(A6);
  dec0EQMid.attach(A2);
  dec0EQLow.attach(33, &adio);
  dec0Vol.attach(2, &adio);
  dec1EQHi.attach(40, &adio);
  dec1EQMid.attach(43, &adio);
  dec1EQLow.attach(12, &adio);
  dec1Vol.attach(0, &adio);
  xFader.attach(20, &adio);

  uint32_t ip = WiFi.localIP() | (255 << 24);
  outIp = IPAddress(ip);
};

void potUpdate() {
  dec0EQHi.update();
  dec0EQMid.update();
  dec0EQLow.update();
  dec0Vol.update();
  dec1EQHi.update();
  dec1EQMid.update();
  dec1EQLow.update();
  dec1Vol.update();
  xFader.update();
}
void addPot() {
  if (dec0EQHi.changed()) {
    bndl.add("/Traktor.Deck0.EQHigh").add(dec0EQHi.read());
  }
  if (dec0EQMid.changed()) {
    bndl.add("/Traktor.Deck0.EQMid").add(dec0EQMid.read());
  }
  if (dec0EQLow.changed()) {
    bndl.add("/Traktor.Deck0.EQLow").add(dec0EQLow.read());
  }
  if (dec0Vol.changed()) {
    bndl.add("/Traktor.Deck0.Volume").add(dec0Vol.read());
  }

  if (dec1EQHi.changed()) {
    bndl.add("/Traktor.Deck1.EQHigh").add(dec1EQHi.read());
  }
  if (dec1EQMid.changed()) {
    bndl.add("/Traktor.Deck1.EQMid").add(dec1EQMid.read());
  }
  if (dec1EQLow.changed()) {
    bndl.add("/Traktor.Deck1.EQLow").add(dec1EQLow.read());
  }
  if (dec1Vol.changed()) {
    bndl.add("/Traktor.Deck1.Volume").add(dec1Vol.read());
  }
  if (xFader.changed()) {
    bndl.add("/Traktor.XFader").add((float)(1.0-xFader.read()));
  }
}

void _loop() {
  potUpdate();

  if (millis() - lastUpdate >= 8) {
    lastUpdate = millis();

    addPot();

    bndl.setTimetag(oscTime());

    Udp.beginPacket(outIp, outPort);
    bndl.send(Udp); // send the bytes to the SLIP stream
    Udp.endPacket(); // mark the end of the OSC Packet
    Udp.flush();
    bndl.empty(); // empty the bundle to free room for a new one
  }
};

