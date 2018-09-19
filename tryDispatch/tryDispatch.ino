#include <WiFiSlotOTA.h>

static const unsigned ledPin = 2;      // LED pin on Arduino Uno


//#include <OSCMessage.h>
#include <OSCBundle.h>
//#include <OSCData.h>
//#include <OSCTiming.h>
//#include <OSCMatch.h>

//#define PAULSSUGGESTION
//#define EPS 0.001f
OSCBundle bndl;

WiFiUDP Udp;
//const IPAddress WiFi.localIP();        // remote IP (not needed for receive)
const unsigned int outPort = 8000;          // remote port ()
const unsigned int localPort = 9000;        // local port to listen for UDP packets (here's where we send the packets)

//char packetBuffer[255]; //buffer to hold incoming packet
//char  ReplyBuffer[] = "acknowledged";       // a string to send back
IPAddress outIp;
//WiFiUDP Udp;

OSCErrorCode error;



void oscMidiNoteOn(OSCMessage &msg)
{
  digitalWrite(ledPin, LOW);
  //  msg.dispatch("/noteon", oscNoteOn);
  int32_t noteOnMsg = msg.getInt(0);
  uint8_t pitch = noteOnMsg & 0x7f;
  uint8_t velocity = (noteOnMsg >> 8) & 0x7f;
  uint8_t channel = (noteOnMsg >> 16) & 0x0f;
  //  MIDI.sendNoteOn(pitch, velocity, channel);
  digitalWrite(ledPin, HIGH);
}
void oscMidiCc(OSCMessage &msg)
{
  digitalWrite(ledPin, LOW);
  //  msg.dispatch("/noteon", oscNoteOn);
  int32_t noteOnMsg = msg.getInt(0);
  uint8_t cc = noteOnMsg & 0x7f;
  uint8_t value = (noteOnMsg >> 8) & 0x7f;
  uint8_t channel = (noteOnMsg >> 16) & 0x0f;
  //  MIDI.sendControlChange(cc, value, channel);
  digitalWrite(ledPin, HIGH);
}
void one(OSCMessage &msg)
{
  Serial.println("HAAAAA");
  /*
    digitalWrite(ledPin, LOW);
    //  msg.dispatch("/noteon", oscNoteOn);
    float m = msg.getFloat(0);
    Serial.println(m);
    digitalWrite(ledPin, HIGH);
  */
}
void routeOne(OSCMessage &msg, int patternOffset) {
  Serial.println("match one");
}

void oscHandler() {
  OSCBundle bundle;
  int size = Udp.parsePacket();

  if (size > 0) {
    while (size--) {
      char c = Udp.read();
      bundle.fill(c);
      Serial.print(c);
    }
    if (!bundle.hasError()) {
      Serial.println("no error");
      //      bundle.dispatch("/midi/noteon", oscMidiNoteOn);
      //      bundle.dispatch("/midi/cc", oscMidiCc);
      bundle.dispatch("/1/1", one);
      bundle.route("/1", routeOne);

    } else {
      error = bundle.getError();
      Serial.print("error: ");
      Serial.println(error);
    }
  }
}



void _setup()
{
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  //  Serial.begin(31250);
  // MIDI.begin(4);                      // Launch MIDI and listen to channel 4
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
};

void _loop() {
  oscHandler();
  /*
    _delay(200);
      digitalWrite(ledPin, HIGH);
      MIDI.sendNoteOn(42, 127, 1);    // Send a Note (pitch 42, velo 127 on channel 1)
      _delay(200);
      MIDI.sendNoteOff(42, 0, 1);     // Stop the note
      digitalWrite(ledPin, LOW);
  */
};

