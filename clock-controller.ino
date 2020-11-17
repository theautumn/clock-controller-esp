// This is a modified version of the clock controller for use with a 
// regular old NodeMCU ESP32. No display or anything fancy.


/**
   The MIT License (MIT)

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/


#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Timezone.h>    // https://github.com/JChristensen/Timezone
#include <TimeLib.h>     // https://playground.arduino.cc/Code/Time
#include "credentials.h" // Wifi

// NTP server name or IP, sync interval in seconds
static const char ntpServerName[] = "us.pool.ntp.org";
#define NTP_SYNC_INTERVAL 300

// Screensaver to save OLED
#define SCREENSAVER_TIMER 600

// Time Zone (DST) settings, change to your country
TimeChangeRule usPST = { "PST", Second, Sun, Mar, 2, -420 }; // US Pacific Time
TimeChangeRule usPDT =  { "PDT ", First, Sun, Nov, 2, -480  }; // US Pacific Time
Timezone ClockTZ(usPST, usPDT);


#define PIN_CH00 12     // motor controller
#define PIN_CH01 13     // motor controller
#define PIN_INIT 14     // button pin

unsigned int localPort = 8888;
WiFiUDP udp;
static const char hname[] = "esp-clock-controller";

/*
    Slave clock configuration

    I been not able to find specification for this clock, so experimentally
    If impulse is > 200ms clock sometime fails to move the arrow on next switch

    IMPULSE_WAIT is used in the INIT mode and if slave catching up master
*/
#define IMPULSE_ON 200
#define IMPULSE_WAIT 800

short state = 0;
time_t last_ntp_sync = 0;

char console_text[256];
Preferences preferences;

long debounceDelay = 80;    // the debounce eelay
long lastDebounceTime = 0;  // the last time the output pin was toggled

void setup() {
  /* Read slave clock state from the EEPROM using Preferences lib.
     We have 12 * 60 (720) possible values
     in the clock and also we need to keep last used polarity as a sign
     This mean that valid values are from -721 to +721 excluding 0
  */
  Serial.begin(115200);
  Serial.println();
  Serial.println("*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*");
  Serial.println("Serial Begin OK");

  pinMode(PIN_CH00, OUTPUT); 				// init GPIO to control clock motor
  pinMode(PIN_CH01, OUTPUT);
  pinMode(PIN_INIT, INPUT_PULLUP); 	// clock reset to 12:00 button
  
  preferences.begin("clock", false);
  state = preferences.getShort("state", -1);
  Serial.print("Booting... Initial state is: ");
  Serial.println(state);
  if (state < -721 || state > 721 || state == 0) {  // if state is silly
    state = 1;                                      // init clock on 12:00
    preferences.putShort("state", state);           // and store state
  }

  int initState = digitalRead(PIN_INIT);
  Serial.print("***Init state: ");
  Serial.println(initState);
  

  // if init mode is on - state is set to 12:00 and pin must be unplugged when
  // clock is displaying this value. State values are inverted because 
  // INPUT_PULLUP
  if (initState == 1) {
    Serial.println("Init LOW. Skipping clock reset mode.");
  }
  if (initState == 0) {
    Serial.println("Init HIGH. Going into clock reset mode.");
    state = 1;
  }
  while (initState == 0) {
    // move minute hand once per second
    digitalWrite(PIN_CH00, HIGH);
    digitalWrite(PIN_CH01, LOW);
    delay(IMPULSE_ON);
    Serial.println("tick");
    digitalWrite(PIN_CH00, LOW);
    digitalWrite(PIN_CH01, LOW);
    delay(IMPULSE_WAIT);
    Serial.println("tock");
    preferences.putShort("state", state);
    initState = digitalRead(PIN_INIT);
  }
  // workaround for the ESP32 SDK bug, see 
  // https://github.com/espressif/arduino-esp32/issues/2537#issuecomment-508558849
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  // set hostname
  WiFi.setHostname(hname);
  // connect to wifi
  sprintf(console_text, "Connecting to wifi (%s)", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_KEY);

  while (WiFi.status() != WL_CONNECTED) {
    delay(10);
  }
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP().toString().c_str());

  Serial.println("Starting UDP...");
  udp.begin(localPort);
  Serial.println("Waiting for sync");

  setSyncProvider(getNtpTime);
  setSyncInterval(NTP_SYNC_INTERVAL); // sync with NTP
  while (timeStatus() == timeNotSet) {
    delay(10);
  }
}

/*-------- Move minute hand and update state ----------*/

void advanceClock(short curr_state) {
  char buf[16], buf2[16];
  Serial.print("Changing state from: ");
  Serial.print(state);
  Serial.print(" -> ");
  Serial.print(formatState(abs(state), buf, 16));
  Serial.print(" to ");
  Serial.print(curr_state);
  Serial.print(" -> ");
  Serial.println(formatState(curr_state, buf2, 16));
  
  // this should never happens. If clock is behind NTP to up to 5m - do nothing, just wait
  if (abs(state) > curr_state && (abs(state) - curr_state) <= 5) {
    Serial.print("Clock is behind NTP for ");
    Serial.print((int)(abs(state) - curr_state));
    Serial.println(" minutes. Ignoring");
    return;
  }
  state++;
  if (state >= 721) state = 1;
  digitalWrite(PIN_CH00, HIGH);
  digitalWrite(PIN_CH01, LOW);
  delay(IMPULSE_ON);
  digitalWrite(PIN_CH00, LOW);
  digitalWrite(PIN_CH01, LOW);
  preferences.putShort("state", state);
}

// convert state variable to the human-readable format
char * formatState(int mystate, char * buf, int bufsize) {
  mystate--;
  snprintf(buf, bufsize, "%2d:%02d", (mystate / 60) ? mystate / 60 : 12, mystate - (mystate / 60) * 60);
  return buf;
}


/*-------- Main loop ----------*/
void loop() {

  int buttonState = digitalRead(PIN_INIT);

    if ((millis() - lastDebounceTime) > debounceDelay) {
      // pressing the button will advance us one minute
      // good for if the clock misses a tick (mine is very old!)
      if ((buttonState == 0) && (PIN_CH00 == LOW)) {
        digitalWrite(PIN_CH00, HIGH);
        digitalWrite(PIN_CH01, LOW);
        lastDebounceTime = millis(); //set the current time
        Serial.println("Advanced one minute by operation of key.");
      }
      else if ((buttonState == 1) && (PIN_CH00 == HIGH)) {
        digitalWrite(PIN_CH00, LOW);
        digitalWrite(PIN_CH01, LOW);
        lastDebounceTime = millis();
      }
    }
  
  time_t utc = now();
  time_t local_t = ClockTZ.toLocal(utc);
  int hour_12 = hour(local_t);
  if (hour_12 >= 12) hour_12 -= 12;
  // current 12h time in minutes, starting from 1
  short curr_state = hour_12 * 60 + minute(local_t) + 1;

  if (curr_state != abs(state)) {
    advanceClock(curr_state);
    delay(IMPULSE_WAIT); // cool down device :)
  }
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime() {
  IPAddress ntpServerIP; // NTP server's ip address

  while (udp.parsePacket() > 0); // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(" / ");
  Serial.println(ntpServerIP.toString().c_str());
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      udp.read(packetBuffer, NTP_PACKET_SIZE); // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 = (unsigned long) packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long) packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long) packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long) packetBuffer[43];
      last_ntp_sync = secsSince1900 - 2208988800UL;
      return secsSince1900 - 2208988800UL;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress & address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0; // Stratum, or type of clock
  packetBuffer[2] = 6; // Polling Interval
  packetBuffer[3] = 0xEC; // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting the time:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
