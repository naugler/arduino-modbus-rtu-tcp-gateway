/* Arduino-based Modbus RTU (slaves) to Modbus TCP/UDP (master) gateway with web interface
   - slaves are connected via RS485 interface
   - master(s) are connected via wifi interface
   - up to 247 Modbus RTU slaves
   - up to 8 TCP/UDP sockets for Modbus TCP/UDP masters and web interface
   - RS485 interface protocols:
            o Modbus RTU
   - Ethernet interface protocols:
            o Modbus TCP
            o Modbus UDP
            o Modbus RTU over TCP
            o Modbus RTU over UDP
   - supports broadcast (slave address 0x00) and error codes
   - supports all Modbus function codes
   - optimized queue for Modbus requests
            o prioritization of requests to responding slaves
            o queue will accept only one requests to non-responding slaves
   - supports OTA sketch updates

  Version history
  v0.1 2020-04-05 Initial commit
  v0.2 2021-03-02 Random MAC generation
  v1.0 2021-03-20 Add web interface, settings stored in EEPROM
  v2.0 2021-04-01 Improve random MAC algorithm (Marsaglia algorithm from https://github.com/RobTillaart/randomHelpers),
                  replace some libraries with more efficient code, compatibility with Arduino Mega
  v2.1 2021-04-12 Code optimisation
  v2.2 2021-06-06 Fix TCP closed socket, support RS485 modules with hardware automatic flow control
  v2.3 2021-09-10 Fix IPAddress cast (gateway freeze)
  v2.4 2021-10-15 Add SW version. Forced factory reset (load defaut settings from sketch) on MAJOR version change.
  v3.0 2021-11-07 Improve POST parameters processing, bugfix 404 and 204 error headers. 
  v3.1 2022-01-28 Code optimization, bugfix DHCP settings.
  v3.2 2022-06-04 Reduce program size (so that it fits on Nano), ethernet data counter only available when ENABLE_EXTRA_DIAG.
  v3.3b 2022-12-02 Modified to run on Adafruit Feather M0 WiFi, add OTA sketch updates, EEPROM removed

*/

const byte version[] = {3, 2};

#include <SPI.h>
#include <WiFi101.h>
#include <WiFi101OTA.h>
#include <CircularBuffer.h>     // CircularBuffer https://github.com/rlogiacco/CircularBuffer
#include <StreamLib.h>          // StreamLib https://github.com/jandrassy/StreamLib

#include <avr/interrupt.h>

/****** ADVANCED SETTINGS ******/

const byte reqQueueCount = 15;       // max number of TCP or UDP requests stored in queue
const int reqQueueSize = 256;        // total length of TCP or UDP requests stored in queue (in bytes)
const byte maxSlaves = 247;          // max number of Modbus slaves (Modbus supports up to 247 slaves, the rest is for reserved addresses)
const int modbusSize = 256;          // size of a MODBUS RTU frame (determines size of serialInBuffer and tcpInBuffer)
#define mySerial Serial1              // define serial port for RS485 interface, for Arduino Mega choose from Serial1, Serial2 or Serial3         
const byte scanCommand[] = {0x03, 0x00, 0x00, 0x00, 0x01};  // Command sent during Modbus RTU Scan. Slave is detected if any response (even error) is received.

// #define DEBUG            // Main Serial (USB) is used for printing some debug info, not for Modbus RTU. At the moment, only web server related debug messages are printed.
#define debugSerial Serial
String logline = "none";

//#ifdef MAX_SOCK_NUM           //if the macro MAX_SOCK_NUM is defined 
// #undef MAX_SOCK_NUM           //un-define it
 #define MAX_SOCK_NUM 8        //redefine it with the new value
//#endif 

/****** EXTRA FUNCTIONS ******/

 #define ENABLE_EXTRA_DIAG      // Enable per socket diagnostics, run time counter

/****** DEFAULT FACTORY SETTINGS ******/

typedef struct {
  unsigned int tcpPort;
  unsigned int udpPort;
  unsigned int webPort;
  bool enableRtuOverTcp;
  unsigned long baud;
  unsigned long serialConfig;
  unsigned int serialTimeout;
  byte serialRetry;
} config_type;

const config_type defaultConfig = {
  502,                   // tcpPort
  502,                   // udpPort
  80,                    // webPort
  false,                  // enableRtuOverTcp
  38400,                 // baud
  SERIAL_8N2,            // serialConfig (Modbus RTU default is 8E1, another frequently used option is 8N2)
  500,                   // serialTimeout
  5                      // serialRetry
};

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "CooksieMonster";        // your network SSID (name)
char pass[] = "sesamestreet";    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;

/****** ETHERNET AND SERIAL ******/

#ifdef UDP_TX_PACKET_MAX_SIZE               //if the macro MAX_SOCK_NUM is defined 
#undef UDP_TX_PACKET_MAX_SIZE               //un-define it
#define UDP_TX_PACKET_MAX_SIZE modbusSize   //redefine it with the new value
#endif 

byte maxSockNum = MAX_SOCK_NUM;

WiFiUDP udpServer;
WiFiServer modbusServer(defaultConfig.tcpPort);
WiFiServer webServer(defaultConfig.webPort);
WiFiClient currentClient;
#ifdef DEBUG
#define dbg(x...) debugSerial.print(x);
#define dbgln(x...) debugSerial.println(x);
#else /* DEBUG */
#define dbg(x...) ;
#define dbgln(x...) ;
#endif /* DEBUG */
#define UDP_REQUEST 0xFF      // We store these codes in "header.clientNum" in order to differentiate 
#define SCAN_REQUEST 0xFE      // between TCP requests (their clientNum is nevew higher than 0x07), UDP requests and scan requests (triggered by scan button)

/****** TIMERS AND STATE MACHINE ******/

class MicroTimer {
  private:
    unsigned long timestampLastHitMs;
    unsigned long sleepTimeMs;
  public:
    boolean isOver();
    void sleep(unsigned long sleepTimeMs);
};
boolean MicroTimer::isOver() {
  if ((unsigned long)(micros() - timestampLastHitMs) > sleepTimeMs) {
    return true;
  }
  return false;
}
void MicroTimer::sleep(unsigned long sleepTimeMs) {
  this->sleepTimeMs = sleepTimeMs;
  timestampLastHitMs = micros();
}
class Timer {
  private:
    unsigned long timestampLastHitMs;
    unsigned long sleepTimeMs;
  public:
    boolean isOver();
    void sleep(unsigned long sleepTimeMs);
};
boolean Timer::isOver() {
  if ((unsigned long)(millis() - timestampLastHitMs) > sleepTimeMs) {
    return true;
  }
  return false;
}
void Timer::sleep(unsigned long sleepTimeMs) {
  this->sleepTimeMs = sleepTimeMs;
  timestampLastHitMs = millis();
}
Timer requestTimeout;
uint16_t crc;
#define RS485_TRANSMIT    HIGH
#define RS485_RECEIVE     LOW
byte scanCounter = 0;
enum state : byte
{
  IDLE, SENDING, DELAY, WAITING
};
enum state serialState;
unsigned int charTimeout;
unsigned int frameDelay;

/****** RUN TIME AND DATA COUNTERS ******/

volatile uint32_t seed1;  // seed1 is generated by CreateTrulyRandomSeed()
volatile int8_t nrot;
uint32_t seed2 = 17111989;   // seed2 is static

// store uptime seconds (includes seconds counted before millis() overflow)
unsigned long seconds;
// store last millis() so that we can detect millis() overflow
unsigned long last_milliseconds = 0;
// store seconds passed until the moment of the overflow so that we can add them to "seconds" on the next call
unsigned long remaining_seconds = 0;
// Data counters
unsigned long serialTxCount = 0;
unsigned long serialRxCount = 0;
unsigned long ethTxCount = 0;
unsigned long ethRxCount = 0;

/****** SETUP: RUNS ONCE ******/

void setup() {

#ifdef DEBUG
  debugSerial.begin(localConfig.baud);    // same baud as RS485
#endif /* DEBUG */

  startSerial();
  startNetwork();
  dbgln(F("\n[arduino] Starting..."));
}

/****** LOOP ******/

void loop() {
  // check for WiFi OTA updates
  WiFiOTA.poll();
  
  recvUdp();
  recvTcp();
  processRequests();

#ifndef DEBUG
  sendSerial();
  recvSerial();
#endif /* DEBUG */

  recvWeb();
  maintainCounters();   // maintain counters and synchronize their reset to zero when they overflow
  maintainUptime();    // maintain uptime in case of millis() overflow
}
