/* *******************************************************************
   WiFi and serial interface functions

   startSerial
   - starts HW serial interface which we use for RS485 line
   - calculates Modbus RTU character timeout and frame delay

   startEthernet
   - initiates WiFi interface
   - gets IP from DHCP
   - starts all servers (Modbus TCP, UDP, web server)

   resetFunc
   - well... resets Arduino

   maintainUptime
   - maintains up time in case of millis() overflow

   maintainCounters
   - synchronizes roll-over of data counters to zero

   ***************************************************************** */


void startSerial() {
  mySerial.begin(defaultConfig.baud, defaultConfig.serialConfig);
  // Calculate Modbus RTU character timeout and frame delay
  byte bits =                                         // number of bits per character (11 in default Modbus RTU settings)
    1 +                                               // start bit
    (((defaultConfig.serialConfig & 0x06) >> 1) + 5) +  // data bits
    (((defaultConfig.serialConfig & 0x08) >> 3) + 1);   // stop bits
  if (((defaultConfig.serialConfig & 0x30) >> 4) > 1) bits += 1;    // parity bit (if present)
  int T = ((unsigned long)bits * 1000000UL) / defaultConfig.baud;       // time to send 1 character over serial in microseconds
  if (defaultConfig.baud <= 19200) {
    charTimeout = 1.5 * T;         // inter-character time-out should be 1,5T
    frameDelay = 3.5 * T;         // inter-frame delay should be 3,5T
  }
  else {
    charTimeout = 750;
    frameDelay = 1750;
  }
#ifdef RS485_CONTROL_PIN
  pinMode(RS485_CONTROL_PIN, OUTPUT);
  digitalWrite(RS485_CONTROL_PIN, RS485_RECEIVE);  // Init Transceiver
#endif /* RS485_CONTROL_PIN */
}

void startNetwork() {
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);
  
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    dbgln("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    dbg(F("[arduino] Attempting to connect to SSID: "));
    dbgln(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  
  dbgln(F("[arduino] Connected to wifi"));
//  printWiFiStatus();
  
  // start the WiFi OTA library with internal (flash) based storage
  WiFiOTA.begin("Arduino", "password", InternalStorage);

  modbusServer.begin();
  webServer.begin();
  udpServer.begin(defaultConfig.udpPort);
  
  dbg(F("[arduino] Server available at http://"));
  dbgln(WiFi.localIP());
}

void(* resetFunc) (void) = 0;   //declare reset function at address 0

void maintainUptime()
{
  unsigned long milliseconds = millis();
  if (last_milliseconds > milliseconds) {
    //in case of millis() overflow, store existing passed seconds
    remaining_seconds = seconds;
  }
  //store last millis(), so that we can detect on the next call
  //if there is a millis() overflow ( millis() returns 0 )
  last_milliseconds = milliseconds;
  //In case of overflow, the "remaining_seconds" variable contains seconds counted before the overflow.
  //We add the "remaining_seconds", so that we can continue measuring the time passed from the last boot of the device.
  seconds = (milliseconds / 1000) + remaining_seconds;
}

void maintainCounters()
{
  // synchronize roll-over of data counters to zero, at 0xFFFFFF00 or 0xFF00 respectively
#ifdef ENABLE_EXTRA_DIAG
  const unsigned long rollover = 0xFFFFFF00;
#else
  const unsigned int rollover = 0xFF00;
#endif /* ENABLE_EXTRA_DIAG */
  if (serialTxCount > rollover || serialRxCount > rollover || ethTxCount > rollover || ethRxCount > rollover) {
    serialRxCount = 0;
    serialTxCount = 0;
    ethRxCount = 0;
    ethTxCount = 0;
  }
}

#define BOARD F("Adafruit Feather M0 WiFi")
