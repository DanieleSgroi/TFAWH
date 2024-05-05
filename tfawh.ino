/**************************************************************************************************************************************
**
** Project: TFA Weather Hub fake sensor emulator
** File:    TFAWH.INO
** Purpose: Main SW File
** 
** (C) 2019 by Daniele Sgroi - daniele.sgroi@gmail.com
**
** VERSION:
**  - May 05, 2024 - V1.0 - D. Sgroi
**
** TARGET HW:
** - Teensy 4.1 with Ethernet Shield
**
** SOFTWARE:
** - Arduino 2.3.2+ IDE
**
** NOTES:
** 
** Inspired by https://github.com/sarnau/MMMMobileAlerts , Written my Markus Fritze, 2018.
**
** Emulates a Technoline MA10100 temperature sensor to understand data upload on WeatherHub cloud service
** 
** This code is in the public domain.
**
**************************************************************************************************************************************/

#define DEBUG                  0  // enable debug without connect to server

/**************************************************************************************************************************************
** DEFINES
**************************************************************************************************************************************/

#define DST_PORT              80  // UDP Destination www.data199.com:443
#define NTP_PACKET_SIZE       48  // NTP time stamp is in the first 48 bytes of the message
#define NTP_PORT            8888  // NTP UDP Port

/**************************************************************************************************************************************
** INCLUDES
**************************************************************************************************************************************/

#include <_Teensy.h>
#include <TimeLib.h>
#include <SPI.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h> 

/**************************************************************************************************************************************
** GLOBALS
**************************************************************************************************************************************/

// TODO: Enter a MAC address for your controller below.
byte mac[] = { 0x0E, 0x01, 0x02, 0x03, 0x04, 0x05 }; 

// TODO resolve DNS for NTP Server
IPAddress timeServer(78,41,116,149); // europe.pool.ntp.org

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

EthernetClient client;

// TODO: customize 0xXX with sensor mac address
static uint8_t mobileAlertPacket[64] = {0xCE,0x66,0x32,0x0C,0x72,0x12,0x02,0xXX,0xXX,0xXX,0xXX,0xXX,0x00,0x01,0x00,0x00,0x00,0x00,0x1A,000,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x51} ; // Sensor Binary Payload
int iCurTmp = 0, iPreTmp = 0;
unsigned char ucTxCnt = 0;
unsigned long ulTimeStamp;
char checksum = 0;

/**************************************************************************************************************************************
** getNtpTime
**
** dunno
**
**************************************************************************************************************************************/
time_t getNtpTime() {

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      //return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
      return secsSince1900 - 2208988800UL;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

/**************************************************************************************************************************************
** sendNTPpacket
**
** send an NTP request to the time server at the given address
**
**************************************************************************************************************************************/

void sendNTPpacket(IPAddress &address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

/**************************************************************************************************************************************
** printIPAddress
**
** Print IP Address in use
**
**************************************************************************************************************************************/

void printIPAddress() {

  Serial.print(F("Local IP address: "));
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    if(thisByte < 3)
      Serial.print('.');
  }
  Serial.println();

} // printIPAddress

/**************************************************************************************************************************************
** printDigits
**
** utility for digital clock display: prints preceding colon and leading 0
**
**************************************************************************************************************************************/

void printDigits(int digits){

  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);

} // printDigits

/**************************************************************************************************************************************
** digitalClockDisplay
**
** digital clock display of the time
**
**************************************************************************************************************************************/

void digitalClockDisplay(){

  Serial.print("UTC ");
  Serial.print(hour());
  Serial.print(":");
  printDigits(minute());
  Serial.print(":");
  printDigits(second());
  Serial.print(" - ");
  printDigits(day());
  Serial.print("/");
  printDigits(month());
  Serial.print("/");
  printDigits(year()); 
  Serial.println(); 

} // digitalClockDisplay

/**************************************************************************************************************************************
** setup
**
** run once at startup
**
**************************************************************************************************************************************/

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on

  Serial.begin(115200);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only
  Serial.println(F("TFAWH - V1.0 - (C)2024 by Daniele Sgroi"));
  Serial.println(F("Booting..."));

    // start Ethernet and UDP
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet Interface not found.");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so hang on:
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off
    while(true);
  }

  printIPAddress();

  Udp.begin(NTP_PORT);
  Serial.println(F("Binding NTP..."));
  setSyncProvider(getNtpTime);
  digitalClockDisplay();
        
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off

  iCurTmp = 220; // 22.0 °C
  ucTxCnt = 0;

} // setup

/**************************************************************************************************************************************
** LOOP
**
** Run continuosly
**
**************************************************************************************************************************************/

void loop() {

  digitalWrite(LED_BUILTIN, HIGH);

  iPreTmp = iCurTmp;
  iCurTmp += 1;
  if (iCurTmp > 400) {
    iCurTmp = -400;
  }
  ucTxCnt++;
  ulTimeStamp = now();
  Serial.print(F("NOW "));
  Serial.println(ulTimeStamp, HEX);
  Serial.print(F("Tmp = "));
  Serial.print(iCurTmp);
  Serial.print(" ");
  Serial.println(iCurTmp, HEX);
  Serial.print(F("Txc = "));
  Serial.println(ucTxCnt);

// Offset       00 01    04 05 06        11 1213 1415 1617 18 19    22 23                                                                            62 63
// HEX package: CE 66320C72 12 02XXXXXXXXXX 0002 00DE 00DE 1B B50A4D1A 00000000000000000000000000000000000000000000000000000000000000000000000000000000 51
//              |  |        |  |            |    |    |    |          |                                                                                |
// header ------/  |        |  |            |    |    |    |          \--- Pad                                                   checksum -------------/
// nix timestamp --/        |  |            |    |    |    \-------------- Unknown (1A/1B)
// pkg lenght --------------/  |            |    |    \------------------- Previous Temperature
// sensor id ------------------/            |    \------------------------ Current Temperature
// tx counter ------------------------------/   

  mobileAlertPacket[1] = (ulTimeStamp >> 24) & 0xff;
  mobileAlertPacket[2] = (ulTimeStamp >> 16) & 0xff;
  mobileAlertPacket[3] = (ulTimeStamp >> 8) & 0xff;
  mobileAlertPacket[4] = (ulTimeStamp) & 0xff;
  mobileAlertPacket[13] = ucTxCnt;
  mobileAlertPacket[14] = (unsigned char) (iCurTmp >> 8) & 0xff; 
  mobileAlertPacket[15] = (unsigned char) (iCurTmp) & 0xff; 
  mobileAlertPacket[14] = (unsigned char) (iPreTmp >> 8) & 0xff; 
  mobileAlertPacket[15] = (unsigned char) (iPreTmp) & 0xff; 

  checksum = 0;    
  for(int index=0; index<63; ++index) {
    checksum += mobileAlertPacket[index];
  }
  mobileAlertPacket[63] = checksum; 

  Serial.print(F("HEX package: "));
  for(int index=0; index<64; index++) {
    if (mobileAlertPacket[index] < 0x0f) {
      Serial.print(F("0"));
    }
      Serial.print(mobileAlertPacket[index], HEX);
  }
  Serial.println();

  if (client.connect("www.data199.com", DST_PORT)) {  
    Serial.print("Connected to ");
    Serial.println(client.remoteIP());

    client.println("PUT /gateway/put HTTP/1.1");
    client.println("Host: www.data199.com");
    client.println("Connection: close");
    client.println("HTTP_IDENTIFY: 80ABCDEF:006789ABCDEF:C0"); // TODO: customize with gateway MAC address and Serial Number
    client.println("Content-Type: application/octet-stream");
    client.println("Content-Length: 64");
    client.println(); // CRLF between header and binary data packet
    client.write(mobileAlertPacket, 64);

    delay (1000); 
    
    // Print server answer
    int len = client.available();
    if (len > 0) {
      byte buffer[256];
      if (len > 256) len = 256;
      client.read(buffer, len);
      Serial.write(buffer, 16); // only print first 16 chars of answer
    }
    else {
      Serial.println("Answer not Available.");
    }

    client.stop();

  }
  else {
    Serial.println("Connection Failed.");  
  }

  digitalWrite(LED_BUILTIN, LOW);

  delay (60*1000*6); // upload every 6 minutes

} // loop

/**************************************************************************************************************************************
** EOF
**************************************************************************************************************************************/