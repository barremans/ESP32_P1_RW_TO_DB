/* ESP32 reading Belgium P1 smart meter and send to Mysql database
Optional sending to domoticz

Created by Barremans
https://github.com/barremans/ESP32_P1_RW_TO_DB
*/

#define SKETCH_VERSION "1.1.2"

#include <TimedBlink.h>  //led control without using delay

#include <HTTPClient.h>
#include "CRC16.h"  // check message
//#include <WiFi.h> //not needed for the esp32

///ESP32
#include <HardwareSerial.h>
HardwareSerial mySerial(1);
#define RXD2 16  // pin for RX2 on ESP32
#define TXD2 17  // pin for tTX2 on ESP32
//static int taskCore = 0; //BB10092021 core handeling


#include "wifi_credentials.h"
#include "led_functions.h"


#define VERSION F("1.1.2 ")


//===Change values from here===
const char* domoticzIP = "***.***.*.***";  // optional to send domoticz ip ex. "192.168.0.136"
const int domoticzPort = ****;             // optional domoticz port ex. 8855   
const int domoticzGasIdx = 12;             // domoticz IDX 12 for P1 Gas
const int domoticzEneryIdx = 24;           // domoticz IDX 24 for P1 electricity
const bool outputOnSerial = true;

char HOST_NAME[] = "***************";   // website for sending data to db
char PATH_NAME[] = "*************";   // website folder for sending data to db
String SENSOR_NAME = "";
//===Change values to here===


// LED SETTINGS ////////////////
// constants won't change. Used here to set a pin number:
LED_Controls RUNLED(32, 500);  // led pin nbr en delay time RUN LED
TimedBlink monitor(32);        // led pin nbr
//LED_Controls ERRORLED(33, 500);  // led pin nbr en delay time error LED



// Vars to store meter readings

long mEVLT = 0;    //Meter reading Electrics - Elektra verbruik dagtarief (Fluvius) - Totale afname van energie in kWh dagtarief
long mEVHT = 0;    //Meter reading Electrics - Elektra verbruik nachttarief (Fluvius) - Totale afname van energie in kWh nachttarief
long mEOLT = 0;    //Meter reading Electrics - Elektra opbrengst dagtarief (Fluvius) - Totale injectie van energie in kWh dagtarief
long mEOHT = 0;    //Meter reading Electrics - Elektra opbrengst nachttarief (Fluvius) - Totale injectie van energie in kWh nachttarief
long mEAV = 0;     //Meter reading Electrics - Actueel verbruik (Fluvius) - Afgenomen ogenblikkelijk vermogen in kW
long mEAT = 0;     //Meter reading Electrics - Actuele teruglevering (Fluvius) - Geïnjecteerd ogenblikkelijk vermogen in kW
long mGAS = 0;     //Meter reading Gas
long mEVT = 0;     //1.8.0 Meter afname totaal reading mEVLT + mEVHT
long mEOT = 0;     //1.8.0 Meter afname totaal reading mEOLT + mEOHT
long mEOVOL1 = 0;  //32.7.0(232.0 V) Ogenblikkelijke spanning in fase 1 in V Fluvius
long mEOVOL2 = 0;  //52.7.0(232.0 V) Ogenblikkelijke spanning in fase 2 in V Fluvius
long mEOVOL3 = 0;  //72.7.0(232.0 V) Ogenblikkelijke spanning in fase 3 in V Fluvius
long mEOPL1 = 0;  //1-0:31.7.0(11.2 A) Ogenblikkelijke stroom door fase 1 in A Fluvius
long mEOPL2 = 0;  //1-0:51.7.0(11.2 A) Ogenblikkelijke stroom door fase 2 in A Fluvius
long mEOPL3 = 0;  //1-0:71.7.0(11.2 A) Ogenblikkelijke stroom door fase 3 in A Fluvius
long prevGAS = 0;

/*
  ////TEST ONLY !!!!!!
  long TmEVLT = 1266365;    //Meter reading Electrics - Elektra verbruik dagtarief (Fluvius) - Totale afname van energie in kWh dagtarief
  long TmEVHT = 6987266;    //Meter reading Electrics - Elektra verbruik nachttarief (Fluvius) - Totale afname van energie in kWh nachttarief
  long TmEOLT = 6580159;    //Meter reading Electrics - Elektra opbrengst dagtarief (Fluvius) - Totale injectie van energie in kWh dagtarief
  long TmEOHT = 9123457;    //Meter reading Electrics - Elektra opbrengst nachttarief (Fluvius) - Totale injectie van energie in kWh nachttarief
  long TmEAV = 9456754;     //Meter reading Electrics - Actueel verbruik (Fluvius) - Afgenomen ogenblikkelijk vermogen in kW
  long TmEAT =  9874556;     //Meter reading Electrics - Actuele teruglevering (Fluvius) - Geïnjecteerd ogenblikkelijk vermogen in kW
  long TmGAS = 2569.23;     //Meter reading Gas
  long TmEVT = 0;     //1.8.0 Meter afname totaal reading mEVLT + mEVHT
  long TmEOT = 0;     //1.8.0 Meter afname totaal reading mEOLT + mEOHT
  long TmEOVOL1 = 237.0;  //32.7.0(232.0 V) Ogenblikkelijke spanning in fase 1 in V Fluvius
  long TmEOVOL2 = 240.0;  //52.7.0(232.0 V) Ogenblikkelijke spanning in fase 2 in V Fluvius
  long TmEOVOL3 = 323.0;  //72.7.0(232.0 V) Ogenblikkelijke spanning in fase 3 in V Fluvius
  long TmEOP1 = 900;  //1-0:31.7.0(11.2 A) Ogenblikkelijke stroom door fase 1 in A Fluvius
  long TmEOP2 = 899;  //1-0:51.7.0(11.2 A) Ogenblikkelijke stroom door fase 2 in A Fluvius
  long TmEOP3 = 900;  //1-0:71.7.0(11.2 A) Ogenblikkelijke stroom door fase 3 in A Fluvius
  long TprevGAS = 0;
  ////TEST
*/

#define MAXLINELENGTH 64  // longest normal line is 47 char (+3 for \r\n\0)
char telegram[MAXLINELENGTH];


unsigned int currentCRC = 0;
int CRCerror = 0;  // if 5 error 5 times in a row than reboot

void SendToDomoLog(char* message) {
  char url[512];
  sprintf(url, "http://%s:%d/json.htm?type=command&param=addlogmessage&message=%s", domoticzIP, domoticzPort, message);
}



void setup() {
  Serial.begin(115200);
  Serial.print(F("Test Sketch "));
  Serial.println(VERSION);


  // delete old config
  WiFi.disconnect(true);

  delay(1000);

  WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);

  /* Remove WiFi event
    Serial.print("WiFi Event ID: ");
    Serial.println(eventID);
    WiFi.removeEvent(eventID);*/


  Serial.println();
  Serial.println();
  Serial.println("Wait for WiFi... ");

  initWiFi();
  Serial.print("RRSI: ");
  Serial.println(WiFi.RSSI());

  mySerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  ;  // (RX, TX. inverted, buffer)
}

/*
  void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    monitor.blink(250, 250);  // Call this as often as possible
    Serial.println("failed!!! !");
    RUNLED.OFF();
    WiFi.reconnect();
  }
  /// normal logica /////////////////
  //  readTelegramTEST();
  RUNLED.ESP_RUN(); // if run then led
  readTelegram();  // to domotics
  }
*/

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    RUNLED.ESP_RUN(); // if run then led
    readTelegram();  // to domotics
  }
  else {
    Serial.println("failed!!! !");
    RUNLED.OFF();
    WiFi.reconnect();
  }
}



//FUNCTIONS//////////////////
//WIFI FUNCTIONS////////////
void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // RUNLED.ESP_RUN();
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.disconnected.reason);
  Serial.println("Trying to Reconnect");
  monitor.blink(150, 150);  // Call this as often as possible
  WiFi.disconnect();
  WiFi.begin(MY_SSID, MY_PASS);
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(APNAME);  //define hostname
  WiFi.begin(MY_SSID, MY_PASS);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    monitor.blink(1000, 1000);  // Call this as often as possible
    Serial.print('.');
  }
}



//P1 FUNCTIONS///////////////
/*
  ///////////TEST
  void UpdateGasTEST() {
  //sends over the gas setting to domoticz

  char sValue[12];
  sprintf(sValue, "%d", TmGAS);
  char SENSOR_NAME[] = "Gas";
  if (SendToDomo(SENSOR_NAME,domoticzGasIdx, 0, sValue ))
    TmGAS += 1;
  TprevGAS = TmGAS;  //not needed for test
  }
  //////////
*/

void UpdateGas() {
  //sends over the gas setting to domoticz
  if (prevGAS != mGAS) {
    char sValue[12];  // was 10 these let the esp32 crashes
    sprintf(sValue, "%f", mGAS / 1000.0);
    char SENSOR_NAME[] = "Gas";
    if (SendToDomo(SENSOR_NAME, domoticzGasIdx, 0, sValue ))
      prevGAS = mGAS;
  }
}
/*
  ///////////TEST
  void UpdateElectricityTEST() {

  char sValue[255];
  sprintf(sValue, "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d", TmEVLT, TmEVHT, TmEOLT, TmEOHT, TmEAV, TmEAT, TmEOVOL1, TmEOVOL2, TmEOVOL3, TmEOP1, TmEOP2, TmEOP3);
  char SENSOR_NAME[] = "Power";
  SendToDomo(SENSOR_NAME,domoticzEneryIdx, 0, sValue );
  TmEVLT += 1;
  TmEVHT += 1;
  TmEOLT += 1;
  TmEOHT += 1;
  TmEAV += 1;
  TmEAT += 1;
  }
  //////////
*/

void UpdateElectricity() {
  char sValue[255];
  // sprintf(sValue, "%d;%d;%d;%d;%d;%d", mEVLT, mEVHT, mEOLT, mEOHT, mEAV, mEAT); //OLD
  //sprintf(sValue, "%f;%f;%f;%f;%d;%d", mEVLT / 1000.0, mEVHT / 1000.0, mEOLT / 1000.0, mEOHT / 1000.0, mEAV, mEAT);  //maybe not relevant, but removing the float from first try and dividing by 1000.0 generates less "Invalid CRC found" error messages...
  sprintf(sValue, "%f;%f;%f;%f;%d;%d;%f;%f;%f;%f;%f;%f", mEVLT / 1000.0, mEVHT / 1000.0, mEOLT / 1000.0, mEOHT / 1000.0, mEAV, mEAT, mEOVOL1 / 1000.0, mEOVOL2 / 1000.0, mEOVOL3 / 1000.0, mEOPL1 / 1000.0, mEOPL2 / 1000.0, mEOPL3 / 1000.0);  //new with voltage and current  dividing by 1000.0 generates less "Invalid CRC found" error messages...
  char SENSOR_NAME [] = "Power";
  SendToDomo(SENSOR_NAME, domoticzEneryIdx, 0, sValue );
}


bool isNumber(char* res, int len) {
  for (int i = 0; i < len; i++) {
    if (((res[i] < '0') || (res[i] > '9')) && (res[i] != '.' && res[i] != 0)) {
      return false;
    }
  }
  return true;
}

int FindCharInArrayRev(char array[], char c, int len) {
  for (int i = len - 1; i >= 0; i--) {
    if (array[i] == c) {
      return i;
    }
  }
  return -1;
}

long getValidVal(long valNew, long valOld, long maxDiffer) {
  //check if the incoming value is valid
  if (valOld > 0 && ((valNew - valOld > maxDiffer) && (valOld - valNew > maxDiffer)))
    return valOld;
  return valNew;
}

long getValue(char* buffer, int maxlen) {
  int s = FindCharInArrayRev(buffer, '(', maxlen - 2);
  if (s < 8) return 0;
  if (s > 32) s = 32;
  int l = FindCharInArrayRev(buffer, '*', maxlen - 2) - s - 1;
  if (l < 4) return 0;
  if (l > 12) return 0;
  char res[16];
  memset(res, 0, sizeof(res));

  if (strncpy(res, buffer + s + 1, l)) {
    if (isNumber(res, l)) {
      return (1000 * atof(res));
    }
  }
  return 0;
}


bool decodeTelegram(int len) {
  //need to check for start
  int startChar = FindCharInArrayRev(telegram, '/', len);
  int endChar = FindCharInArrayRev(telegram, '!', len);
  bool validCRCFound = false;
  if (startChar >= 0) {
    //start found. Reset CRC calculation
    currentCRC = CRC16(0x0000, (unsigned char*)telegram + startChar, len - startChar);
    if (outputOnSerial) {
      for (int cnt = startChar; cnt < len - startChar; cnt++)
        Serial.print(telegram[cnt]);
    }
    //Serial.println("Start found!");

  } else if (endChar >= 0) {
    //add to crc calc
    currentCRC = CRC16(currentCRC, (unsigned char*)telegram + endChar, 1);
    char messageCRC[5];
    strncpy(messageCRC, telegram + endChar + 1, 4);
    messageCRC[4] = 0;  //thanks to HarmOtten (issue 5)
    if (outputOnSerial) {
      for (int cnt = 0; cnt < len; cnt++)
        Serial.print(telegram[cnt]);
    }
    validCRCFound = (strtol(messageCRC, NULL, 16) == currentCRC);
    if (validCRCFound)
      Serial.println("\nVALID CRC FOUND!");
    else
      Serial.println("\n===INVALID CRC FOUND!===");
    currentCRC = 0;
    CRCerror++;
  } else {
    currentCRC = CRC16(currentCRC, (unsigned char*)telegram, len);
    if (outputOnSerial) {
      for (int cnt = 0; cnt < len; cnt++)
        Serial.print(telegram[cnt]);
    }
  }

  long val = 0;
  long val2 = 0;
  // 1-0:1.8.1(000992.992*kWh)
  // 1-0:1.8.1 = Elektra verbruik dagtarief (Fluvius) - Totale afname van energie in kWh dagtarief
  if (strncmp(telegram, "1-0:1.8.1", strlen("1-0:1.8.1")) == 0)
    mEVLT = getValue(telegram, len);


  // 1-0:1.8.2(000560.157*kWh)
  // 1-0:1.8.2 = Elektra verbruik nachttarief (Fluvius) - Totale afname van energie in kWh nachttarief
  if (strncmp(telegram, "1-0:1.8.2", strlen("1-0:1.8.2")) == 0)
    mEVHT = getValue(telegram, len);


  // 1-0:2.8.1(000348.890*kWh)
  // 1-0:2.8.1 = Elektra opbrengst dagtarief (Fluvius) - Totale injectie van energie in kWh dagtarief
  if (strncmp(telegram, "1-0:2.8.1", strlen("1-0:2.8.1")) == 0)
    mEOLT = getValue(telegram, len);


  // 1-0:2.8.2(000859.885*kWh)
  // 1-0:2.8.2 = Elektra opbrengst nachttarief (Fluvius) - Totale injectie van energie in kWh nachttarief
  if (strncmp(telegram, "1-0:2.8.2", strlen("1-0:2.8.2")) == 0)
    mEOHT = getValue(telegram, len);


  // 1-0:1.7.0(00.424*kW) Actueel verbruik (Fluvius) - Afgenomen ogenblikkelijk vermogen in kW
  // 1-0:2.7.0(00.000*kW) Actuele teruglevering (Fluvius) - Geïnjecteerd ogenblikkelijk vermogen in kW
  // 1-0:1.7.x = Electricity consumption actual usage (DSMR v4.0)
  if (strncmp(telegram, "1-0:1.7.0", strlen("1-0:1.7.0")) == 0)
    mEAV = getValue(telegram, len);

  if (strncmp(telegram, "1-0:2.7.0", strlen("1-0:2.7.0")) == 0)
    mEAT = getValue(telegram, len);


  // 0-1:24.2.1(150531200000S)(00811.923*m3)
  // 0-1:24.2.3 = Gas (Fluvius) on Siconia S211 meter
  if (strncmp(telegram, "0-1:24.2.3", strlen("0-1:24.2.3")) == 0)
    mGAS = getValue(telegram, len);

  // EXTRA INFO FLUVIUS
  // 1-0:1.8.0(000859.885*kWh)
  // Totale afgenomen energie in kWh (som van 1.8.1 en 1.8.2)
  if (strncmp(telegram, "0-1:1.8.0", strlen("0-1:1.8.0")) == 0)
    mEVT = getValue(telegram, len);

  // 1-0:2.8.0(000859.885*kWh)
  // Totale geïnjecteerde energie in kWh (som van 2.8.1 en 2.8.2)
  if (strncmp(telegram, "0-1:2.8.0", strlen("0-1:2.8.0")) == 0)
    mEOT = getValue(telegram, len);

  // 1-0:32.7.0(232.0 V)
  // Ogenblikkelijke spanning in fase 1 in V Fluvius
  if (strncmp(telegram, "1-0:32.7.0", strlen("1-0:32.7.0")) == 0)
    mEOVOL1 = getValue(telegram, len);

  // 1-0:52.7.0(232.0 V)
  // Ogenblikkelijke spanning in fase 2 in V Fluvius
  if (strncmp(telegram, "1-0:52.7.0", strlen("1-0:52.7.0")) == 0)
    mEOVOL2 = getValue(telegram, len);

  // 1-0:72.7.0(232.0 V)
  // Ogenblikkelijke spanning in fase 3 in V Fluvius
  if (strncmp(telegram, "1-0:72.7.0", strlen("1-0:72.7.0")) == 0)
    mEOVOL3 = getValue(telegram, len);

  // 1-0:31.7.0(11.2 A)
  // Ogenblikkelijke stroom door fase 1 in A Fluvius
  if (strncmp(telegram, "1-0:31.7.0", strlen("1-0:31.7.0")) == 0)
    mEOPL1 = getValue(telegram, len);

  // 1-0:51.7.0(11.2 A)
  // Ogenblikkelijke stroom door fase 2 in A Fluvius
  if (strncmp(telegram, "1-0:51.7.0", strlen("1-0:51.7.0")) == 0)
    mEOPL2 = getValue(telegram, len);

  // 1-0:71.7.0(11.2 A)
  // Ogenblikkelijke stroom door fase 3 in A Fluvius
  if (strncmp(telegram, "1-0:71.7.0", strlen("1-0:71.7.0")) == 0)
    mEOPL3 = getValue(telegram, len);

  return validCRCFound;
}

void readTelegram() {
  if (mySerial.available()) {
    memset(telegram, 0, sizeof(telegram));
    while (mySerial.available()) {
      int len = mySerial.readBytesUntil('\n', telegram, MAXLINELENGTH);
      telegram[len] = '\n';
      telegram[len + 1] = 0;
      yield();
      if (decodeTelegram(len + 1)) {
        UpdateElectricity();
        UpdateGas();
      }
    }
  }
}

//SEND FUNCTIONS///////////////
bool SendToDomo( char* SENSOR_NAME, int idx, int nValue, char* sValue) {
  HTTPClient http;
  bool retVal = false;
  char url[255];
  sprintf(url, "http://%s/%s/json.htm?type=command&param=udevice&Name=%s&idx=%d&nvalue=%d&svalue=%s", HOST_NAME, PATH_NAME, SENSOR_NAME, idx, nValue, sValue);
  Serial.printf("[HTTP] GET... URL: %s\n", url);
  http.begin(url);  //HTTP
  int httpCode = http.GET();
  // httpCode will be negative on error
  if (httpCode > 0) {  // HTTP header has been send and Server response header has been handled
    Serial.printf("[HTTP] GET... code: %d\n", httpCode);

    // file found at server
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      retVal = true;
    }
  } else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
  return retVal;
}

/////////////////////////////////////////////////////
/*
  void readTelegramTEST() {
  UpdateElectricityTEST();
  delay(1000);
  UpdateGasTEST();
  delay(6000);
  }
*/
