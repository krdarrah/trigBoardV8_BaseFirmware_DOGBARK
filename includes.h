#ifndef INCLUDES_H
#define INCLUDES_H

/*
 * Versions:
 * Arduino IDE v1.8.15
 * ESP32 v1.06
 * PubSubClient Library v2.8.0
 * Arduino Json Library v6.18.0
 * UniversalTelegramBot v1.3.0
 */

// **********************    THIS ALL FOR AUDIO STUFF    ****************************
#include <SoftwareSerial.h>//connection to mp3 board
const int ARDUINO_RX_PIN = 33; //should connect to TX of the Serial MP3 Player module
const int ARDUINO_TX_PIN = 32; //connect to RX of the module
const int AUDIO_ENABLE_PIN = 13;
SoftwareSerial mp3(ARDUINO_RX_PIN, ARDUINO_TX_PIN);
bool donePlaying = false;//so we know when done playing, then kill power to the speaker
byte currentHour = 0;//this is the current hour from RTC
static int8_t Send_buf[8] = {0}; // Buffer for Send commands.  // BETTER LOCALLY
static uint8_t ansbuf[10] = {0}; // Buffer for the answers.    // BETTER LOCALLY
String mp3Answer;           // Answer from the MP3.
String sanswer(void);
String sbyte2hex(uint8_t b);
void playTheTrack();
int trackToPlay;
/************ Command byte **************************/
#define CMD_NEXT_SONG     0X01  // Play next song.
#define CMD_PREV_SONG     0X02  // Play previous song.
#define CMD_PLAY_W_INDEX  0X03
#define CMD_VOLUME_UP     0X04
#define CMD_VOLUME_DOWN   0X05
#define CMD_SET_VOLUME    0X06

#define CMD_SNG_CYCL_PLAY 0X08  // Single Cycle Play.
#define CMD_SEL_DEV       0X09
#define CMD_SLEEP_MODE    0X0A
#define CMD_WAKE_UP       0X0B
#define CMD_RESET         0X0C
#define CMD_PLAY          0X0D
#define CMD_PAUSE         0X0E
#define CMD_PLAY_FOLDER_FILE 0X0F

#define CMD_STOP_PLAY     0X16  // Stop playing continuously. 
#define CMD_FOLDER_CYCLE  0X17
#define CMD_SHUFFLE_PLAY  0x18 //
#define CMD_SET_SNGL_CYCL 0X19 // Set single cycle.

#define CMD_SET_DAC 0X1A
#define DAC_ON  0X00
#define DAC_OFF 0X01

#define CMD_PLAY_W_VOL    0X22
#define CMD_PLAYING_N     0x4C
#define CMD_QUERY_STATUS      0x42
#define CMD_QUERY_VOLUME      0x43
#define CMD_QUERY_FLDR_TRACKS 0x4e
#define CMD_QUERY_TOT_TRACKS  0x48
#define CMD_QUERY_FLDR_COUNT  0x4f
#define DEV_TF            0X02
/*******************************************************************/

//libraries used
#include "FS.h"// for SPIFFS
#include "SPIFFS.h"
#include "time.h"

#include <WiFi.h>
#include <Wire.h>//for I2C RTC
#include <WiFiClientSecure.h>

//*** THESE ARE THE ONLY LIBRARIES YOU NEED TO INSTALL ***
#include <PubSubClient.h>//for mqtt
#include <ArduinoJson.h>
#include <UniversalTelegramBot.h>
//********************************************************

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <ESPmDNS.h>
#include <ArduinoOTA.h>

//trigBoard PINS
const int BatteryPin = 36;//analog Input
const int LEDpin = 0;//output
const int ESPlatchPin = 16;//output
const int killPowerPin = 17;//output
const int contactOpenedPin = 18;//input
const int contactClosedPin = 19;//input
const int contactStatusPin = 23;//input
const int wakeButtonPin = 27;//input
const int SDApin = 21;//rtc I2C
const int SCLpin = 22;//rtc I2C

//globals
struct Config {//full configuration file
  char ssid[50];
  char pw[50];
  int wifiTimeout;
  char trigName[50];
  char trigSelect[10];
  char triggerOpensMessage[50];
  char triggerClosesMessage[50];
  char buttonMessage[50];
  int  timerCountDown;
  char timerSelect[10];
  char StillOpenMessage[50];
  char StillClosedMessage[50];
  double batteryThreshold;
  double batteryOffset;
  char pushOverEnable[3];
  char pushUserKey[50];
  char pushAPIKey[50];
  char pushSaferEnable[3];
  char pushSaferKey[50];
  char iftttEnable[3];
  char iftttMakerKey[50];
  char udpEnable[3];
  char tcpEnable[3];
  char udpTargetIP[20];
  char udpStaticIP[20];
  char udpGatewayAddress[20];
  char udpSubnetAddress[20];
  char udpPrimaryDNSAddress[20];
  char udpSecondaryDNSAddress[20];
  char udpSSID[50];
  char udpPW[50];
  int udpPort;
  char rtcCountdownMinute[3];
  char mqttEnable[3];
  int mqttPort;
  char mqttServer[50];
  char mqttTopic[50];
  char mqttSecureEnable[3];
  char mqttUser[50];
  char mqttPW[50];
  char staticIPenable[3];
  char staticIP[20];
  char staticGatewayAddress[20];
  char staticSubnetAddress[20];
  char staticPrimaryDNSAddress[20];
  char staticSecondaryDNSAddress[20];
  int udpBlastCount;
  int udptimeBetweenBlasts;
  char highSpeed[3];
  char clkEnable[3];
  int clkTimeZone;
  char clkAppendEnable[3];
  char clkAlarmEnable[3];
  int clkAlarmHour;
  int clkAlarmMinute;
  char clkUpdateNPTenable[3];
  char clkAlarmMessage[50];
  char clkAppendAlmEnable[3];
  char telegramEnable[3];
  char telegramBOT[50];
  char telegramCHAT[50];
  char appendRSSI[3];
  char checkAgain[3];
  char timerCheck[3];
  char lastState[3];
  char failedConnect[3];
  int secondsAfterToCheckAgain;
  
};

//bluetooth
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long bluetoothFlasherTime;

uint8_t txValue = 0;
#define SERVICE_UUID           "a5f125c0-7cec-4334-9214-58cffb8706c0" // UART service UUID
#define CHARACTERISTIC_UUID_RX "a5f125c1-7cec-4334-9214-58cffb8706c0"
#define CHARACTERISTIC_UUID_TX "a5f125c2-7cec-4334-9214-58cffb8706c0"
unsigned long bluetoothStatusStartTime;
unsigned long bluetoothParamStartTime;
bool sendParam = false;
bool testPushover = false;
char batCharString[5];
char pushMessage[100];
bool OTAsetup = false;
const char *filename = "/config.txt";
//logic
bool timerWake;
bool clockWake;
bool updateRTC;
bool updateRTCtime;
bool contactLatchClosed;
bool contactLatchOpen;
bool lowBattery;
bool contactStatusClosed;
bool buttonWasPressed;
bool contactChanged = false;
bool wiFiNeeded = false;
Config config;

//rtc
char rtcTimeStamp[20];
bool checkAgainSet = false;
//function prototypes
//    wakeup tab
void checkWakeupPins();
void killPower();
void waitForButton();
void checkIfContactChanged();
//    rtc tab
bool rtcInit(byte timeValue, bool setNewTime);
void rtcGetTime();
bool getNTPtime();
void nptUpdateTime();
void timestampAppend();
//    pushover tab
boolean pushOver();
//    WiFi tab
bool connectWiFi();
void appendRSSI();
char rssiChar[10];
//    battery tab
float getBattery();
//    congiguration tab
void loadConfiguration(const char *filename, Config &config);
void saveConfiguration(const char *filename, const Config &config);
//    bluetooth tab
void initBluetooth();
void serviceBluetooth();
//    logic tab
bool pushLogic();
//    udp tab
void udp();
int oneIP = 0, twoIP = 0, threeIP = 0, fourIP = 0;
void getFourNumbersForIP(const char *ipChar);//used also in WiFi & tcp tab
//tcp tab

// ifttt tab
void ifttt();
//pushSafer tab
struct PushSaferInput {
  String message;
  String title;
  String sound;
  String vibration;
  String icon;
  String iconcolor;
  String device;
  String url;
  String urlTitle;
  String time2live;
  String priority;
  String retry;
  String expire;
  String answer;
  String picture;
  String picture2;
  String picture3;
};
void pushSafer();
String Pushsafer_buildString(String boundary, String name, String value);
String Pushsafer_sendEvent(PushSaferInput input);
//  mqtt tab
void mqtt();
//  OTA tab
void setupOTA();
void checkOTA();


#endif
