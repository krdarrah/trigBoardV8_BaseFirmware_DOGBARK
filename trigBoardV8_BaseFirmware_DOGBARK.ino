#define OTA_DEBUG
#include "includes.h"


const char fwVersion[] = "11/29/21_DOG";

void setup() {

  pinMode(ESPlatchPin, OUTPUT);
  digitalWrite(ESPlatchPin, HIGH);
  pinMode(LEDpin, OUTPUT);
  Serial.begin(115200);
  Wire.begin(SDApin, SCLpin);
  checkWakeupPins();
  loadConfiguration(filename, config);
  rtcInit(config.timerCountDown, false);
  rtcGetTime();// so we only bark between certain hours

  Serial.println(getBattery(), 2);
  Serial.print("Current Hour: ");
  Serial.println(currentHour);
  if (pushLogic() && (currentHour >= 22 || currentHour < 8)) { //decide if push will occur or not and what message will be

    if (!buttonWasPressed) {
      mp3.begin(9600);
      delay(500);
      sendCommand(CMD_SEL_DEV, 0, DEV_TF);
      delay(200);
      if (mp3.available())
      {
        Serial.println(decodeMP3Answer());
        mp3.flush();
      }
      //if (SPIFFS.begin(true)) {
      Serial.println("Playing File");
      trackToPlay = 1;
      playTheTrack();

      //      audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
      //      //audio.setBalance(-16); // mutes the left channel
      //      audio.setVolume(21); // 0...21
      //      //audio.connecttoFS(SPIFFS, "dogBarkmp3.mp3");
      //      audio.connecttoFS(SPIFFS, "dogBarkmp3.mp3");
      //      //}
      //      // wait until done
      //      unsigned long playStartTime = millis();
      //      while (!donePlayingAudio && millis() - playStartTime < 5000) {
      //        audio.loop();
      //      }
    }
    //    SPIFFS.end();

    timestampAppend();
    if (wiFiNeeded) {
      if (connectWiFi()) {
        nptUpdateTime();
        pushOver();
        pushSafer();
        ifttt();
        telegram();
        mqtt();
      }
    }
    udp();
    tcp();
  }
  killPower();
  waitForButton();
  initBluetooth();
}

void loop() {
  if (!OTAsetup)
    serviceBluetooth();
  else
    checkOTA();
}
