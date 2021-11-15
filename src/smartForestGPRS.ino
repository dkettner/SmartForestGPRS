/********************************************************************************************
 * Board used for development: "Arducam ESP32 UNO PSRAM (Arducam IoTai)"                    *
 * https://www.arducam.com/docs/esp32-camera/the-arducam-iotai/                             *
 *                                                                                          *
 * Initializes camera,                                                                      *
 * takes a picture,                                                                         *
 * initializes storage,                                                                     *
 * creates directories,                                                                     *
 * manages picture index with EEPROM,                                                       *
 * saves picture to sd card,                                                                *
 * enters deep sleep with wakeup conditions (PIR sensor).                                   *
 *                                                                                          *
 * This code is based on the following sources:                                             *
 *                                                                                          *
 * ArduCAM coding examples included with Arduino package:                                   *
 * -> "Arducam_ESP32_PSRAM_OV2640_Capture2SD"                                               *
 * -> "Arducam_ESP32_PSRAM_ExternalWakeUp"                                                  *
 * https://www.arducam.com/downloads/esp32_uno_psram/package_ArduCAM_ESP32_PSRAM_index.json *
 * ArduCAM demo (C)2019 Lee Jackson                                                         *
 * Web: http://www.ArduCAM.com                                                              *
 *                                                                                          *
 * Random Nerd Tutorials:                                                                   *
 * https://randomnerdtutorials.com/esp32-cam-take-photo-save-microsd-card/                  *
 * https://randomnerdtutorials.com/esp32-cam-pir-motion-detector-photo-capture/             *
 * by Sara Santos and Rui Santos                                                            *
 *                                                                                          *
 * Others:                                                                                  *
 * https://lastminuteengineers.com/esp32-deep-sleep-wakeup-sources/                         *
 * https://eloquentarduino.github.io/2020/01/easy-tinyml-on-esp32-and-arduino/              *
 ********************************************************************************************/

#include <EEPROM.h>
#include "SD_MMC.h"
#include "arducam_esp32s_camera.h"
#include "driver/rtc_io.h"

#define PIR_PIN 33

#define EEPROM_SIZE             4
#define PICTURE_INDEX_ADDRESS   0

#define PICTURES_PATH "/pictures/"
#define REPORTS_PATH  "/reports/"
#define ERRORS_PATH   "/errors/"

String directories[] = {
  PICTURES_PATH,
  REPORTS_PATH,
  ERRORS_PATH
};

RTC_DATA_ATTR int falseAlarmCounter = 0;
RTC_DATA_ATTR int bootCounter = 0;

void enterDeepSleep() {
  delay(100); // small delay for serial prints

  //rtc_gpio_pullup_dis(GPIO_NUM_33);
  //rtc_gpio_pulldown_en(GPIO_NUM_33);

  // helps to keep the pin low
  analogRead(PIR_PIN);
  analogRead(PIR_PIN);
  analogRead(PIR_PIN);
  
  gpio_set_level(GPIO_NUM_33, 0);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1);
  esp_deep_sleep_start();
}

void doRandomStuff() {
  Serial.print("Powering up the flux compensator");
  for (int i=0; i<3; i++) {
    delay(2000);
    Serial.print(".");
  }
  delay(2000);
  Serial.print(" Done!");
  delay(2000);
  Serial.println();
}

void setup(){
  Serial.begin(115200);
  Serial.println();

  // Do nothing on boot 0 except entering deep sleep
  if (bootCounter++ == 0) {
    Serial.println("Boot 0. Entering deep sleep now.");
    enterDeepSleep();
  }

  /* Dirty fix:
   * Added false alarm check because the development board will randomly put GPIO 33 
   * in active state during deep sleep if the onboard camera has been used before.
   * GPIO 33 seems to be the most stable out of all possible RTC pins.
   * -> For further development a different/custom board should be used.
   */
  if (analogRead(PIR_PIN) < 4000) {
    falseAlarmCounter++;
    Serial.printf("False alarm! Current false alarm ratio: %d/%d\n", falseAlarmCounter, bootCounter);
    Serial.println("Entering deep sleep now.");
    enterDeepSleep();
  }
  
  // initialize camera
  esp_err_t err = arducam_camera_init(PIXFORMAT_JPEG);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  } else {
    Serial.println("Camera init success");
  }
  sensor_t * sensor = arducam_camera_sensor_get();
  sensor->set_framesize(sensor, FRAMESIZE_UXGA);

  // take picture
  camera_fb_t * frameBuffer = NULL;
  frameBuffer = arducam_camera_fb_get();
  if (!frameBuffer) {
    Serial.println("Camera capture failed");
    // What to do? restart?
  } else {
    Serial.println("Camera capture done");
  }

  // initialize sd
  if(!SD_MMC.begin()){
    Serial.println("Card Mount Failed");
    return; // really?
  } else {
    Serial.println("Card Mount success");
  }
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD_MMC card attached");
    return; // really?
  }

  // create directories
  Serial.println("Starting to create nonexistent directories.");
  for (String currentDirectory: directories) {
    if (SD_MMC.exists(currentDirectory.c_str())) {
      if (SD_MMC.mkdir(currentDirectory.c_str())) {
        Serial.printf("Created directory: %s \n", currentDirectory.c_str());
      } else {
        Serial.printf("Could not create directory: %s !\n", currentDirectory.c_str());
        // Try again?
      }
    } else {
      Serial.printf("Directory %s already exists.\n", currentDirectory.c_str());
    }
  }

  // get and update picture index in EEPROM
  EEPROM.begin(EEPROM_SIZE);
  unsigned long currentPictureIndex = EEPROM.readULong(PICTURE_INDEX_ADDRESS);
  EEPROM.writeULong(PICTURE_INDEX_ADDRESS, currentPictureIndex + 1);
  EEPROM.commit(); // .end() ?
  
  // save picture to sd
  String currentPicturePath = String(PICTURES_PATH) + String(currentPictureIndex) + ".jpg";
  File file = SD_MMC.open(currentPicturePath.c_str(), "w");
  if(!file){
    Serial.println("Failed to open file for writing");
    // What to do?
  } else {
    file.write(frameBuffer->buf, frameBuffer->len);
    file.close();
    Serial.printf("Saved picture to %s\n", currentPicturePath.c_str());
  }

  // build report
  // save report to sd
  // send report via gsm
  // will take at least 10 seconds (gps and transmission of transport)
  doRandomStuff();
  
  // clean up and deep sleep
  arducam_camera_fb_return(frameBuffer);
  Serial.println("Entering deep sleep now.");
  enterDeepSleep();
}

void loop() {}
