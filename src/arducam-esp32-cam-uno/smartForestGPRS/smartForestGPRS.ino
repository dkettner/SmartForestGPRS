/********************************************************************************************
 * Board used for development: "Arducam ESP32 UNO PSRAM (Arducam IoTai)"                    *
 * https://www.arducam.com/docs/esp32-camera/the-arducam-iotai/                             *
 *                                                                                          *
 * Initializes camera,                                                                      *
 * initializes storage,                                                                     *
 * creates directories,                                                                     *
 * preapares a dummy TinyML model,                                                          *
 * enters light sleep with wakeup conditions (PIR sensor).                                  *
 * If presence is dectected,                                                                *
 * wakes up,                                                                                *
 * takes a picture,                                                                         *
 * manages picture index with EEPROM,                                                       *
 * saves picture to sd card,                                                                *
 * performs dummy calculations on model,                                                    *
 * enters light sleep again.                                                                *
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
 * EloquentTinyML:                                                                          *
 * https://eloquentarduino.github.io/2020/01/easy-tinyml-on-esp32-and-arduino/              *
 * by Simone Salerno                                                                        *
 *                                                                                          *
 * Last Minute Engineers:                                                                   *
 * https://lastminuteengineers.com/esp32-deep-sleep-wakeup-sources/                         *
 * by Unknown                                                                               *
 ********************************************************************************************/

#include <EEPROM.h>
#include <EloquentTinyML.h>
#include <eloquent_tinyml/tensorflow.h>
#include "sine_model.h"
#include "SD_MMC.h"
#include "arducam_esp32s_camera.h"
#include "driver/rtc_io.h"

#define PIR_PIN 33

#define N_INPUTS 1
#define N_OUTPUTS 1
#define TENSOR_ARENA_SIZE 2*1024 // adjust value if needed

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

int falseAlarmCounter = 0;
camera_fb_t * frameBuffer = NULL;
unsigned long currentPictureIndex = 0;
String currentPicturePath;
File file;

Eloquent::TinyML::TensorFlow::TensorFlow<N_INPUTS, N_OUTPUTS, TENSOR_ARENA_SIZE> tf;

void enterLightSleep() {
  delay(100); // small delay for possible serial prints

  rtc_gpio_pullup_dis(GPIO_NUM_33);
  //rtc_gpio_pulldown_en(GPIO_NUM_33);

  // helps to keep the pin low
  analogRead(PIR_PIN);
  analogRead(PIR_PIN);
  analogRead(PIR_PIN);
  
  gpio_set_level(GPIO_NUM_33, 0);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,1);
  delay(100);
  esp_light_sleep_start();
}

void doRandomStuff() {
  String messages[] = {
    "Powering up the flux capacitor",
    "Go get some coffee, this may take a while",
    "Just count to 10",
    "Moving the satellites into position",
    "Upgrade to PRO to skip this loading screen",
    "Den Filter säbern",
    "Computing chance of success",
    "Convincing AI not to turn evil",
    "Computing the secret to life, the universe, and everything",
    "Just waisting your time really",
    "Dividing by zero",
    "Sending your data to the NS- uh, I mean, our servers",
    "Reheating the coffee",
    "Does anyone even read this?",
    "Downloading more RAM",
    "Updating to Windows Vista",
    "Deleting System32 folder",
    "Alt-F4 speeds things up",
    "Hold on, I'm just finishing the song",
    "Patience! This is difficult, you know",
    "Discovering new ways of making you wait",
    "Your time is very important to us. Please wait while we ignore you",
    "You are number 42069 in the queue",
    "Dusting off spellbooks",
    "Sharpening pencils",
    "Ressurecting dead memes",
    "Removing pen from pineapple",
    "Loading, please wait",
    "Your PC ran into a problem, please restart",
    "Defragmentierung läuft",
    "Installing shaders",
    "Cleaning rusty contacts",
    "Solving traveling salesman in O(1)"
  };
  randomSeed(analogRead(14));
  
  Serial.print(messages[random(33)]);
  for (int i=0; i<3; i++) {
    delay(2250);
    Serial.print(".");
  }
  delay(2250);
  Serial.print(" Done!");
  delay(1000);
  Serial.println();
}

void setup(){
  Serial.begin(115200);
  Serial.printf("\nBoot 0: Starting initial setup.\n");

  pinMode(PIR_PIN, INPUT);

  EEPROM.begin(EEPROM_SIZE);

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

  // Prepare TinyML model
  tf.begin(sine_model);
  Serial.println("Prepared TinyML model.");

  // allocated time for reading GPS and compass
  doRandomStuff();
}

void loop() {
  Serial.printf("Entering light sleep now.\n\n");
  enterLightSleep();
  
  /* Dirty fix:
   * Added false alarm check because the development board will randomly put GPIO 33 
   * in active state during light sleep if the onboard camera has been used before.
   * GPIO 33 seems to be the most stable out of all possible RTC pins.
   * -> For further development a different/custom board should be used.
   */
  if (analogRead(PIR_PIN) < 4000) {
    Serial.printf("AnalogRead of PIR sensor: %i\n", analogRead(PIR_PIN));
    falseAlarmCounter++;
    Serial.printf("False alarm! Current false alarm count: %d/\n", falseAlarmCounter);
    return;
  }

  // take picture
  frameBuffer = arducam_camera_fb_get();
  if (!frameBuffer) {
    Serial.println("Camera capture failed");
    // What to do? restart?
  } else {
    Serial.println("Camera capture done");
  }

  // get and update picture index in EEPROM
  currentPictureIndex = EEPROM.readULong(PICTURE_INDEX_ADDRESS);
  EEPROM.writeULong(PICTURE_INDEX_ADDRESS, currentPictureIndex + 1);
  EEPROM.commit(); // .end() ?
  
  // save picture to sd
  currentPicturePath = String(PICTURES_PATH) + String(currentPictureIndex) + ".jpg";
  file = SD_MMC.open(currentPicturePath.c_str(), "w");
  if(!file){
    Serial.println("Failed to open file for writing");
    // What to do?
  } else {
    file.write(frameBuffer->buf, frameBuffer->len);
    file.close();
    Serial.printf("Saved picture to %s\n", currentPicturePath.c_str());
  }
  
  // using sine TinyML model (proof of concept)
  Serial.printf("\nTinyML proof of concept:\n");
  if (!tf.isOk()) {
    Serial.print("ERROR: ");
    Serial.println(tf.getErrorMessage());
  } else {
    for (float i = 0; i < 10; i++) {
      // pick x from 0 to PI
      float x = 3.14 * i / 10;
      float y = sin(x);
      float input[1] = { x };
      float predicted = tf.predict(input);

      Serial.print("sin(");
      Serial.print(x);
      Serial.print(") = ");
      Serial.print(y);
      Serial.print("\t predicted: ");
      Serial.println(predicted);
    }
    Serial.println();
  }
  
  // save report to sd
  // send report via gsm
  // will take at least 10 seconds (GPS and transmission report)
  doRandomStuff();
  
  // clean up
  arducam_camera_fb_return(frameBuffer);
}
