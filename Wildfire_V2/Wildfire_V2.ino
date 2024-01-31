//libraries
#include <Adafruit_HTU21DF.h>
#include <ArduCAM.h>
#include <ArduinoHttpClient.h>
#include <MKRGSM.h>
#include <PubSubClient.h>
#include "secrets.h"
#include <SD.h>
#include <SPI.h>
#include <stdint.h>
#include <TinyGPS++.h>
#include <Wire.h>

//global variables
const char* pin = SECRET_PINNUMBER;
const char* apn = SECRET_GPRS_APN;
const char* login = SECRET_GPRS_LOGIN;
const char* password = SECRET_GPRS_PASSWORD;
const char* mqtt_server = IOT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* http_server = S3_SERVER;
const int http_port = HTTP_PORT;

//Objects
GSM gsmAccess;
GPRS gprsAccess;
GSMClient Gclient;
HttpClient client = HttpClient(Gclient, http_server, http_port);
PubSubClient mqttClient(Gclient);

char incomingByte;
#define photoRelayPin 1
#define ionRelayPin 2

//GPSSerial object created, linked to serial port of MKR GSM 1400
#define GPSSerial Serial1

//baud rate for the GPS
int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

//Object created for location
GSMLocation location;

File videoFile;
char str[8];
const int bufferSize = 10240;
uint8_t buffer[bufferSize];
String videoName;

const char outTopic[] = TOPIC;
const int SIZE = 512;
//char postData[SIZE];
String postData;

//Temp/Humi Sensor object
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
float temp = 0, humi = 0;
bool photoAlarm = false;

long time = 0;

//PWM pin for fan
const int FAN_PIN = 3;
//Fan speed var
float fanSpeed = 255;

//Anemometer variables
int anemPin = A4;  //, anemDebug = 1;
float anemVolt = 0, anemSpeed = 0;

//Wind vane variables
float vanVolt = 0;
String myDirection = "ERROR";
//int vanDebug = 1,
int vanPin = A6;

//variables for LED
int led1State = LOW;  // ledState used to set the LED
unsigned long currentMillis;
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 1000;        // interval at which to blink (milliseconds)

int debugFlag = 1;

//variables for ion and photo sensor values
int photoSensorValue;
int ionSensorValue;
String photoSensor = "false", ionSensor = "false";

//variables for longitude and lattitude (GPS)
float GPSlongi = 0;
float GPSlati = 0;

//variables for timing getting coordinates
unsigned long startTime;        //approx time when we started
unsigned long finishTime;       //approx time we ended when we looked for a coordinate
unsigned long elapsedTime = 0;  //interval of elapsed time

//variables for GSM coordinates
float gsmLati = 0;    //holds first reading
float gsmLongi = 0;   //holds first reading
float gsmLati2 = 0;   //holds second more accurate reading
float gsmLongi2 = 0;  //holds second more accurate reading
int GPSkey = 0;       //key so we only get GPS coordinates once
int GSMkey = 0;       //key so we only iterate through the cell tower triangulation twice

//coordinates to be used, decided between either GPS or cell tower triangulation coordinates
float mainLati = 0;
float mainLongi = 0;


int pollSensorsFlag = 0;
//----------------------------------------------------------------------------------

// DEFINES
#if !(defined(OV5640_MINI_5MP_PLUS) || defined(OV5642_MINI_5MP_PLUS))
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif

//#define FISHINO_UNO // Nice UNO board with integrated RTC, microSD, WiFi

#define SERIAL_SPEED 115200
#define BUFFSIZE 512  // 512 is a good buffer size for SD writing. 4096 would be better, on boards with enough RAM (not Arduino Uno of course)
#define FRAME_SIZE OV5642_320x240
#define WIDTH_1 0x40      // Video width in pixel, hex. Here we set 320 (Big Endian: 320 = 0x01 0x40 -> 0x40 0x01). For 640: 0x80
#define WIDTH_2 0x01      // For 640: 0x02
#define HEIGHT_1 0xF0     // 240 pixels height (0x00 0xF0 -> 0xF0 0x00). For 480: 0xE0
#define HEIGHT_2 0x00     // For 480: 0x01
#define FPS 0x0F          // 15 FPS. Placeholder: will be overwritten at runtime based upon real FPS attained
#define TOTAL_FRAMES 200  // Number of frames to be recorded. If < 256, easier to recognize in header (for manual hex debug)
//set pin 7 as the slave select for SPI:
#define SPI_CS 7

//ETCG Notes -- SD Pin
#define SD_CS 4  // 9 on Arducam adapter Uno and SD shields
//set this to 10 due to OV5642 Model

//if using FISHINO_UNO
#ifdef FISHINO_UNO
#define SD_AUX 10  // Needs to be Output on Fishino Uno for the integrated SD card to work
#endif
#define AVIOFFSET 240  // AVI main header length

//ionization sensor read voltage
const int voltage = A0;

// GLOBALS
unsigned long movi_size = 0;
unsigned long jpeg_size = 0;
const char zero_buf[4] = { 0x00, 0x00, 0x00, 0x00 };
const int avi_header[AVIOFFSET] PROGMEM = {
  0x52,
  0x49,
  0x46,
  0x46,
  0xD8,
  0x01,
  0x0E,
  0x00,
  0x41,
  0x56,
  0x49,
  0x20,
  0x4C,
  0x49,
  0x53,
  0x54,
  0xD0,
  0x00,
  0x00,
  0x00,
  0x68,
  0x64,
  0x72,
  0x6C,
  0x61,
  0x76,
  0x69,
  0x68,
  0x38,
  0x00,
  0x00,
  0x00,
  0xA0,
  0x86,
  0x01,
  0x00,
  0x80,
  0x66,
  0x01,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x10,
  0x00,
  0x00,
  0x00,
  0x64,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x01,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  WIDTH_1,
  WIDTH_2,
  0x00,
  0x00,
  HEIGHT_1,
  HEIGHT_2,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x4C,
  0x49,
  0x53,
  0x54,
  0x84,
  0x00,
  0x00,
  0x00,
  0x73,
  0x74,
  0x72,
  0x6C,
  0x73,
  0x74,
  0x72,
  0x68,
  0x30,
  0x00,
  0x00,
  0x00,
  0x76,
  0x69,
  0x64,
  0x73,
  0x4D,
  0x4A,
  0x50,
  0x47,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x01,
  0x00,
  0x00,
  0x00,
  FPS,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x0A,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x73,
  0x74,
  0x72,
  0x66,
  0x28,
  0x00,
  0x00,
  0x00,
  0x28,
  0x00,
  0x00,
  0x00,
  WIDTH_1,
  WIDTH_2,
  0x00,
  0x00,
  HEIGHT_1,
  HEIGHT_2,
  0x00,
  0x00,
  0x01,
  0x00,
  0x18,
  0x00,
  0x4D,
  0x4A,
  0x50,
  0x47,
  0x00,
  0x84,
  0x03,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x4C,
  0x49,
  0x53,
  0x54,
  0x10,
  0x00,
  0x00,
  0x00,
  0x6F,
  0x64,
  0x6D,
  0x6C,
  0x64,
  0x6D,
  0x6C,
  0x68,
  0x04,
  0x00,
  0x00,
  0x00,
  0x64,
  0x00,
  0x00,
  0x00,
  0x4C,
  0x49,
  0x53,
  0x54,
  0x00,
  0x01,
  0x0E,
  0x00,
  0x6D,
  0x6F,
  0x76,
  0x69,
};
//edited to this from past code, works better and sense OV5642 camera
#if defined(OV5640_MINI_5MP_PLUS)
ArduCAM myCAM(OV5640, SPI_CS);
#else
ArduCAM myCAM(OV5642, SPI_CS);
#endif
// END GLOBALS

static void inline print_quartet(unsigned long i, File fd) {  // Writes an uint32_t in Big Endian at current file position
  fd.write(i % 0x100);
  i = i >> 8;  //i /= 0x100;
  fd.write(i % 0x100);
  i = i >> 8;  //i /= 0x100;
  fd.write(i % 0x100);
  i = i >> 8;  //i /= 0x100;
  fd.write(i % 0x100);
}

static void Video2SD() {  // We don't enforce FPS: we just record and save frames as fast as possible
  // Then we compute the attained FPS and update the AVI header accordingly
  char str[8];
  uint16_t n;
  File outFile;
  byte buf[BUFFSIZE];
  static int i = 0;
  uint8_t temp = 0, temp_last = 0;
  unsigned long fileposition = 0;
  uint16_t frame_cnt = 0;


  uint16_t remnant = 0;
  uint32_t length = 0;
  uint32_t startms;
  uint32_t elapsedms;
  uint32_t uVideoLen = 0;
  bool is_header = false;

#ifndef DISABLE_SD

  //ETCG Notes -- Create File Name
  // Create a avi file. Name should be unique-ish, but short
  digitalWrite(SD_CS, HIGH);
  randomSeed(analogRead(0) * millis());
  n = (random(2, 999));  // Don't use 1.avi: was the default in old code, we don't want to overwrite old recordings
  itoa(n, str, 10);
  strcat(str, ".avi");

  //Open the new file
  outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
  if (!outFile) {
    Serial.println(F("File open failed"));
    while (1)
      ;
    return;
  }

  videoName = String(str);

#endif

  //Write AVI Main Header
  // Some entries will be overwritten later
  for (i = 0; i < AVIOFFSET; i++) {
    char ch = pgm_read_byte(&avi_header[i]);
    buf[i] = ch;
  }
#ifndef DISABLE_SD
  outFile.write(buf, AVIOFFSET);
#endif

  Serial.print(F("\nRecording "));
  Serial.print(TOTAL_FRAMES);
  Serial.println(F(" video frames: please wait...\n"));

  startms = millis();

  //Write video data, frame by frame
  for (frame_cnt = 0; frame_cnt < TOTAL_FRAMES; frame_cnt++) {
#if defined(ESP8266)
    yield();
#endif
    temp_last = 0;
    temp = 0;
    //Capture a frame
    //Flush the FIFO
    myCAM.flush_fifo();
    //Clear the capture done flag
    myCAM.clear_fifo_flag();
    //Start capture
    myCAM.start_capture();
    // Wait for frame ready
    while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK))
      ;
    length = myCAM.read_fifo_length();  // Length of FIFO buffer. In general, it contains more than 1 JPEG frame;
                                        // so we'll have to check JPEG markers to save a single JPEG frame
#if defined(SPI_HAS_TRANSACTION)
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
#endif
#ifndef DISABLE_SD
    // Write segment. We store 1 frame for each segment (video chunk)
    outFile.write("00dc");       // "start of video data chunk" (00 = data stream #0, d = video, c = "compressed")
    outFile.write(zero_buf, 4);  // Placeholder for actual JPEG frame size, to be overwritten later
#endif
    i = 0;
    jpeg_size = 0;

    // Deassert camera Chip Select to start SPI transfer
    myCAM.CS_LOW();
    // Set FIFO to burst read mode
    myCAM.set_fifo_burst();
    // Transfer data, a byte at a time
    while (length--) {  // For every byte in the FIFO...
#if defined(ESP8266)
      yield();
#endif
      // We always need the last 2 bytes, to check for JPEG begin/end markers
      temp_last = temp;           // Save current temp value
      temp = SPI.transfer(0x00);  // Overwrite temp with 1 byte from FIFO (0x00 is dummy byte for the slave: we are reading, the slave will ignore it)
#if defined(SPI_HAS_TRANSACTION)
      SPI.endTransaction();
#endif
      // a JPEG ends with the two bytes 0xFF, 0xD9
      if ((temp == 0xD9) && (temp_last == 0xFF))  // End of the image
      {
        buf[i++] = temp;  // Add this last byte to the buffer
        myCAM.CS_HIGH();  // End of transfer: re-assert Slave Select
#ifndef DISABLE_SD
        // Write the buffer to file
        outFile.write(buf, i);
#endif
        is_header = false;  // We are at the last byte of the JPEG: sure is not the header :)
        jpeg_size += i;     // Update total jpeg size with this last buffer size
        i = 0;              // Reset byte counter (restart writing from the first element of the buffer)
      }
      if (is_header == true)  // Not at end of JPEG, yet
      {
        //Write image data to buffer if not full
        if (i < BUFFSIZE)
          buf[i++] = temp;
        else {              // Buffer is full: transfer to file
          myCAM.CS_HIGH();  // End SPI transfer

#ifndef DISABLE_SD
          //Write BUFFSIZE bytes image data to file
          outFile.write(buf, BUFFSIZE);
#endif
          i = 0;                   // Restart writing from the first element
          buf[i++] = temp;         // Save current byte as first in "new" buffer
          myCAM.CS_LOW();          // Re-enable SPI transfer
          myCAM.set_fifo_burst();  // Set FIFO to burst read mode
          jpeg_size += BUFFSIZE;
        }
      } else if ((temp == 0xD8) & (temp_last == 0xFF)) {  // A JPEG starts with the two bytes 0xFF, 0XD8; so here we are at the beginning of the JPEG
        is_header = true;
        buf[i++] = temp_last;  // Save the first two bytes (off-cycle)
        buf[i++] = temp;
      }
    }  // end loop over each byte in the FIFO: JPEG is complete

    // Padding
    remnant = jpeg_size & 0x00000001;  // Align to 16 bit: add 0 or 1 "0x00" bytes
#ifndef DISABLE_SD
    if (remnant > 0) {
      outFile.write(zero_buf, remnant);  // see https://docs.microsoft.com/en-us/windows/desktop/directshow/avi-riff-file-reference
    }
#endif
    movi_size += jpeg_size;  // Update totals
    uVideoLen += jpeg_size;  // <- This is for statistics only

    // Now we have the real frame size in bytes. Time to overwrite the placeholder

#ifndef DISABLE_SD
    fileposition = outFile.position();                     // Here, we are at end of chunk (after padding)
    outFile.seek(fileposition - jpeg_size - remnant - 4);  // Here we are the the 4-bytes blank placeholder
    print_quartet(jpeg_size, outFile);                     // Overwrite placeholder with actual frame size (without padding)
    outFile.seek(fileposition - jpeg_size - remnant + 2);  // Here is the FOURCC "JFIF" (JPEG header)
    outFile.write("AVI1", 4);                              // Overwrite "JFIF" (still images) with more appropriate "AVI1"

    // Return to end of JPEG, ready for next chunk
    outFile.seek(fileposition);
#endif
  }  // End cycle for all frames
  // END CAPTURE

  // Compute statistics
  elapsedms = millis() - startms;
  float fRealFPS = (1000.0f * (float)frame_cnt) / ((float)elapsedms);
  float fmicroseconds_per_frame = 1000000.0f / fRealFPS;
  uint8_t iAttainedFPS = round(fRealFPS);                  // Will overwrite AVI header placeholder
  uint32_t us_per_frame = round(fmicroseconds_per_frame);  // Will overwrite AVI header placeholder

#ifndef DISABLE_SD
  //Modify the MJPEG header from the beginning of the file, overwriting various placeholders
  outFile.seek(4);
  print_quartet(movi_size + 12 * frame_cnt + 4, outFile);  //    riff file size

  //movie_size = movi_size;
  //frame_count = frame_cnt;
  //overwrite hdrl
  //hdrl.avih.us_per_frame:
  outFile.seek(0x20);
  print_quartet(us_per_frame, outFile);
  unsigned long max_bytes_per_sec = movi_size * iAttainedFPS / frame_cnt;  //hdrl.avih.max_bytes_per_sec
  outFile.seek(0x24);
  print_quartet(max_bytes_per_sec, outFile);
  //hdrl.avih.tot_frames
  outFile.seek(0x30);
  print_quartet(frame_cnt, outFile);
  outFile.seek(0x84);
  print_quartet((int)iAttainedFPS, outFile);
  //hdrl.strl.list_odml.frames
  outFile.seek(0xe0);
  print_quartet(frame_cnt, outFile);
  outFile.seek(0xe8);
  print_quartet(movi_size, outFile);  // size again
  myCAM.CS_HIGH();
  //Close the file
  outFile.close();
#endif

  Serial.println(F("\n*** Video recorded and saved ***\n"));
  Serial.print(F("Recorded "));
  Serial.print(elapsedms / 1000);
  Serial.print(F("s in "));
  Serial.print(frame_cnt);
  Serial.print(F(" frames\nFile size is "));
  Serial.print(movi_size + 12 * frame_cnt + 4);
  Serial.print(F(" bytes\nActual FPS is "));
  Serial.print(fRealFPS, 2);
  Serial.print(F("\nMax data rate is "));
  Serial.print(max_bytes_per_sec);
  Serial.print(F(" byte/s\nFrame duration is "));
  Serial.print(us_per_frame);
  Serial.println(F(" us"));
  Serial.print(F("Average frame length is "));
  Serial.print(uVideoLen / TOTAL_FRAMES);
  Serial.println(F(" bytes"));
}
//-------------------------------------------------------------------------------

void setup() {
  uint8_t vid, pid;

  uint8_t temp;

  Wire.begin();

  Serial.begin(SERIAL_SPEED);  //115200

  pinMode(voltage, INPUT);  //voltage read

  pinMode(FAN_PIN, OUTPUT);

  pinMode(INPUT, vanPin);

  pinMode(INPUT, anemPin);

  pinMode(1, OUTPUT); //relay 1 pin

  pinMode(2, OUTPUT); //relay 2 pin

  digitalWrite(1, HIGH); //relay is high, or engaged

  digitalWrite(2, HIGH); //relay is high, or engaged

  analogWrite(FAN_PIN, fanSpeed);

  htu.begin();

  //while (!Serial) {};

  delay(200);

  Serial.println(F("ArduCAM Start!\n"));

  delay(5000);  // Gain time to start logic analyzer

#ifndef DISABLE_SD
  // set the SPI_CS as an output:
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
#ifdef FISHINO_UNO
  pinMode(SD_AUX, OUTPUT);
#endif
#endif

  delay(1000);

  // initialize SPI:
  SPI.begin();

#ifndef DISABLE_SD
  //Initialize SD Card
  while (!SD.begin(SD_CS)) {
    Serial.println(F("SD Card Error!"));
    delay(1000);
  }
  Serial.println(F("SD Card detected."));
  delay(200);
#endif


  //Reset the CPLD
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(200);

  while (1) {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55) {
      Serial.println(F("SPI interface Error!"));
      delay(1000);
      continue;
    } else {
      Serial.println(F("SPI interface OK."));
      break;
    }
  }
  delay(100);
  //edited in order to detect OV5642
#if defined(OV5640_MINI_5MP_PLUS)
  while (1) {
    //Check if the camera module type is OV5640
    myCAM.rdSensorReg16_8(OV5640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5640_CHIPID_LOW, &pid);
    if ((vid != 0x56) || (pid != 0x40)) {
      Serial.println(F("Can't find OV5640 module!"));
      delay(1000);
      continue;
    } else {
      Serial.println(F("OV5640 detected."));
      break;
    }
  }
#else
  while (1) {
    //Check if the camera module type is OV5642
    myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    if ((vid != 0x56) || (pid != 0x42)) {
      Serial.println(F("Can't find OV5642 module!"));
      delay(1000);
      continue;
    } else {
      Serial.println(F("OV5642 detected."));
      break;
    }
  }
#endif
  delay(1000);
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  //myCAM.OV5642_set_JPEG_size(FRAME_SIZE);
  myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
  myCAM.OV5642_set_JPEG_size(FRAME_SIZE);
}

void loop() {
  ConnectGSM();
  GPScoordinates();
  GSMcoordinates();
  GSMorGPS();
  DebugHandler();
  ConnectMQTT();
  TempHumiFan();
  AnemVane();
  PublishData();

  //Determine alarm state
  if (temp > 57 && humi < 15 && /*voltage1 > 4.65 &&*/ (photoSensor == "true" || ionSensor == "true")) {
    photoAlarm = 2;
  } else if (temp > 57 && humi < 15 /* || voltage1 > 4.65*/) {
    photoAlarm = 1;
  } else {
    photoAlarm = 0;
  }


  if (photoAlarm == 2 || debugFlag == 0) {
    if (debugFlag == 0) {
      pollSensorsFlag = 0;

      Serial.println("***********************************************************");

      Serial.println("DO NOT PANIC! THIS IS A FIRE DETECTION TEST! DEBUGGER ON!");

      Serial.println("***********************************************************\n");

      PollSensors();
    }
    Serial.print("Fire Detected!!!");

    Video2SD();

    delay(1000);

    OpenVideo(videoName);

    ConnectGSM();

    ConnectMQTT();

    PublishData();

    PostVideo(videoName);

    DeleteVideo();
  }

  Serial.println("---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");

  delay(5000);  //5 minutes
}

//----------------------------------------------------------------------------------
void TempHumiFan() {
  //Poll sensors
  temp = htu.readTemperature();  //C
  humi = htu.readHumidity();

  if (temp < 24) {
    fanSpeed = 64;
  } else if (temp > 23 && temp < 76) {
    fanSpeed = (3.82 * temp - 27.68);
  } else {
    fanSpeed = 255;
  }

  analogWrite(FAN_PIN, fanSpeed);
}
//----------------------------------------------------------------------------------
void ConnectGSM() {
  //Connecting GSM network
  if (gsmAccess.status() != GSM_READY) {
    Serial.print("Connecting GSM: ");
    while (gsmAccess.begin(pin) != GSM_READY) {
      Serial.print(".");
      delay(1000);
    }
    Serial.println("Connected");
  }

  //Connecting GPRS network
  if (gprsAccess.status() != GPRS_READY) {
    Serial.print("Connecting GPRS: ");
    while (gprsAccess.attachGPRS(apn, login, password) != GPRS_READY) {
      Serial.print(".");
      delay(1000);
    }
    Serial.println("Connected");
  }
}
//----------------------------------------------------------------------------------
void ConnectMQTT() {
  //Connect to mqtt broker
  mqttClient.setServer(mqtt_server, mqtt_port);

  if (!mqttClient.connected()) {
    // MQTT client is disconnected, connect
    Serial.print("Attempting to MQTT broker: ");
    Serial.print(mqtt_server);
    Serial.println(" ");

    // while (!mqttClient.connect(url, port)) {
    while (!mqttClient.connect("MKR_client")) {
      // failed, retry
      Serial.print(".");

      ConnectGSM();

      delay(5000);
    }
    Serial.println();

    Serial.println("You're connected to the MQTT broker");
    Serial.println();
  }
}
//----------------------------------------------------------------------------------
void AnemVane() {
  //Vane voltage
  vanVolt = analogRead(vanPin) * 5.0 / 1023.0;
  //Anemomter voltage
  anemVolt = analogRead(anemPin) * 2.5 / 1023.0;

  float vanMax = 3.83;

  //myDirections expressed as a range of voltages
  if (vanVolt <= vanMax * 1 / 16 || vanVolt >= vanMax * 15 / 16) {
    myDirection = "N";
  } else if (vanVolt > vanMax * 1 / 16 && vanVolt <= vanMax * 3 / 16) {
    myDirection = "NE";
  } else if (vanVolt > vanMax * 3 / 16 && vanVolt <= vanMax * 5 / 16) {
    myDirection = "E";
  } else if (vanVolt > vanMax * 5 / 16 && vanVolt <= vanMax * 7 / 16) {
    myDirection = "SE";
  } else if (vanVolt > vanMax * 7 / 16 && vanVolt <= vanMax * 9 / 16) {
    myDirection = "S";
  } else if (vanVolt > vanMax * 9 / 16 && vanVolt <= vanMax * 11 / 16) {
    myDirection = "SW";
  } else if (vanVolt > vanMax * 11 / 16 && vanVolt <= vanMax * 13 / 16) {
    myDirection = "W";
  } else if (vanVolt > vanMax * 13 / 16 && vanVolt < vanMax * 15 / 16) {
    myDirection = "NW";
  } else {
    myDirection = "ERROR";
  }

  //Wind speed = 12 * voltage
  anemSpeed = 12.0 * anemVolt;

  if (debugFlag == 0) {
    //Wind vane voltage
    Serial.print("VV: " + String(vanVolt) + " V - ");

    //Wind myDirection output
    Serial.print(String(myDirection) + "\t");

    //Anemomter voltage output
    Serial.print("AV: ");
    Serial.print(anemVolt);
    Serial.print(" V - ");

    //Wind speed output
    Serial.print(anemSpeed);
    Serial.println(" mph");
  }
}
//----------------------------------------------------------------------------------
String SubName(String videoName) {
  String filename = videoName;  // Your original filename

  if (filename.endsWith(".AVI") || filename.endsWith(".avi")) {
    filename = filename.substring(0, filename.length() - 4);
    // Removes the last 4 characters (i.e. ".AVI" or ".avi")
  }

  // Now, the value of filename will be "example"
  return filename;
}
//----------------------------------------------------------------------------------
void PublishData() {
  //Poll for messages and keep alives
  mqttClient.loop();

  //Send a message
  Serial.println("Publishing message");

  time = millis();

  postData = "{\"alert\": " + String(photoAlarm) + ",\"env\": {\"humidity\": " + String(humi) + ",\"ionization\": " + ionSensor + ",\"photoelectric\": " + photoSensor + ",\"temperature\": " + String(temp) + "},\"GPS\": {\"latitude\": " + String(mainLati) + ",\"longitude\": " + String(mainLongi) + "},\"GSM\": {\"latitude\": " + String(gsmLati2) + ",\"longitude\": " + String(gsmLongi2) + "},\"name\": \"Device 1\",\"video\": \"" + SubName(videoName) + "\",\"wind\": {\"direction\": \"" + myDirection + "\",\"speed\": " + String(anemSpeed) + "}}";  //,\"timestamp\": \"" + String(millis()) + "\"}";

  if (debugFlag == 0) {
    Serial.println(postData);
  }

  mqttClient.beginPublish(outTopic, postData.length(), true);

  mqttClient.print(postData);

  mqttClient.endPublish();

  //disconnect
  Serial.println();
}
//-----------------------------------------------------------------------------
void OpenVideo(String fileName) {
  //Open video from SD card
  Serial.print("Accessing SD: ");

  while (!SD.begin(SD_CS)) {
    // Failed to open video file
    Serial.print(".");
  }

  videoFile = SD.open(fileName);

  while (!videoFile) {
    Serial.print(".");
  }

  Serial.println("Success");
}
//-----------------------------------------------------------------------------
void PostVideo(String filename) {
  // Set the timeout for the HTTP client to 5 seconds
  client.setTimeout(5000);

  Serial.println("Uploading " + String(filename));

  // Make PUT request
  client.beginRequest();
  client.put("/" + filename);

  // Set the content type of the request to indicate that it is a multipart/form-data request
  client.sendHeader("Content-Type", "video/avi; boundary=---------------------------974767299852498929531610575");

  // Calculate the total size of the request body, including the boundary markers
  size_t totalSize = strlen("-----------------------------974767299852498929531610575\r\n");
  String temp0 = "Content-Disposition: avi; name=\"file\"; filename=\"" + String(filename) + "\"\r\n";
  totalSize += temp0.length();
  totalSize += strlen("Content-Type: video/avi\r\n");
  totalSize += strlen("\r\n");
  totalSize += videoFile.size();
  totalSize += strlen("\r\n-----------------------------974767299852498929531610575--\r\n");

  //Serial.println("Content-Length: " + String(totalSize));

  // Set the content length of the request to the total size of the request body
  client.sendHeader("Content-Length", String(totalSize));

  // Start the request body with the boundary marker
  client.beginBody();
  client.print("-----------------------------974767299852498929531610575\r\n");
  client.print(temp0);
  client.print("Content-Type: video/avi\r\n");
  client.print("\r\n");

  long startTime = millis();  // record start time
  long prevTime = startTime;  // for calculating upload speed
  long bytesUploaded = 0;     // for calculating progress

  if (debugFlag == 0) {
    while (videoFile.available()) {
      size_t bytesRead = videoFile.read(buffer, sizeof(buffer));
      client.write(buffer, bytesRead);

      // Update progress bar
      bytesUploaded += bytesRead;
      float progress = (float)bytesUploaded / (float)videoFile.size() * 100.0;
      Serial.print('\r');
      Serial.print("Uploading... ");
      Serial.print(progress, 1);
      Serial.print("%  ");
      long currTime = millis();
      float uploadSpeed = (float)bytesUploaded / (float)(currTime - startTime) * 1000.0 / 1024.0;
      Serial.print(uploadSpeed, 1);
      Serial.println(" KB/s");

      // Wait a bit before sending the next chunk of data
      delay(1);
    }
  } else {
    while (videoFile.available()) {
      size_t bytesRead = videoFile.read(buffer, sizeof(buffer));
      client.write(buffer, bytesRead);
    }
  }

  // End the request body with the boundary marker
  client.print("\r\n-----------------------------974767299852498929531610575--\r\n");

  // End the request and read the status code and body of the response
  client.endRequest();

  // Get the status code and body of the response
  int statusCode = client.responseStatusCode();
  Serial.print("Status code: ");
  Serial.println(statusCode);

  if (statusCode == 200) {
    Serial.println("Video uploaded");
  } else if (statusCode == -3) {
    Serial.println("Video error");
  } else {
    String response = client.responseBody();
    Serial.print("Response: ");
    Serial.println(response);
  }

  // Close the video file
  videoFile.close();
}
//-----------------------------------------------------------------------------
void DeleteVideo() {
  if (!SD.begin(SD_CS)) {
    // Failed to open video file
    Serial.println("SD card failed");

    return;
  }

  int fileCount = 0;
  File root = SD.open("/");
  while (true) {
    File file = root.openNextFile();
    if (!file) {
      break;
    }
    if (!file.isDirectory()) {
      fileCount++;
    }
    file.close();
  }
  root.close();

  // delete files if there are more than MAX_FILES
  if (fileCount >= 20) {
    Serial.print("Deleting ");
    Serial.print(fileCount);
    Serial.println(" files");

    root = SD.open("/");
    while (true) {
      File file = root.openNextFile();
      if (!file) {
        break;
      }
      if (!file.isDirectory()) {
        Serial.print("Deleting ");
        Serial.println(file.name());
        file.close();
        SD.remove(file.name());
      }
    }
    root.close();

    Serial.println("Done deleting files");
  } else {
    Serial.print("Found ");
    Serial.print(fileCount);
    Serial.println(" files");
  }
}
//-----------------------------------------------------------------------------
void Blink(int pin, long time, int iter, int debuger) {
  //delay(500);

  if (debuger == 0) {
    for (int i = 0; i < iter; i++) {
      digitalWrite(pin, HIGH);  // turn the LED on (HIGH is the voltage level)
      delay(time);              // wait for a second
      digitalWrite(pin, LOW);   // turn the LED off by making the voltage LOW
      delay(time);
    }  // wait for a second
  }
}
//----------------------------------------------------------------------------------
void DebugHandler() {
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == '0') {
      debugFlag = 0;

      Serial.println("Debug on");
    }

    else if (input == '1') {
      debugFlag = 1;

      Serial.println("Debug off");
    }
  }
}
//----------------------------------------------------------------------------------
void menu() {
  Serial.println("---------------------- TROUBLESHOOTING MODE ---------------------------");
  Serial.println("-------- SELECTION ------------------------- FUNCTION -----------------");
  Serial.println("=======================================================================");
  Serial.println("----------'c' ------------------------------ camera -------------------");
  Serial.println("----------'t' ----------------------------- temp/humi -----------------");
  Serial.println("----------'g' -------------------------------- GPS/GSM ----------------");
  Serial.println("----------'i' ---------------------------- ionization sensor ----------");
  Serial.println("----------'p' --------------------------- photoelectric sensor --------");
  //Serial.println("----------'m' ------------------------------ alarm 1 on ---------------");
  //Serial.println("----------'n' ------------------------------ alarm 1 off --------------");
  //Serial.println("----------'k' ------------------------------ alarm 2 on ---------------");
  //Serial.println("----------'j' ------------------------------ alarm 2 off --------------");
  Serial.println("----------'f' ------------------------------ fan ----------------------");
  Serial.println("----------'l' ------------------------------ LED ----------------------");
  Serial.println("----------'a' ----------------------------- anemometer ----------------");
  Serial.println("----------'v' ---------------------------- wind vane ------------------");
  Serial.println("----------'q' ------------------------- quits current menu ------------");
  Serial.println("----------'x' ------------------------- go back to main loop ----------");
}
//----------------------------------------------------------------------------------
void PollSensors() {
  while (pollSensorsFlag == 0) {
    incomingByte = Serial.read();
    incomingByte = Serial.read();

    menu();

    incomingByte = 'b';

    while (incomingByte == 'b') {
      if (Serial.available() > 0)  //read the incoming byte to drive the selection
      {
        incomingByte = Serial.read();
      }
    }

    switch (incomingByte) {

      case 'x':
        pollSensorsFlag = 1;
        break;

      case 'c':
        Serial.println("--------------TAKING VIDEO---------------");
        Video2SD();
        delay(1000);
        break;

      case 't':
        Serial.println("--------------READING TEMP/HUMI---------------");
        while (incomingByte != 'q') {
          TempHumiFan();
          Serial.print("Temp:\t" + String(temp) + " C\tHumi:\t" + String(humi) + "\n");

          if (Serial.available() > 0) {
            // read the incoming byte:
            incomingByte = Serial.read();
          }

          delay(2000);
        }
        break;

      case 'g':
        Serial.println("--------------GETTING GSM/GPS COORDINATES---------------");
        GSMcoordinates();
        GPScoordinates();
        break;

      case 'i':
        Serial.println("--------------READING IONIZATION SENSOR---------------");
        while (incomingByte != 'q') {
          Ion();
          Serial.print("Ion Vol:\t" + String(ionSensorValue) + " C\tSmoke:\t" + String(ionSensor) + "\n");

          if (Serial.available() > 0) {
            // read the incoming byte:
            incomingByte = Serial.read();
          }

          delay(2000);
        }
        break;

      case 'p':
        Serial.println("--------------READING PHOTOELECTRIC SENSOR---------------");
        while (incomingByte != 'q') {
          Photo();
          Serial.print("Photo Vol:\t" + String(photoSensorValue) + " C\tSmoke:\t" + String(photoSensor) + "\n");

          if (Serial.available() > 0) {
            // read the incoming byte:
            incomingByte = Serial.read();
          }

          delay(2000);
        }
        break;

      case 'f':
        Serial.println("--------------ENGAGING FAN---------------");
        while (incomingByte != 'q') {
          analogWrite(FAN_PIN, 255 * 0.25);

          delay(5000);

          analogWrite(FAN_PIN, 255 * 0.5);

          delay(5000);

          analogWrite(FAN_PIN, 255 * 0.75);

          delay(5000);

          analogWrite(FAN_PIN, 255);

          delay(5000);

          if (Serial.available() > 0) {
            // read the incoming byte:
            incomingByte = Serial.read();
          }
        }
        break;

      case 'm':
        //code to test relay to poll photoelectric sensor ---------------------------------------------------
        digitalWrite(photoRelayPin, LOW);
        Serial.println("TESTING.... ALARM SHOULD BE ON");
        break;

      case 'n':
        //code to test relay to poll photoelectric sensor ------------------------------------------------
        digitalWrite(photoRelayPin, HIGH);
        Serial.println("TESTING.... ALARM SHOULD BE OFF");
        break;

      case 'k':
        //code to test relay to poll ionization sensor ---------------------------------------------------
        digitalWrite(ionRelayPin, LOW);
        Serial.println("TESTING.... ALARM SHOULD BE ON");
        break;

      case 'j':
        //code to test relay to poll ionization sensor ---------------------------------------------------
        digitalWrite(ionRelayPin, HIGH);
        Serial.println("TESTING.... ALARM SHOULD BE ON");
        break;


      case 'l':
        Serial.println("--------------BLINKING LED---------------");
        while (incomingByte != 'q') {
          locationBlink();
          if (Serial.available() > 0) {
            // read the incoming byte:
            incomingByte = Serial.read();
          }
        }
        break;

      case 'a':
        Serial.println("--------------READING ANEM SPEED---------------");
        while (incomingByte != 'q') {
          anemVolt = analogRead(anemPin) * 2.5 / 1023.0;
          anemSpeed = 12.0 * anemVolt;
          Serial.print("AV: ");
          Serial.print(anemVolt);
          Serial.print(" V - ");
          Serial.print(anemSpeed);
          Serial.println(" mph");
          delay(500);
          if (Serial.available() > 0) {
            // read the incoming byte:
            incomingByte = Serial.read();
          }
        }
        break;

      case 'v':
        Serial.println("--------------READING WIND VANE---------------");
        while (incomingByte != 'q') {
          vanVolt = analogRead(vanPin) * 5.0 / 1023.0;
          if (vanVolt <= 0.16 || vanVolt >= 2.35) {
            myDirection = "North";
          } else if (vanVolt > 0.16 && vanVolt <= 0.47) {
            myDirection = "Northeast";
          } else if (vanVolt > 0.47 && vanVolt <= 0.79) {
            myDirection = "East";
          } else if (vanVolt > 0.79 && vanVolt <= 1.10) {
            myDirection = "Southeast";
          } else if (vanVolt > 1.10 && vanVolt <= 1.41) {
            myDirection = "South";
          } else if (vanVolt > 1.41 && vanVolt <= 1.72) {
            myDirection = "Southwest";
          } else if (vanVolt > 1.72 && vanVolt <= 2.04) {
            myDirection = "West";
          } else if (vanVolt > 2.04 && vanVolt < 2.35) {
            myDirection = "Northwest";
          } else {
            myDirection = "ERROR";
          }
          Serial.print("VV: " + String(vanVolt) + " V - ");
          Serial.print(String(myDirection) + "\n");
          delay(500);
          if (Serial.available() > 0) {
            // read the incoming byte:
            incomingByte = Serial.read();
          }
        }
        break;

      default:
        Serial.println("Invalid choice. Try again.");
        break;
    }
  }
}
//----------------------------------------------------------------------------------
void GSMcoordinates() {
  startTime = millis();  //hold start time
  elapsedTime = 0;       //reset elapsed time to 0
  Serial.println("Triangulating... getting coordinates via cell towers");
  //-----------------------------------------------------------------------------------------------------
  while (elapsedTime < 240000)  //while our elapsed time is less than 4 minutes
  {

    if (GSMkey == 0)  //if its the first time getting cell tower coordinates
    {
      if (location.available())  //get first innacurate reading, will leave after first reading
      {
        Serial.println("FIRST ITERATION, STILL GETTING GSM COORDINATES");
        //Serial.print("Location: ");
        gsmLati = location.latitude();

        gsmLongi = location.longitude();

        finishTime = millis();                 //get time when we finished trying to get a coordinate
        elapsedTime = finishTime - startTime;  //calculate elapsed time
        Serial.print("Elapsed time is: ");
        Serial.println(elapsedTime);

        if ((gsmLati == 0) && (gsmLongi == 0)) {
          Serial.println("The coordinates are still 0");
          locationBlink();
        }

        if ((gsmLati != 0) && (gsmLongi != 0)) {
          Serial.println("Yay! Coordinates are not 0. They are:");
          Serial.println(gsmLati);
          Serial.println(gsmLongi);
          Serial.println("END LOOP");
          GSMkey = 1;
          gsmLati2 = gsmLati;
          gsmLongi2 = gsmLongi;
          // break;
        }
      }
    }



    if (GSMkey == 1) {
      if (location.available())  //get a more accurate reading
      {
        Serial.println("SECOND ITERATION, STILL GETTING COORDINATES");
        //Serial.print("Location: ");
        gsmLati2 = location.latitude();

        gsmLongi2 = location.longitude();
        finishTime = millis();                 //get time when we finished trying to get a coordinate
        elapsedTime = finishTime - startTime;  //calculate elapsed time
        Serial.print("Time elapsed for second iteration is: ");
        Serial.println(elapsedTime);
        locationBlink();
        if ((gsmLati != gsmLati2) && (gsmLongi != gsmLongi2)) {
          Serial.println("Yay! Coordinates are more accurate. They are:");
          Serial.println(gsmLati2);
          Serial.println(gsmLongi2);
          Serial.println("END LOOP");
          GSMkey = 2;
          break;
        }
      }
    }
  }

  if ((gsmLati == 0) && (gsmLongi == 0)) 
  {
    Serial.println("We couldnt get coordinates, so we timed out......");
  }
}
//----------------------------------------------------------------------------------
void GSMorGPS() {
  if ((GPSlongi != 0) && (GPSlati != 0))  //if GPS isnt 0, use coordinates by default
  {
    Serial.println("We will use GPS coordinates by default.... they are: ");
    mainLati = GPSlati;
    mainLongi = GPSlongi;
    Serial.println(mainLati);
    Serial.println(mainLongi);
  }

  if ((gsmLongi != 0) && (gsmLati != 0) && (GPSlongi == 0) && (GPSlati == 0)) {
    Serial.print("We will use GSM coordinates by default.... they are: ");
    if (GSMkey == 1)  //if 1 iteration of accuracy
    {
      mainLati = gsmLati;
      mainLongi = gsmLongi;
      Serial.println(mainLati);
      Serial.println(mainLongi);
    }

    else  //second degree of accuracy
    {
      mainLati = gsmLati2;
      mainLongi = gsmLongi2;
      Serial.println(mainLati);
      Serial.println(mainLongi);
    }

  }

  else
    Serial.println("Couldn't get GSM or GPS coordinates.....");
}
//----------------------------------------------------------------------------------
void GPScoordinates() {
  
  Serial.println("In function, now getting your GPS Coordinates..");
  startTime = millis();
   while (elapsedTime < 240000) 
  {
    
    if (GPSSerial.available() > 0)
      if (gps.encode(GPSSerial.read())) 
      {
         // if (gps.location.isValid()) 
         // {
          GPSlati = gps.location.lat();
          GPSlongi = gps.location.lng();
         // }
          
            finishTime = millis();                 //get time when we finished trying to get a coordinate
            elapsedTime = finishTime - startTime;  //calculate elapsed time
            //locationBlink();
            Serial.print("Time elapsed is: ");
            Serial.println(elapsedTime);

                  if ((GPSlongi != 0) && (GPSlati != 0)) 
                  {
                  Serial.print("Yay! The coordinates arent 0 for GPS. They are: Latitude: ");
                  Serial.print(GPSlati,4);Serial.print(", Longitude: ");
                  Serial.println(GPSlongi,4);
                  break;
                  }
      }
  }
     elapsedTime = 0;


  //If 5000ms pass and no characters are read in, there is a problem with the GPS
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected");
    while (true)
      ;
  }

  if ((GPSlati == 0) && (GPSlongi == 0)) 
  {
    Serial.println("We couldnt get coordinates for GPS, so we timed out......");
  }
}
//----------------------------------------------------------------------------------
void locationBlink() {
  currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (led1State == LOW) {
      led1State = HIGH;
    } else {
      led1State = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(5, led1State);
  }
}
//----------------------------------------------------------------------------------
void Photo() {
  //code for reading photoelectric sensor
  photoSensorValue = analogRead(A2);
  //Serial.println(photoSensorValue);
  if (photoSensorValue < 15) {
    photoSensor = "false";
  }
  if (photoSensorValue > 15) {
    photoSensor = "true";
  }
}
//----------------------------------------------------------------------------------
void Ion() {
  ionSensorValue = analogRead(A1);
  //Serial.println(ionSensorValue);
  if (ionSensorValue < 20) {
    ionSensor = "false";
  }
  if (ionSensorValue > 20) {
    ionSensor = "true";
  }
}
