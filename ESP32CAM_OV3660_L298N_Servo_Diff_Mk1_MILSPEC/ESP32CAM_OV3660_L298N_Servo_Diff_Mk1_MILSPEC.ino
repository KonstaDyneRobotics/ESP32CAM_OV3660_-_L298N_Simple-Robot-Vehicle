/* 
- Code is optimized to suit mainly small to medium size tablets (like RedMi Se 8.7) used for controlling.
-  VGA (640x480) video streaming .
- Touchscreen optimized highly responsive UI.
- Motor speed & LED lighting control with "blanket" and "signal" features.
- Tilt Camera control (with optional servo motor attached).
- Large video streaming window .
- Enhanced Video image adapted to OV3660 camera sensor.
- "Carbon Fiber_Red" (somewhat "muted" ) style UI ,but gentle with the eyes .
*************** Basic Parts List ******************************************* 
-ESP32CAM AI THINKER module with OV3660 camera sensor and external WiFi antenna 
(0 Ohm resistor has to be unsoldered from onboard antenna pad and be soldered to the external antenna plug pad)
-L298N Motor Driver
-3 x18650 3.7V Li ion batteries + 3x18650 battery case/holder + toggle switch 
-2WD / 4WD platform (chassis + motors + tracks/wheels)
-(optional) S90 analog micro servo + bracket for camera tilt (DIY)
-(optional) DC/DC 12V>5V 2A PSU for powering the servo motor
*****************************************************************************
Arduino IDE : Download " esp32 " at board manager by Espressif Systems Ver .: 3.1.0 <===IMPORTANT !!!
(optional): Download SimpleServoESP32  ver: 1.0.1  library for the servo motor.
#############################Pin connections#################################  
5V ==> to 5V out of the L298N Motor Driver module 
GND ==> to GND of the L298N Motor Driver module 
12 ==> to IN4  of the L298N Motor Driver module, LEFT_M1
13 ==> to IN3  of the L298N Motor Driver module, LEFT_M0 
15 ==> to IN2  of the L298N Motor Driver module ,RIGHT_M1 
14 ==> to IN1  of the L298N Motor Driver module ,RIGHT_M0 
2 ==> to Signal Input of the servo motor  ( camera tilt )
****************************************************************************
// L298N Motor Connections 
OUT1 : Red wire (+) of Right motor(s) 
OUT2 : Black wire (-) of Right motor(s) 
OUT3 : Black wire (-) of Left motor(s)
OUT1 : Red wire (+) of Left motor(s)
*****************************************************************************
WiFi ssid : ESP32CAM Robot
WiFi password: 1234567890
Once WiFi connection is established ,on the web browser type : 192.168.4.1 to enter the UI control page
#############################################################################
*/
#include "esp_camera.h"
#include <WiFi.h>
//WiFi credentials
const char* ssid = "ESP32CAM Robot"; //input robot's Wifi SSID
const char* password = "1234567890";// input robot'sWifi password
//AI THINKER Pin Definitions
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
//
extern void robot_Stopp();
extern void robot_setup();
extern void servo_set();
extern int obLed ;
extern int RLED ;
extern int servoPin;
extern int serval;
extern String WiFiAddr;
extern void startCameraServer();
unsigned long lastWiFiCheck = 0;
bool clientConnected = false;
//Set Up 
void setup() 
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  robot_setup();
  servo_set();
  pinMode(obLed, OUTPUT); //Onboard flash led
  digitalWrite(obLed, LOW);
  //
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  //
  config.xclk_freq_hz = 16000000; // 20000000 Hz is standard but requires stable power supply (5V ,2A)
  //and can interfere with the 20MHz Wifi4 Signal
  //16000000 Hz or 8000000 provide better more stability. Range: 6000000 to 24000000
  config.pixel_format = PIXFORMAT_JPEG;
  // 
  if(psramFound()){
    config.jpeg_quality = 10; // 0 to 63 higher to lower ,10 default 
    config.fb_count = 2; 
  } else {
  config.jpeg_quality = 15;
  config.fb_count = 1;
  }
  // Camera start
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: Error 0x%x", err);
    return;
  }
  //
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);  //  QVGA /CIF for better WiFi performance / VGA middleground / SVGA for image quality
  /*FRAMESIZE_QVGA (320 x 240)
  FRAMESIZE_CIF (352 x 288)
  FRAMESIZE_VGA (640 x 480)
  FRAMESIZE_SVGA (800 x 600)
  *///
  //Camera Picture/video adjustments 
  //( !!!Uncheck only the necessary !!! framerate drops the more unchecked !)
  //
  // Set camera rotation 180 degrees and horizontal mirror
  //s->set_hmirror(s, 0);  // Horizontal mirror
  s->set_vflip(s, 0);    // Vertical flip =1 ,Normal =0
  //Basic image enhancements
  s->set_saturation(s, 1);//saturation (-2 to 2 )
  s->set_contrast(s, -1); // contrast (-2 to 2 )
  //s->set_brightness(s, 0);//set brightness (-2 to 2 )
  s->set_sharpness(s, 2); // -2 to +2
   // Auto white Balance 
  s->set_whitebal(s, 1);//auto wb (0 = disable , 1 = enable)
  s->set_awb_gain(s, 1);//auto wb gain(0 = disable , 1 = enable)
  s->set_wb_mode(s, 0);//auto wb ( 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  //Software auto corrections
  //->set_lenc(s, 1); //auto lens correction  0 = disable , 1 = enable)
  //s->set_raw_gma(s, 0);//auto γ correction 0 = disable , 1 = enable)
  //s->set_bpc(s, 0);//auto black pixel correction  0 = disable , 1 = enable)
  //s->set_wpc(s, 0); // auto white pixel correction 0 = disable , 1 = enable)
  //Special effects 
  //s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  // Auto exposure control
  s->set_exposure_ctrl(s, 1); // 0 = disable , 1 = enable ,basic auto exposure control 
  s->set_aec2(s, 1); // 0 = disable , 1 = enable  , advanced auto exposure control
  s->set_ae_level(s, 0);// -2 to 2 auto exposure control  adjustment (trimming)
  //s->set_aec_value(s, 600); // 0 to  1200 manual exposure control ,manual EV steps WHEN set_exposure_ctrl(s, 0) & s->set_aec2(s, 0)
  s->set_gain_ctrl(s, 1);                  // 0 = disable , 1 = enable auto gain control 
  //s->set_agc_gain(s, 15);                   // 0 to 30 manual gain control WHEN s->set_gain_ctrl(s, 0)  
  //s->set_gainceiling(s, (gainceiling_t)0); // 0 to 6 ,light gain for dark environments 
  //
  WiFi.softAP(ssid, password, 1, 0, 1);// ( ssid 63char max , password 8char min ,Wifi channel ( 1 to 13),
  //ssid_hidden: (0 = broadcast SSID, 1 = hide SSID),max_connection: maximum simultaneous connected clients (1-4))
  WiFi.setSleep(false); // !!! Disable sp32 cam wifi sleep mode !!! Very important for uninterrupted WiFi communication
  WiFi.mode(WIFI_AP); // !!! Lock WiFi mode = Access point mode: stations can connect to the ESP32
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // !!! Setting WiFi Max Transmision power
  IPAddress RoboIP = WiFi.softAPIP();
  Serial.print("RobotUI IP address: ");
  Serial.println(RoboIP);
  Serial.print("Camera Initialized! Enter: 'http://");
  Serial.print(WiFi.softAPIP());
  WiFiAddr = WiFi.softAPIP().toString();
  Serial.println("' for UI");
  startCameraServer();
  digitalWrite(RLED,LOW); // on-board red LED lights up after WiFi is initiated.
}
//Loop
void loop() {
//  WiFi client watchdog ( every 400 ms )
  if (millis() - lastWiFiCheck > 400) {
    lastWiFiCheck = millis();
    int stations = WiFi.softAPgetStationNum();
    if (stations == 0 && clientConnected) {
      // Client just disconnected :Stop motors !
      Serial.println("WiFi client lost.Robot STOP!");
      robot_Stopp();
      digitalWrite(obLed, LOW);
      clientConnected = false;
    }
    if (stations > 0 && !clientConnected) {
      // Client just connected
      Serial.println("WiFi client connected");
      clientConnected = true;
    }
  }
}
//

//**********************-----END------***************************************
