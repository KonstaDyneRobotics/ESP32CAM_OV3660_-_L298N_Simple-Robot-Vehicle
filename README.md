- Code is optimized to suit mainly small to medium size tablets (like RedMi Se 8.7) used for controlling.
- VGA (640x480) video streaming .
- Touchscreen optimized highly responsive UI.
- Motor speed & LED lighting control with "blanket" and "signal" features.
- Tilt Camera control (with optional servo motor attached).
- Large video streaming window .
- Enhanced Video image adapted to OV3660 camera sensor.
- "Carbon Fiber_Red" (somewhat "muted" ) style UI ,but gentle with the eyes .
- Code is directly compatible with quite a few  2WD/4WD ESP32 CAM & L298N based robot vehicles found on the global market.


Basic Parts List
-ESP32CAM AI THINKER module with OV3660 camera sensor and external WiFi antenna 
(0 Ohm resistor has to be unsoldered from onboard antenna pad and be soldered to the external antenna plug pad)
-L298N Motor Driver
-3 x18650 3.7V Li ion batteries + 3x18650 battery case/holder + toggle switch 
-2WD / 4WD platform (chassis + motors + tracks/wheels)
-(optional) S90 analog micro servo + bracket for camera tilt (DIY)
-(optional) DC/DC 12V>5V 2A PSU for powering the servo motor


Software
Arduino IDE : Download " esp32 " at board manager by Espressif Systems Ver .: 3.1.0 <===IMPORTANT !!!
(optional): Download SimpleServoESP32  ver: 1.0.1  library for the servo motor.

Pin connections
5V ==> to 5V out of the L298N Motor Driver module 
GND ==> to GND of the L298N Motor Driver module 
12 ==> to IN4  of the L298N Motor Driver module, 
13 ==> to IN3  of the L298N Motor Driver module, 
15 ==> to IN2  of the L298N Motor Driver module ,
14 ==> to IN1  of the L298N Motor Driver module ,
2 ==> to Signal Input of the servo motor  ( camera tilt )


// L298N Motor Connections 
OUT1 : Red wire (+) of Right motor(s) 
OUT2 : Black wire (-) of Right motor(s) 
OUT3 : Black wire (-) of Left motor(s)
OUT1 : Red wire (+) of Left motor(s)


WiFi ssid : ESP32CAM Robot
WiFi password: 1234567890
Once WiFi connection is established ,on the web browser type : 192.168.4.1 to enter the UI control page


UI DEscription (top of screen to bottom ) :
* 540 pixel wide video sreaming screen
* 
* "CAM TILT" Slider : adjusts camera tilt at steps of 5 degrees ,range : 0 to 180 degrees
* 
* "CAM_RST" button : sets camera at default position / servo at 45 deg 
* "TURN_L" button : moves robot forward and left (differential turn )
* "ADVANCE" button :moves robot straight forward
* "TURN_R" button : moves robot forward and right (differential turn ) 
* "CAM_UP" button : sets camera at 45 deg position / servo at 90 deg
* 
* "LT_ON/X" button : LED light ON / Blanket light when ON 
* "ROTATE_L" button : rotates robot left (counterclockwise)
* "STOP!" button : stops motors
* "ROTATE_R" button : rotates robot right (clockwise)
* "LT_OFF/S" button : LED light OFF / light signaling  when OFF
* 
* "SPD_145" button : sets pwm motor speed at 145
* "RVRS_L" button : moves robot backwards and left (differential turn )
* "REVERSE" button :moves robot straight backwards
* "RVRS_R" button :moves robot backwards and right (differential turn) 
* "SPD_205" button :  sets pwm motor speed at 205
* 
* "SPEED" Slider : adjusts pwm motor speed ,range : 105 to 255


Known bugs :
1) Due to browser touchscreen latency deactivation ,touching a button and dragging the finger away from the button 
results for the given command not to cancel itself .i.e. robot doesn't n stop or lights remain ON or OFF ,after signalling.
UI is super crisp and sensitive at the cost of that bug .Try not to drag or swipe your fingers over the buttons ,
or from button to button .Light touches only ,are more than enough and the control is very accurate and precise.

For more info and questions  : sdsvcstems@gmail.com









