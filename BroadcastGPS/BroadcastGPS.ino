/*
  Configuring the GNSS to automatically send position reports over Serial and CAN.
  The systems will broadcast and display them using a callback.
  
  Original program is
  By: Paul Clark
  SparkFun Electronics
  Date: December 21st, 2022

  Modified by Jeremy Daily 
  Colorado State University

  License: MIT. Please see LICENSE.md for more information.

  This example shows how to instruct a u-blox module to output its position, velocity and time (PVT) data periodically.
  We also turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic dramatically.
  The data is accessed using a callback.

  Note: Lat/Lon are large numbers because they are * 10^7. To convert lat/lon
  to something google maps understands simply divide the numbers by 10,000,000.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)    https://www.sparkfun.com/products/15136
  SparkFun GPS-RTK-SMA - ZED-F9P (GPS-16481) https://www.sparkfun.com/products/16481
  SparkFun MAX-M10S Breakout (GPS-18037)     https://www.sparkfun.com/products/18037
  SparkFun ZED-F9K Breakout (GPS-18719)      https://www.sparkfun.com/products/18719
  SparkFun ZED-F9R Breakout (GPS-16344)      https://www.sparkfun.com/products/16344

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and your microcontroller board
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/


#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include <FlexCAN_T4.h>
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD

SerLCD lcd; // Initialize the library with default I2C address 0x72

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

SFE_UBLOX_GNSS_SERIAL myGNSS; // SFE_UBLOX_GNSS_SERIAL uses Serial (UART). 

#define UTC_OFFSET 6 //Mountain Daylight Time

#define gpsSerial Serial2 // Use Serial1 to connect to the GNSS module. Change this if required
#define lcdSerial Serial5

#define RED_LED    3
#define GREEN_LED  2
#define YELLOW_LED 4
#define AMBER_LED  5 

#define LED_TIMEOUT 200

boolean GREEN_LED_state;
boolean RED_LED_state;
boolean YELLOW_LED_state;
boolean AMBER_LED_state;

elapsedMillis green_toggle_timer;
elapsedMillis red_toggle_timer;
elapsedMillis yellow_toggle_timer;
elapsedMillis AMBER_toggle_timer;
elapsedMillis display_timer;

uint32_t us;  //microseconds returned by getUnixEpoch()
uint32_t epoch;
uint32_t lat;  
uint32_t lon;
uint8_t  num_sats;
uint16_t heading_deg;
double speed_kph;

uint8_t source_address = 28; //Vehicle Navigation
uint32_t position_pgn = 0xFF0100; //Proprietary
uint32_t velocity_pgn = 0xFF0200; //Proprietary
uint32_t gps_time_pgn = 0xFF0300; //Proprietary
uint32_t priority = 0x10000000; //4

void setup()
{
  //LED Setup
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(GREEN_LED,OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(AMBER_LED,OUTPUT);
  GREEN_LED_state = HIGH;
  YELLOW_LED_state = HIGH;
  RED_LED_state = HIGH;
  AMBER_LED_state = HIGH;
  
  check_led_timers();

  Can1.begin();
  Can1.setBaudRate(250000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();
  Can1.onReceive(can1Sniff);
  Can1.mailboxStatus();
  
  Can2.begin();
  Can2.setBaudRate(250000);
  Can2.setMaxMB(16);
  Can2.enableFIFO();
  Can2.enableFIFOInterrupt();
  Can2.onReceive(can2Sniff);
  Can2.mailboxStatus();

  Serial.begin(115200);
  delay(1000); 
  Serial.println("SparkFun u-blox Example");

  gpsSerial.begin(38400); // u-blox F9 and M10 modules default to 38400 baud. Change this if required
  while (myGNSS.begin(gpsSerial) == false) //Connect to the u-blox module using gpsSerial (defined above)
  {
    Serial.println(F("u-blox GNSS not detected. Retrying..."));
    delay (1000);
  }
  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

 

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART1 port to output UBX only (turn off NMEA noise) 
  myGNSS.setNavigationFrequency(10); // Produce two solutions per second
  myGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
  
  lcdSerial.begin(9600);

  lcd.begin(lcdSerial); //Set up the LCD for Serial communication at 9600bps

  lcd.setBacklight(255, 255, 255); //Set backlight to bright white
  lcd.setContrast(5); //Set contrast. Lower to 0 for higher contrast.

  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  lcd.print("Hello, World!");
}


// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{   
  CAN_message_t msg;
  msg.len = 8;
  msg.flags.extended = 1;

  //Send out time and number of sats.
  num_sats = myGNSS.getSIV();
  //Number of satelites as the first byte
  msg.buf[0] = num_sats;
  msg.id = priority + gps_time_pgn + source_address;
  epoch = myGNSS.getUnixEpoch(us);
  double time_float = double(epoch) + double(us)/1000000.;
  uint32_t usec = us;
  memcpy(&msg.buf[1],&usec,3);
  memcpy(&msg.buf[4],&epoch,4);
  Can2.write(msg);
  if (num_sats>3) Can1.write(msg);

  Serial.printf("(%0.06f) %08X ", time_float, msg.id);
  for (uint8_t i = 0; i< msg.len; i++){
    Serial.printf("%02X ",msg.buf[i]);
  }
  Serial.println();

  msg.id = priority + position_pgn + source_address;
  lat = myGNSS.getLatitude(); // Print the latitude // Latitude: deg * 1e-7
  lon = myGNSS.getLongitude(); // Print the longitude // Longitude: deg * 1e-7
  memcpy(&msg.buf[0],&lat,4);
  memcpy(&msg.buf[4],&lon,4);
  if (num_sats>3){
    Can1.write(msg);
    Can2.write(msg);
    GREEN_LED_state = !GREEN_LED_state;
    green_toggle_timer = 0;
    AMBER_LED_state = !AMBER_LED_state;
    AMBER_toggle_timer = 0;
    
  
    Serial.printf("(%0.06f) %08X ",time_float,msg.id);
    for (uint8_t i = 0; i< msg.len; i++){
      Serial.printf("%02X ",msg.buf[i]);
    }
    Serial.println();
    
  }

  msg.id = priority + velocity_pgn + source_address;
  uint32_t speed = uint32_t(myGNSS.getGroundSpeed()) * 3600; // m/hour
  heading_deg = myGNSS.getHeading(); // deg * 1e-5
  memcpy(&msg.buf[0],&speed,4);
  memcpy(&msg.buf[4],&heading_deg,4);
  if (num_sats>3) {
    Can2.write(msg);

    speed_kph = double(speed)/1000000.0;
    
    Serial.printf("(%0.06f) %08X ", time_float, msg.id);
    for (uint8_t i = 0; i< msg.len; i++){
      Serial.printf("%02X ",msg.buf[i]);
    }
    Serial.println();
    

  }  
  

  update_display(ubxDataStruct);
}

void can1Sniff(const CAN_message_t &msg) {
  YELLOW_LED_state = !YELLOW_LED_state;
  yellow_toggle_timer = 0;
  //CAN_message_t new_msg = msg;
  //Pass through PGNs of interest
  if (msg.id == 0x18FEF100) {
    Can2.write(msg); //Cruise Control Vehicle Speed
    GREEN_LED_state = !GREEN_LED_state;
    green_toggle_timer = 0;
  }
  else if (msg.id == 0x0CF00400){
    Can2.write(msg); // Engine control
    GREEN_LED_state = !GREEN_LED_state;
    green_toggle_timer = 0;
  } 
  
}

void can2Sniff(const CAN_message_t &msg) {
  RED_LED_state = !RED_LED_state;
  red_toggle_timer = 0;
  Can1.write(msg); //Pass through all
  AMBER_LED_state = !AMBER_LED_state;
  AMBER_toggle_timer = 0;
  }


void check_led_timers(){
  if (red_toggle_timer > LED_TIMEOUT){RED_LED_state = LOW;}
  if (AMBER_toggle_timer > LED_TIMEOUT){AMBER_LED_state = LOW;}
  if (green_toggle_timer > LED_TIMEOUT){GREEN_LED_state = LOW;}
  if (yellow_toggle_timer > LED_TIMEOUT){YELLOW_LED_state = LOW;}
  digitalWrite(GREEN_LED,GREEN_LED_state);
  digitalWrite(RED_LED,RED_LED_state);
  digitalWrite(YELLOW_LED,YELLOW_LED_state);
  digitalWrite(AMBER_LED,AMBER_LED_state);
}

void update_display(UBX_NAV_PVT_data_t *ubxDataStruct){
  if (display_timer >= 200){
    display_timer = 0;
    int8_t hr = ubxDataStruct->hour;
    if (hr > UTC_OFFSET) hr -= int8_t(UTC_OFFSET);
    else hr += (24 - UTC_OFFSET);
    lcd.clear(); //Clear the display - this moves the cursor to home position as well
    //lcd.printf("%02d  %02d:%02d:%02d\n",num_sats,ubxDataStruct->hour,ubxDataStruct->min,ubxDataStruct->sec);
    if (num_sats>3) lcd.printf("Sats:%02d %02d:%02d:%02d%8d S:%4.2f\n",num_sats,hr,ubxDataStruct->min,ubxDataStruct->sec,lat,speed_kph);
    else lcd.printf("Searching 4 Sats%02d:%02d:%02d",ubxDataStruct->hour,ubxDataStruct->min,ubxDataStruct->sec);
  }
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
  Can1.events();
  Can2.events();
  check_led_timers();
}
