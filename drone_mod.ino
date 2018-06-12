/* 
drone_mod


*/

#include <Wire.h>
#include <VL53L0X.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <SimpleTimer.h>
#include "arduino_secrets.h" 

VL53L0X sensorF;
VL53L0X sensorR;
VL53L0X sensorL;
VL53L0X sensorB;

int status = WL_IDLE_STATUS;

char ssid[] = SECRET_SSID;   // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 2391;      // local port to listen on

char packetBuffer[255];             //buffer to hold incoming packet

WiFiUDP Udp;

int period = 0;
//boolean front_output = HIGH;
//boolean back_output  = HIGH;
int num = 0;

// the timer object
SimpleTimer timer;

void setupWiFi() {
  //Initialize serial and wait for port to open:
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    //Serial.print("Attempting to connect to SSID: ");
    //Serial.println(ssid);
    digitalWrite(6, HIGH);digitalWrite(6, LOW);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  //Serial.println("Connected to wifi");
  //printWiFiStatus();
  digitalWrite(6, HIGH);digitalWrite(6, LOW);

  //Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

void setupToFSensor() {
  pinMode(FSHUT, OUTPUT);
  pinMode(RSHUT, OUTPUT);
  pinMode(LSHUT, OUTPUT);
  pinMode(BSHUT, OUTPUT);
  digitalWrite(FSHUT, LOW);
  digitalWrite(RSHUT, LOW);
  digitalWrite(LSHUT, LOW);
  digitalWrite(BSHUT, LOW);
 
  delay(100);
  Wire.begin();

  pinMode(FSHUT, INPUT);
  delay(150);
  sensorF.init(true);
  delay(100);
  sensorF.setAddress((uint8_t)24);
  sensorF.setTimeout(500);

  pinMode(RSHUT, INPUT);
  delay(150);
  sensorR.init(true);
  delay(100);
  sensorR.setAddress((uint8_t)23);
  sensorR.setTimeout(500);
 
  pinMode(LSHUT, INPUT);
  delay(150);
  sensorL.init(true);
  delay(100);
  sensorL.setAddress((uint8_t)24);
  sensorL.setTimeout(500);
 
  pinMode(BSHUT, INPUT);
  delay(150);
  sensorB.init(true);
  delay(100);
  sensorB.setAddress((uint8_t)25);
  sensorB.setTimeout(500);

  sensorF.startContinuous();
  sensorR.startContinuous();
  sensorL.startContinuous();
  sensorB.startContinuous();
  digitalWrite(13,LOW);

  timer.setInterval(10, MeasureDistance);
}

void setupLED() {
  pinMode(FRONT_LED_PIN, OUTPUT);
  pinMode(BACK_LED_PIN,  OUTPUT);
  timer.setInterval(INTERVAL, LChika);
}

void LChika() {
  digitalWrite(FRONT_LED_PIN, !(num % 4));
  digitalWrite(BACK_LED_PIN,  !(num % 8));
  //back_output  =  !(front_output != back_output);
  //front_output = !front_output;
  num = (num + 1) % 8;
}

void MeasureDistance() {
  //Serial.print("F: ");
  //Serial.print(sensorF.readRangeContinuousMillimeters());
  //Serial.print("R: ");
  //Serial.print(sensorR.readRangeContinuousMillimeters());
  //Serial.print(" L: ");
  //Serial.print(sensorL.readRangeContinuousMillimeters());
  //Serial.print(" B: ");
  //Serial.println(sensorB.readRangeContinuousMillimeters());
  Udp.beginPacket("192.168.10.32", 2391); //nagaonet no surface no ip
  Udp.write("R: ");
  Udp.write(char(sensorR.readRangeContinuousMillimeters()));
  Udp.write(" L: ");
  Udp.write(char(sensorL.readRangeContinuousMillimeters()));
  Udp.write(" B: ");
  Udp.write(char(sensorB.readRangeContinuousMillimeters()));

  Udp.endPacket();
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(19200);
  pinMode(6, OUTPUT);
  setupWiFi();
  setupToFSensor();
  setupLED();
}

void loop() {
  // put your main code here, to run repeatedly:
  timer.run();
}



