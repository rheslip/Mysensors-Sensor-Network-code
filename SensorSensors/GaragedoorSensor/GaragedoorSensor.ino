/*
Garage door node
uses an ultrasonic distance sensor above the garage door to sense door open/closed
if the sensor is above the parking area it can also determine if there is a car in the garage when the door is closed
node also has a TMP100 temperature sensor to measure garage temp
This node is designed to work off a wall wart since it consumes a couple of ma even while sleeping
R Heslip Sept 2014

*/

#include <SPI.h>
#include <Wire.h>
#include "TMP100.h"
#include "MyNodes.h"
#include <MySensor.h>  
#include <NewPing.h>

#define NODE_ID GARAGE_DOOR_NODE
#define DIST_CHILD_ID GARAGE_DOOR_NODE_DIST
#define TEMP_CHILD_ID GARAGE_DOOR_NODE_TEMP
#define TRIGGER_PIN  A0  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A1  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define DIST_DELTA 5  // change of distance in cm needed to cause a message to be sent

#define  SLEEP  5000 // Sleep time between reads (in milliseconds)
#define REPORT_COUNT  120000/SLEEP // time between check-in reports (in milliseconds) - send data regardless
unsigned long SLEEP_TIME = SLEEP; // Sleep time between reads (in milliseconds)
int nsleeps= 0;

int ledPin = 4;   
TMP100 tmp100;  // temperature sensor high about 2 degrees C
#define TEMP_FUDGE (0<<4) // correction for temperature

MySensor gw;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
MyMessage distmsg(DIST_CHILD_ID, V_DISTANCE);
MyMessage tempMsg(TEMP_CHILD_ID, V_TEMP);  // temp measured by TMP100 
int lastDist;
boolean metric = true; 

void setup()  
{ 
  pinMode(ledPin, OUTPUT);   
  gw.begin(NULL, NODE_ID);

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Garage Door Sensor", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  gw.present(DIST_CHILD_ID, S_DISTANCE);
  gw.present(TEMP_CHILD_ID, S_TEMP);
  boolean metric = gw.getConfig().isMetric;
}

void loop()      
{    
  int t100temp; 
  int dist = metric?sonar.ping_cm():sonar.ping_in();
  Serial.print("Ping: ");
  Serial.print(dist); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println(metric?" cm":" in");
 
 // blip the LED so we know the node is working 
  digitalWrite(ledPin, HIGH);   // sets the LED on
  delay(50);                  // waits for a second
  digitalWrite(ledPin, LOW);    // sets the LED off
  
  if ((abs(dist - lastDist) > DIST_DELTA) || (nsleeps ==REPORT_COUNT)) {
      gw.send(distmsg.set(dist));
      lastDist = dist;
  }
  t100temp= tmp100.readTemperature() - TEMP_FUDGE; // tmp100 sends 7.4 integer.fraction format

  float Temp=(float)(t100temp&0xf)/16;
  Temp+=(float)(t100temp>>4);
  Serial.print("Temp=");
  Serial.println(Temp);
  if (nsleeps ==REPORT_COUNT) {
    gw.send(tempMsg.set(Temp,1));   // 1 means 1 decimal
  }
  ++nsleeps;
  if (nsleeps > REPORT_COUNT) nsleeps=0;
  
  gw.sleep(SLEEP_TIME);
}


