#include <SPI.h>
#include <MySensor.h> 
#include <MyNodes.h>

// mysensor network stuff

#define NODE_ID FREEZER_NODE
#define CHILD_ID_FREEZER1     FREEZER_NODE_FREEZER1
#define CHILD_ID_FREEZER2     FREEZER_NODE_FREEZER2

#define LEDPIN    4
#define V1_PIN    A0    // freezer temps detected using a pullup and a thermistor
#define V2_PIN    A1

unsigned long SLEEP_TIME = 140000; // Sleep time between reads (in milliseconds)

MySensor gw;

boolean metric = true; 

MyMessage msgTemp1(CHILD_ID_FREEZER1, V_TEMP);
MyMessage msgTemp2(CHILD_ID_FREEZER2, V_TEMP);


void setup()  
{ 
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT);  
  pinMode(V1_PIN, INPUT_PULLUP);
  pinMode(V2_PIN, INPUT_PULLUP);     
  gw.begin(NULL, NODE_ID);
//  analogReference(INTERNAL);       // use the 1.1v reference - external 1.5m/100k divider
  // Send the Sketch Version Information to the Gateway
  gw.sendSketchInfo("Freezer Alarm", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_FREEZER1, S_TEMP);
  gw.present(CHILD_ID_FREEZER2, S_TEMP);
  
  metric = gw.getConfig().isMetric;

}

void loop()      
{  
  
 unsigned int v1 = analogRead(V1_PIN);
 gw.send(msgTemp1.set(v1));
 Serial.print("Analog 0=");
 Serial.println(v1);
 
 unsigned int v2 = analogRead(V2_PIN);
 gw.send(msgTemp2.set(v2));
 Serial.print("Analog 1=");
 Serial.println(v2);
 digitalWrite(LEDPIN, LOW);   // sets the LED on
 delay(100);                  // waits for a second
 digitalWrite(LEDPIN, HIGH);    // sets the LED off

 gw.sleep(SLEEP_TIME); //sleep a bit
}


