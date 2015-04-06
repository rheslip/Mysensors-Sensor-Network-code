// sump sensor node aug 2014
// R Heslip
// this node monitors the sump battery backup switch, the battery voltage,
// temperature and humidity
// Oct/14 added water depth monitoring to detect pump failure earlier, can slso detect
// case of pump switch stuck which happened recently
// the controller code decides what to do with all the readings

#include <SPI.h>
#include <MySensor.h> 
#include <MyNodes.h>
#include <DHT.h>  
#include <NewPing.h>

// mysensor network stuff

#define NODE_ID SUMP_NODE
#define CHILD_ID_HUM     SUMP_NODE_HUMIDITY
#define CHILD_ID_TEMP    SUMP_NODE_TEMPERATURE
#define CHILD_ID_SUMP_SWITCH    SUMP_NODE_SUMP_SWITCH
#define DIST_CHILD_ID SUMP_NODE_DIST

#define HUMIDITY_SENSOR_DIGITAL_PIN 3
#define HUMIDITY_SENSOR_POWER_PIN 4  // power with a port so we can shut it down
#define BATTERY_PIN  A3    // 12V battery power source
#define SUMP_SWITCH_PIN A2  // goes to 12V when sump level switch closes

// added water depth sensor oct 2014
// note we measure distance to the water surface - the controller converts that to water depth
#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 50 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

unsigned long SLEEP_TIME = 3000; // Sleep time between reads (in milliseconds)
#define CHECK_IN_INTERVAL 30  // time in SLEEP_TIMEs to do an update,otherwise report only changes
#define BATT_DIVIDER 16.5  // scale factor for external resistor dividers - corrected
#define BATT_FULL  12.7    // approximate voltage of freshly charged battery
#define BATT_EMPTY 11.0    // voltage below which battery is effectively dead
#define TEMPFUDGE 0  // sensor correction factors
#define HUMIDITYFUDGE 30  // humidity sensor is not accurate at all
#define SUMP_SWITCH_THRESHOLD 500 // reading that says switch is closed - approx 8v

MySensor gw;
DHT dht;
float lastTemp;
float lastHum;
int oldBatteryPcnt = 0;
boolean metric = true; 
int sleepcycles=CHECK_IN_INTERVAL;  // makes sure we report at least once on power up
unsigned int avgdepth=0;   // averaged water distance reading
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msg(CHILD_ID_SUMP_SWITCH,V_TRIPPED);
MyMessage distmsg(DIST_CHILD_ID, V_DISTANCE);

void setup()  
{ 
  gw.begin(NULL, NODE_ID);
  pinMode(HUMIDITY_SENSOR_POWER_PIN, OUTPUT);      // power output pin
  digitalWrite(HUMIDITY_SENSOR_POWER_PIN,1);   // apply power
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN); 
  analogReference(INTERNAL);       // use the 1.1v reference - external 1.5m/100k divider
  // Send the Sketch Version Information to the Gateway
  gw.sendSketchInfo("Sump Monitor", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  gw.present(CHILD_ID_HUM, S_HUM);
  gw.present(CHILD_ID_TEMP, S_TEMP);
  gw.present(DIST_CHILD_ID, S_DISTANCE);
  
  metric = gw.getConfig().isMetric;

}

void loop()      
{  
  bool report;
  if (sleepcycles++ >= CHECK_IN_INTERVAL) { // force send periodically
     report=true;
     sleepcycles=0;
  }
  else report=false;

  int dist = metric?sonar.ping_cm():sonar.ping_in();
  Serial.print("Ping: ");
  Serial.print(dist); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.println(metric?" cm":" in");
  avgdepth+=dist; // accumulate readings
  
  if (report) {
      gw.send(distmsg.set(avgdepth/CHECK_IN_INTERVAL)); // send average reading
      avgdepth=0;  // reset the average value
  }
  
  digitalWrite(HUMIDITY_SENSOR_POWER_PIN,1);   // apply power
  delay(dht.getMinimumSamplingPeriod());

  float temperature = dht.getTemperature()+TEMPFUDGE;
  if (isnan(temperature)) {
      Serial.println("Failed reading temperature from DHT");
  } else if ((temperature != lastTemp) || report) {
    lastTemp = temperature;
    if (!metric) {
      temperature = dht.toFahrenheit(temperature);
    }
    gw.send(msgTemp.set(temperature, 1));
    Serial.print("T: ");
    Serial.println(temperature);
  }
  
  float humidity = dht.getHumidity()+HUMIDITYFUDGE;
  if (humidity > 99) humidity = 95; // sensor can give some wildly bad readings
  if (isnan(humidity)) {
      Serial.println("Failed reading humidity from DHT");
  } else if ((humidity != lastHum)|| report) {
      lastHum = humidity;
      gw.send(msgHum.set(humidity, 1));
      Serial.print("H: ");
      Serial.println(humidity);
  }
  digitalWrite(HUMIDITY_SENSOR_POWER_PIN,0);   // remove power to DHT sensor
  
  float battery = analogRead(BATTERY_PIN);
  battery=battery*1.1*BATT_DIVIDER/1024;     // Vref*scale factor for external divider + a correction/1024
  float batterylevel=((battery-BATT_EMPTY)/(BATT_FULL-BATT_EMPTY))*100; // we report a percentage of useful battery voltages - not relative to zero volts which is rather useless
  if (batterylevel>100) batterylevel=100;
  int batteryPcnt=(int)batterylevel;

  if ((abs(oldBatteryPcnt - batteryPcnt) >1) || report) { // report change if 2% or more - filters A/D noise
      Serial.print("Battery voltage: ");
      Serial.print(battery);
      Serial.print(" ");
      Serial.print(batteryPcnt);
      Serial.println('%');
     // Power up radio after sleep
     gw.sendBatteryLevel(batteryPcnt);
     oldBatteryPcnt = batteryPcnt;
  }

  int sumpswitch = analogRead(SUMP_SWITCH_PIN);  // sump switch read via analog
  bool sumpalarm = (sumpswitch >SUMP_SWITCH_THRESHOLD);
  
  if (sumpalarm || report) { 
     Serial.print("Sump alarm switch: ");
     Serial.println(sumpalarm);
     gw.send(msg.set(sumpalarm));
  }
  gw.sleep(SLEEP_TIME); //sleep a bit
}


