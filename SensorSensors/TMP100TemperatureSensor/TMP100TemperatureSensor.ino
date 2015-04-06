// TMP100 temperature 
// sensor node Nov 2014
// R Heslip
// this node monitors the 
// temperature in the attic

#include <MySensor.h> 
#include "MyNodes.h"
#include <SPI.h>
#include <Wire.h>
#include <TMP100.h>

// mysensor network stuff

#define NODE_ID ATTIC_NODE
#define CHILD_ID_HUM     ATTIC_NODE_HUMIDITY
#define CHILD_ID_TEMP    ATTIC_NODE_TEMPERATURE

#define LEDPIN 4

unsigned long SLEEP_TIME = 120000; // Sleep time between reads (in milliseconds)
#define CHECK_IN_INTERVAL 15  // time in SLEEP_TIMEs to do an update,otherwise report only changes
#define TEMPFUDGE 0  // sensor correction factors

// his node runs directly off two lithum cells - 3.6v when fresh
#define BATT_FULL 3600  // readings in mv
#define BATT_EMPTY 3000  // lithiums are hammered at 1.5v

MySensor gw;
TMP100 tmp100;  // temperature sensor 
float lastTemp;
int oldBatteryPcnt = 0;
boolean metric = true; 
int sleepcycles=CHECK_IN_INTERVAL;  // makes sure we report at least once on power up


MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void setup()  
{ 
  Serial.begin(9600);
  gw.begin(NULL, NODE_ID);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN,1); // turn led on
  Wire.begin();

  analogReference(INTERNAL);       // use the 1.1v reference - external 1.5m/100k divider
  // Send the Sketch Version Information to the Gateway
  gw.sendSketchInfo("Attic Monitor", "1.0");

  // Register all sensors to gw (they will be created as child devices)

  gw.present(CHILD_ID_TEMP, S_TEMP);
  
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

//  report=true; // force report
  digitalWrite(LEDPIN,1); // turn led on
  int t100temp= tmp100.readTemperature() - TEMPFUDGE; // tmp100 sends 7.4 integer.fraction format
  
  float Temp=(float)(t100temp&0xf)/16;
  Temp+=(float)(t100temp>>4);
  Serial.print("Temp=");
  Serial.println(Temp);
  if ((Temp != lastTemp)|| report) {
    gw.send(msgTemp.set(Temp, 1)); // 1 means 1 decimal
    lastTemp=Temp;
  }

  float battery = (float)readVcc();
  //battery=battery*1.1*BATT_DIVIDER/1024;     // Vref*scale factor for external divider + a correction/1024
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
  digitalWrite(LEDPIN,0); // turn led off
  gw.sleep(SLEEP_TIME); //sleep a bit
}


