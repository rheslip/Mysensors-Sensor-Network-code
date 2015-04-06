/*
 * GPSClock.pde
 * Clock synced to a GPS based on example from Time library and GLCD example
 * also has a tmp100 I2C temp sensor and a BMP170 (works with BMP085 driver) pressure/temp sensor
 * Aug 2014 - modded for NRF radio so it can be added to a sensor network
 * GPS is still on the board but time comes from the network now
 * note that I removed MISO from the LCD since its not used and seems to cause problems with the NRF radio
 */


#include <TFT.h>  // Arduino LCD library
#include <SPI.h>
#include <Wire.h>
#include <BMP085.h>
#include "TMP100.h"
#include <Time.h>
#include <Timezone.h>
#include <TinyGPS.h>       //http://arduiniana.org/libraries/TinyGPS/
#include <MySensor.h>
#include <MyNodes.h>

// stuff for the NRF24L01 radio

#define NODE_ID LCDGPSCLOCK_NODE
#define RADIO_CE 31     // mighty 1284 pinout, DON'T USE sanguino
#define RADIO_CS 30

#define BARO_CHILD LCDGPSCLOCK_BARO_CHILD  // child nodes for this sensor
#define TEMP_CHILD LCDGPSCLOCK_TEMP_CHILD
#define EXT_TEMP_CHILD 
#define EXT_RH_CHILD LCDGPSCLOCK_EXT_RH_CHILD


MySensor gw(RADIO_CE,RADIO_CS);

MyMessage tempMsg(LCDGPSCLOCK_TEMP_CHILD, V_TEMP);  // temp measured by TMP100 - inside temp near clock
MyMessage ExttempMsg(LCDGPSCLOCK_EXT_TEMP_CHILD, V_TEMP);  // external temp from RR3 remote
MyMessage pressureMsg(LCDGPSCLOCK_BARO_CHILD, V_PRESSURE);  // BMP180 pressure
MyMessage ExtRHMsg(LCDGPSCLOCK_EXT_RH_CHILD, V_HUM);  // external humidity from RR3 remote

// TFT stuff
// pin definition for the Uno
//#define cs   10
//#define dc   9
//#define rst  2  

// pin definition for my 1284 LCD clock board
#define cs   0
#define rst  1  
#define Backlight 13

#define BACKLIGHTDAY 255  // backlight settings for day and night
#define BACKLIGHTNIGHT 10

// pin definition for my 3.3v proto board
//#define cs   2
//#define rst  3  

// pin definition for the Leonardo
// #define cs   7
// #define dc   0
// #define rst  1 

// Assign human-readable names to some common 16-bit color values:
// fidus code:	theColourRed = 0xffe0, theColourGreen = 0xf81f, theColourBlue = 0x07ff
// the renasys controller inverts the colors and swaps red and blue
#define	BLACK   0xFFFF
#define	BLUE    0xffe0
#define	RED    0x07ff
#define	GREEN   0xf81f
#define CYAN    (BLUE & GREEN)
#define MAGENTA (BLUE & RED)
#define YELLOW  (RED & GREEN)
#define WHITE   0x0000

#define TIME_R  0  // truly display inverts colors
#define TIME_G  0
#define TIME_B  0

#define GRID_R  200
#define GRID_G  200
#define GRID_B  180

#define EXTTEMPCOLOR GREEN
#define EXTRHCOLOR MAGENTA

#define DSTSWITCH 14   // switch to gnd for DST/normal time



//#include <NewSoftSerial.h>  //http://arduiniana.org/libraries/newsoftserial/
// GPS and NewSoftSerial libraries are the work of Mikal Hart

BMP085 bmp;  // create new sensor objects
TMP100 tmp100;
TinyGPS gps; 
//NewSoftSerial serial_gps =  NewSoftSerial(3, 2);  // receive on pin 3


time_t prevDisplay = 0; // when the digital clock was displayed

bool timesync=0;

//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev
time_t utc, local;

long timer;    // keeps track of time requests
/*
// custom font
#include "font5x7.h"
#define CUSTOMFONT font5x7
#define FONTWIDTH 5
#define FONTHEIGHT 8
#define FONTBYTES 5  // number of bytes in each character
#define STARTCHAR  0 // first character in the font table
*/

// custom font
#include "font20x28.h"
#define CUSTOMFONT font20x28
#define FONTWIDTH 20
#define FONTHEIGHT 28
#define FONTHBYTES 4  // number of bytes in height
#define FONTBYTES FONTWIDTH*FONTHBYTES  // number of bytes in each character
#define STARTCHAR  '0' // first character in the font table

// define the graph ranges
#define GRAPHX_START 0
#define GRAPHY_START 85
#define GRAPHX_END 320
#define GRAPHY_END 206

#define PRESSUREHIGH 1035  
#define PRESSURELOW 990 

// tmp100 sends 7.4 integer.fraction format 
// define the graph ranges in terms of sensor values
// 12/30/13 changed indoor temp to same scale as outdoor
#define TEMPHIGH 30<<4  
#define TEMPLOW (-30)*16

// define the external temp graph ranges
#define EXTTEMPHIGH 30*10+500  // raw sensor values 
#define EXTTEMPLOW -30*10+500

// define the external RH graph ranges
#define EXTRHHIGH 100  // raw sensor values 
#define EXTRHLOW 0

#define PRESSURE_FUDGE 12    // adjustment required on sensor
#define HUMIDITY_FUDGE 10   // RH is about 10% higher than actual
#define HISTORY_SIZE 320 // size of data history

int TempHistory[HISTORY_SIZE];
int ExtTempHistory[HISTORY_SIZE];
int ExtRHHistory[HISTORY_SIZE];
int PressureHistory[HISTORY_SIZE];
unsigned int TempIndex, ExtTempIndex,ExtRHIndex,PressureIndex;

// variables for 433mhz receiver which listens to outside temp & Humidity sensor

const int RR3data =  2;      //  radio data input pin - set up as an interrupt

volatile uint8_t bitcount = 0;  // current number of bits received
volatile unsigned long bittime; // time of last edge
volatile unsigned long rrdata; // received data in progress
unsigned int RR3temp = (10*10)+500;  // external temp from radio - temp in degrees C/10, offset of 50 degrees ie .1C resolution. set it to some reasonable starting value
unsigned int RR3rh = 50;  // external RH from radio 0-100 binary
// note the code doesn't actually use the flag below. We just initialize to reasonable values and they get updated asynchronously
int RR3ready=0;  // radio data ready flag

// create an instance of the TFT library
TFT t1 = TFT(cs,  rst);

void setup()
{
  Serial.begin(9600);  // console out
  Serial1.begin(4800); // GPS input
  Serial.println("Initializing...");  
  pinMode(cs, OUTPUT);      // sets the digital pin as output
  digitalWrite(cs,1);   // inactive state
  pinMode(RADIO_CS, OUTPUT);      // sets the digital pin as output
  digitalWrite(RADIO_CS,1);   // inactive state

// this works best if TFT starts first, then the NRF code which changes the SPI config a bit   
  t1.begin();
  t1.fillScreen(BLACK); 
  bmp.begin();  
  pinMode(RR3data, INPUT); 
  attachInterrupt(2, bitreader, CHANGE); // set up ISR for radio

  pinMode(Backlight, OUTPUT);      // sets the digital pin as output
  analogWrite(Backlight,BACKLIGHTDAY );   // sets the backlight LED 

  pinMode(DSTSWITCH, INPUT); 
  
/*
  Serial.println("Waiting for GPS time ... ");
  setSyncInterval(10);  // sync to GPS every 10 seconds
  setSyncProvider(gpsTimeSync);
  */

    // Initialize library 
  Serial.println("Starting network ");
//  gw.begin(NULL, NODE_ID);

  // Initialize library and add callback for incoming messages
  gw.begin(incomingMessage, NODE_ID, false);
  
  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("GPSLCDClock", "1.0");

  // Request time from controller. 
  gw.requestTime(receiveTime);  

  gw.present(BARO_CHILD, S_BARO);  // register sensors
  gw.present(TEMP_CHILD, S_TEMP);
  
  // throw up a display to start  
  drawGrid();
  initTempHistory(); // intialize the temperature history data
  drawTempGraph(); // draw
  initExtTempHistory(); // intialize the external temperature history data
  drawExtTempGraph(); // draw
  initExtRHHistory(); // intialize the external relative humidity history data
  drawExtRHGraph(); // draw
  initPressureHistory(); // intialize the pressure history data
  drawPressureGraph(); // draw
  digitalClockDisplay();  // update once per minute 
  tmp100Display();  // tmp100 temperature
  ExtTempDisplay();  // external temperature
  ExtRHDisplay();  // external temperature
  pressureDisplay(); // BMP170 pressure
  timer=millis(); 
  Serial.println("Initialization complete"); 
}

void loop()
{
  int lastPressure,lastExtRH, lastIntTemp, lastExtTemp;
/*  while (Serial1.available()) 
  {
    gps.encode(Serial1.read()); // process gps messages
  }
//  if(timeStatus()!= timeNotSet) // seems this is never true
*/

  if ((millis()-timer) > 300000) {  // sync every few minutes
    gw.requestTime(receiveTime);  // request time from network 
    timer=millis();
  } 
  gw.process();  // process network messages
  utc = now();
  local = myTZ.toLocal(utc, &tcr);
  
  {
     if( local != prevDisplay) //update the display only if the time has changed
     {
       prevDisplay = local;
       if (second(prevDisplay) == 0) { // update sensors once per minute

         if (minute(prevDisplay)%10 ==0) { // update the charts every 10 minutes
           drawGrid();
           updateTempHistory();
           drawTempGraph();
           updateExtTempHistory();
           drawExtTempGraph();
           updateExtRHHistory();
           drawExtRHGraph();
           updatePressureHistory();
           drawPressureGraph();
         }
         digitalClockDisplay();  // update once per minute 
         tmp100Display();  // tmp100 temperature
         ExtTempDisplay();
         ExtRHDisplay();
         pressureDisplay(); // BMP170 pressure
         // send sensor data to network
         updateIntTempSensor();
         updateExtTempSensor();
         updatePressureSensor();
         updateExtRHSensor();
       }
     }

  
     int thishour=hour(local);
     if ((thishour >22) || (thishour< 6)) analogWrite(Backlight,BACKLIGHTNIGHT);   // sets the backlight to night mode 
     else analogWrite(Backlight,BACKLIGHTDAY);   // sets the backlight to day mode 
  }	 
}

void digitalClockDisplay(){
  // digital clock display of the time
   int hour; 
/*  Serial.print(  hourFormat12());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
*/
   time_t t = local; // store the current time in time variable t 
   hour=hourFormat12(t);
//   int thishour=hourFormat12(t);
   
 //  t1.fillScreen(BLACK); 
   t1.fillRect(20, 0, 300, 80, BLACK);
   t1.setTextColor(t1.newColor(TIME_R,TIME_G,TIME_B));    
   t1.setTextSize(10);
   t1.setCursor(20, 0); // 
   
 //   t1.print(("%02d:%02d:%02d "), hourFormat12(t), minute(t), second(t));
//   if (hourFormat12(t)<10) t1.print(' ');
//   t1.print(hourFormat12(t));
//   t1.print(":");  
//   if (minute(t)<10) t1.print('0');
//   t1.print(minute(t));
   if (hour>9) drawCustomChar(50,70,hour/10+'0',WHITE,BLACK,2);
   drawCustomChar(100,70,hour%10+'0',WHITE,BLACK,2);
   drawCustomChar(180,70,minute(t)/10+'0',WHITE,BLACK,2);
   drawCustomChar(230,70,minute(t)%10+'0',WHITE,BLACK,2);
   t1.fillCircle(160,20,5,WHITE); // draw the colon
   t1.fillCircle(160,55,5,WHITE); // draw the colon    
 //  t1.print(":"); 
 //  t1.setTextSize(3);
//   t1.print(second(t));

   
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

// display the external temperature
// note that we can't request this reading, it comes in asynchronously every 3 minutes from the external wireless sensor

void ExtTempDisplay(void) {
  int temp=(int)RR3temp-500;  // convert to int since it can go negative
  t1.fillRect(85, 215, 180, 239, BLACK);
  t1.setTextColor(EXTTEMPCOLOR);    
  t1.setTextSize(3);
  t1.setCursor(85, 215); // 
  t1.print(temp/10);
  t1.print('.');
  t1.print((abs)(temp%10)); // corrects negative fraction
  t1.setTextSize(1);
  t1.print('C');  
}

// send the external temperature value to the network controller

void updateExtTempSensor(void) {
  int temp=RR3temp-500;  // convert to int since it can go negative
  float Temp;
  Temp=((float)(temp%10))/10; // fraction
  Temp+=(float)(temp/10);;
  gw.send(ExttempMsg.set(Temp,1));   // 1 means 1 decimal
}

// display the external humidity
// note that we can't request this reading, it comes in asynchronously every 3 minutes from the external wireless sensor

void ExtRHDisplay(void) {
  t1.fillRect(190, 215, 260, 239, BLACK);
  t1.setTextColor(EXTRHCOLOR);    
  t1.setTextSize(3);
  t1.setCursor(190, 215); // 
  t1.print(RR3rh);
  t1.setTextSize(1);
  t1.print('%');
}

// send the external RH value to the network controller

void updateExtRHSensor(void) {
  float humidity= (float)RR3rh;
  gw.send(ExtRHMsg.set(humidity,0));   // 0 means 0 decimals
}

void tmp100Display(void) {
  // display TMP100 sensor value
  unsigned int t100temp;
  t1.fillRect(0, 215, 112, 239, BLACK);
  t1.setTextColor(RED);    
  t1.setTextSize(3);
  t1.setCursor(0, 215); // 
  //    tempC= (int) (bmp.readTemperature()*10); // returns float 
  t100temp= tmp100.readTemperature(); // tmp100 sends 7.4 integer.fraction format
//  t1.print("%d.%dC",t100temp>>4,t100temp&0xf);
  t1.print(t100temp>>4);
  t1.print('.');
  t1.print((t100temp&0xf)/16);
  t1.setTextSize(1);
  t1.print('C');  
}

// send the TMP100 sensor value to the network controller

void updateIntTempSensor(void) {
  // display TMP100 sensor value
  unsigned int t100temp;
  float Temp;
  t100temp= tmp100.readTemperature(); // tmp100 sends 7.4 integer.fraction format
//  t1.print("%d.%dC",t100temp>>4,t100temp&0xf);
  Temp=(float)(t100temp&0xf)/16;
  Temp+=(float)(t100temp>>4);
  gw.send(tempMsg.set(Temp,1));   // 1 means 1 decimal
}

void pressureDisplay(void) {
  // display BMP170 pressure in kilopascals
  unsigned long pressure;
  t1.fillRect(240, 215, 319, 239, BLACK);
  t1.setCursor(240, 215); // 
  t1.setTextColor(YELLOW);    
  t1.setTextSize(3);
  pressure= bmp.readPressure(); // returns int32
//  t1.print("%ld.%ldkPa",pressure/1000,(pressure/100)%10);
  pressure=pressure/100+PRESSURE_FUDGE; // fudge factor needed for my sensor
  if (pressure <1000) t1.print(' '); // keep spacing
  t1.print(pressure); 
 // t1.print('.');
 // t1.print((pressure/100)%10);
//  t1.setCursor(278, 222); // 
  t1.setCursor(314, 215); //  print units vertical because they don't fit horizontal
  t1.setTextSize(1);
  t1.print('h');
  t1.setCursor(314, 223); //   
  t1.print('P');
  t1.setCursor(314, 231); //   
  t1.print('a');
}

void updatePressureSensor(void) {
  // display BMP170 pressure in kilopascals
  unsigned long p_reading;
  float pressure;
  p_reading= bmp.readPressure(); // returns int32
//  t1.print("%ld.%ldkPa",pressure/1000,(pressure/100)%10);
  pressure=float(p_reading/100+PRESSURE_FUDGE); // fudge factor needed for my sensor
  gw.send(pressureMsg.set(pressure, 0));
}


// ISR that checks bit timing and saves RRS3 radio data
// interrupt on change mode

void bitreader() 
{
 if (digitalRead(RR3data)==1) 
 { //start of a bit
   if ((micros()-bittime) > 10000) bitcount=0; // long time since last symbol, must be a new packet
   bittime=micros();    // record time of the symbol start
 } else 
 {         // input level is low
   bittime=micros()-bittime; // calc high time of symbol
   if ((bittime > 1500) && (bittime < 2500)) 
   { // 2ms pulse is a zero
     rrdata=(rrdata<<1) & 0xfffffffe;
     ++bitcount;
   } else if ((bittime > 5500) && (bittime < 6500)) 
   { // 6ms pulse is a 1
     rrdata=(rrdata<<1) | 1;
     ++bitcount;
   } else bitcount=0; // invalid symbol, start again
   bittime=micros(); // save time of last edge
   if (bitcount ==36) 
   { // we have a complete packet - note that the first packet is 40 bits and will fail the validity check 
     bitcount=0;  // restart counter for next packet
     if ((rrdata & 0xFE000070) == 0x12000010) // these bits are fixed, use as a validity check
     { // we have a good packet
        RR3temp= (rrdata >> 14) & 0x7ff;
        RR3rh= (rrdata >> 7) & 0x7f;  
        if (RR3rh > HUMIDITY_FUDGE) RR3rh-= HUMIDITY_FUDGE; // sensor reads high
        if (RR3rh >99) RR3rh=99; // keep range 0-99 to keep spacing on display
        RR3ready=1;    // flag data available
     }     
   }   
 }
}  



// initialize the temperature history to the current temperature
void initTempHistory() {
  unsigned int i;
 // tmp100 sends 7.4 integer.fraction format  
  for (i=0;i<HISTORY_SIZE;++i) TempHistory[i]=constrain((int)tmp100.readTemperature(),TEMPLOW,TEMPHIGH);
  TempIndex=0; // update pointer
}

void updateTempHistory() {
    TempHistory[TempIndex++]=constrain((int)tmp100.readTemperature(),TEMPLOW,TEMPHIGH);
   if (TempIndex>=HISTORY_SIZE) TempIndex=0; 
}

void drawTempGraph(){
  unsigned int i, index,lasty,screeny;
//  t1.fillRect(0, 81, 320, 199, BLACK);
  index=TempIndex; // start with the oldest data
  lasty=map(TempHistory[index],TEMPHIGH,TEMPLOW,GRAPHY_START,GRAPHY_END);
  if (++index >=HISTORY_SIZE) index=0;
  for (i=0;i<=(HISTORY_SIZE-2);++i){
    screeny=map(TempHistory[index],TEMPHIGH,TEMPLOW,GRAPHY_START,GRAPHY_END);
//Serial.println(screeny);
    t1.drawLine(i, lasty, i+1, screeny, RED);  // draw line segments - there is no drawpixel
    lasty=screeny;
    if (++index >=HISTORY_SIZE) index=0;
  }  
}


// initialize the external temperature history to the initial sensor value
void initExtTempHistory() {
  unsigned int i;
 // sensor sends degrees C*10 ie .1C resolution 
  for (i=0;i<HISTORY_SIZE;++i) ExtTempHistory[i]=RR3temp;
  ExtTempIndex=0; // update pointer
}

void updateExtTempHistory() {
   ExtTempHistory[ExtTempIndex++]=constrain(RR3temp,EXTTEMPLOW,EXTTEMPHIGH);
   if (ExtTempIndex>=HISTORY_SIZE) ExtTempIndex=0; 
}

void drawExtTempGraph(){
  unsigned int i, index,lasty,screeny;
//  t1.fillRect(0, 81, 320, 199, BLACK);
  index=ExtTempIndex; // start with the oldest data
  lasty=map(ExtTempHistory[index],EXTTEMPHIGH,EXTTEMPLOW,GRAPHY_START,GRAPHY_END);
  if (++index >=HISTORY_SIZE) index=0;
  for (i=0;i<=(HISTORY_SIZE-2);++i){
    screeny=map(ExtTempHistory[index],EXTTEMPHIGH,EXTTEMPLOW,GRAPHY_START,GRAPHY_END);
//Serial.println(screeny);
    t1.drawLine(i, lasty, i+1, screeny, EXTTEMPCOLOR);  // draw line segments - there is no drawpixel
    lasty=screeny;
    if (++index >=HISTORY_SIZE) index=0;
  }  
}

// initialize the external RH history to the current value
void initExtRHHistory() {
  unsigned int i;
  for (i=0;i<HISTORY_SIZE;++i) ExtRHHistory[i]=constrain(RR3rh,EXTRHLOW,EXTRHHIGH);
  ExtRHIndex=0; // update pointer
}

void updateExtRHHistory() {
    ExtRHHistory[ExtRHIndex++]=constrain(RR3rh,EXTRHLOW,EXTRHHIGH);
   if (ExtRHIndex>=HISTORY_SIZE) ExtRHIndex=0; 
}

void drawExtRHGraph(){
  unsigned int i, index,lasty,screeny;
//  t1.fillRect(0, 81, 320, 199, BLACK);
  index=ExtRHIndex; // start with the oldest data
  lasty=map(ExtRHHistory[index],EXTRHHIGH,EXTRHLOW,GRAPHY_START,GRAPHY_END);
  if (++index >=HISTORY_SIZE) index=0;
  for (i=0;i<=(HISTORY_SIZE-2);++i){
    screeny=map(ExtRHHistory[index],EXTRHHIGH,EXTRHLOW,GRAPHY_START,GRAPHY_END);
//Serial.println(screeny);
    t1.drawLine(i, lasty, i+1, screeny, EXTRHCOLOR);  // draw line segments - there is no drawpixel
    lasty=screeny;
    if (++index >=HISTORY_SIZE) index=0;
  }  
}


// initialize the pressure history to the current pressure
void initPressureHistory() {
  unsigned int i, pressurenow;
  pressurenow=constrain((unsigned int)(bmp.readPressure()/100)+PRESSURE_FUDGE, PRESSURELOW,PRESSUREHIGH);
  for (i=0;i<HISTORY_SIZE;++i) PressureHistory[i]=pressurenow;
  PressureIndex=0; // update pointer
}

void updatePressureHistory(){
   PressureHistory[PressureIndex++]=constrain((unsigned int)(bmp.readPressure()/100) +PRESSURE_FUDGE,PRESSURELOW,PRESSUREHIGH); // returns int32;
   if (PressureIndex>=HISTORY_SIZE) PressureIndex=0; 
}

void drawPressureGraph(){
  unsigned int i, index,lasty,screeny;
//  t1.fillRect(160, 81, 319, 199, BLACK);
  index=PressureIndex; // start with the oldest data
  lasty=map(PressureHistory[index],PRESSUREHIGH,PRESSURELOW,GRAPHY_START,GRAPHY_END);
  if (++index >=HISTORY_SIZE) index=0;
  for (i=0;i<=(HISTORY_SIZE-2);++i){
    screeny=map(PressureHistory[index],PRESSUREHIGH,PRESSURELOW,GRAPHY_START,GRAPHY_END);
//Serial.println(screeny);
    t1.drawLine(i, lasty, i+1, screeny, YELLOW);  // draw line segments - there is no drawpixel
    lasty=screeny;
    if (++index >=HISTORY_SIZE) index=0;
  }  
}

// draw the graph grid
void drawGrid() {
  int  x, y, w = (GRAPHX_END -GRAPHX_START), h = GRAPHY_END-GRAPHY_START;
  t1.fillRect(GRAPHX_START, GRAPHY_START, GRAPHX_END,GRAPHY_END, BLACK);
  t1.drawLine(GRAPHX_START, GRAPHY_START, GRAPHX_END,GRAPHY_START, WHITE);
  for(y=GRAPHY_START; y<(GRAPHY_START+h); y+=10) t1.drawFastHLine(GRAPHX_START, y, w, t1.newColor(GRID_R,GRID_G,GRID_B)); // 5 ticks approx 2 hPa
  for(x=GRAPHX_START; x<w; x+=12) t1.drawFastVLine(x, GRAPHY_START, h, t1.newColor(GRID_R,GRID_G,GRID_B)); // 6 ticks = 1 hour
  t1.drawLine(GRAPHX_START, GRAPHY_START, GRAPHX_END,GRAPHY_START, WHITE);
  t1.drawLine(GRAPHX_START, GRAPHY_END, GRAPHX_END,GRAPHY_END, WHITE);
}

// draw a custom character - this is copied from the
// adafruit library and modded for different size fonts
// here the font table is only numbers
// note that the fonts I'm using are flipped relative to Adafruit's
// so the code draws the font above the starting x,y point
// note also that the adafruit font files are set up with bytes in the Y direction ie you step vertically as you decode each pixel
// ** note this code needed a lot of little hacks to accomodate the 20x28 font which is stored in kind of a strange way

void drawCustomChar(int16_t x, int16_t y, unsigned char c,
			    uint16_t color, uint16_t bg, uint8_t size) {
/*
  if((x >= _width)            || // Clip right
     (y >= _height)           || // Clip bottom
     ((x + FONTWIDTH * size - 1) < 0) || // Clip left
     ((y + FONTHEIGHT * size - 1) < 0))   // Clip top
    return;
    */
   c=c-STARTCHAR;  // adjust for start of font table

// **** this needs to handle multi byte widths
  for (int8_t i=0; i<FONTWIDTH; i++ ) {
    uint8_t line;
    uint8_t nbytes, npixels;
    /*
    if (i == FONTWIDTH) // puts in the space, I don't care
      line = 0x0;
    else */
      npixels=0;
      nbytes=FONTHBYTES-1;
      line = pgm_read_byte(CUSTOMFONT+(c*FONTBYTES)+i*FONTHBYTES+nbytes); // read first byte 
//    for (int8_t j = 0; j<FONTHEIGHT; j++) {
    for (int8_t j = 0; j>-(FONTHEIGHT+4); j--) {  // flips the Y direction
      if (line & 0x1) {
        if (size == 1) // default size
          t1.drawPixel(x+i, y+j, color);
        else {  // big size
          t1.fillRect(x+(i*size), y+(j*size), size, size, color);
        } 
      } else if (bg != color) {
        if (size == 1) // default size
         t1.drawPixel(x+i, y+j, bg);
        else {  // big size
          t1.fillRect(x+i*size, y+j*size, size, size, bg);
        } 	
      }
      line >>= 1;
      if (++npixels ==8) { // read next byte of font
        npixels=0;
        --nbytes;
        line = pgm_read_byte(CUSTOMFONT+(c*FONTBYTES)+i*FONTHBYTES+nbytes); 
      }
    }
  }
}

 /* 
time_t gpsTimeSync(){
  //  returns time if avail from gps, else returns 0
  unsigned long fix_age = 0 ;
  gps.get_datetime(NULL,NULL, &fix_age);
  if(fix_age < 2000)
    return gpsTimeToArduinoTime(); // return time only if updated recently by gps  
  return 0;
}

time_t gpsTimeToArduinoTime(){
  // returns time_t from gps date and time with the given offset hours
  tmElements_t tm;
  int year;
  gps.crack_datetime(&year, &tm.Month, &tm.Day, &tm.Hour, &tm.Minute, &tm.Second, NULL, NULL);
  tm.Year = year - 1970; 
  time_t time = makeTime(tm);
  return time + (offset * SECS_PER_HOUR);
}
*/

// This is called when a new network time value is received

void receiveTime(unsigned long time) {
  // Ok, set incoming time 
  setTime(time);
  timesync=true;
}

// display messages from controller
// the line number and color is encoded in the sensor ID

#define YTOP 90
#define LINEHEIGHT 24


void incomingMessage(const MyMessage &message) {
  int y=YTOP;
  // We only expect one type of message from controller. But we better check anyway.
  if (message.type==V_VAR1) {
    switch (message.sensor & DISPLAYLINEMASK) { // the sensor ID tells us what to do with the message
      case LINE1:
        y=YTOP;
        break;
      case LINE2:
        y=YTOP+LINEHEIGHT;
        break;
      case LINE3:
        y=YTOP+LINEHEIGHT*2;
        break;
      case LINE4:
        y=YTOP+LINEHEIGHT*3;
        break;
      case LINE5:
        y=YTOP+LINEHEIGHT*4;
        break;
      case LINE6:
        y=YTOP+LINEHEIGHT*5;
        break;        
    }  
     t1.fillRect(0, y, 319, LINEHEIGHT, BLACK);  // erase old message 
     switch ((message.sensor & DISPLAYCOLORMASK)) { // display according the color bits
      case COLORBLACK: 
       t1.setTextColor(BLACK);  // allows sending a blank message ?
       break;
      case COLORBLUE: 
       t1.setTextColor(BLUE);
       break; 
      case COLORRED: 
       t1.setTextColor(RED);
       break; 
      case COLORGREEN: 
       t1.setTextColor(GREEN);
       break; 
      case COLORCYAN: 
       t1.setTextColor(CYAN);
       break; 
      case COLORMAGENTA: 
       t1.setTextColor(MAGENTA);
       break;
      case COLORYELLOW: 
       t1.setTextColor(YELLOW);
       break;
      case COLORWHITE: 
       t1.setTextColor(WHITE);
       break;  
      default:
        t1.setTextColor(WHITE);
     }      
     t1.setTextSize(2);
     t1.setCursor(0, y); // 
     t1.print(message.getString());
   } 
}
