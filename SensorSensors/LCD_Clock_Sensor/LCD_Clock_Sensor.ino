/*
 * RTCClock.pde
 * Clock using DS1302 RTC 
 * 1/8/14 -changed the code so it syncs the Arduino time libraries
 * vs just reading the RTC all the time. Reading the RTC and no doubt writing the LCD causes a lot of interference
 * with the 433mhz RX on the board. So we sync to the RTC every hour and update the LCD once per minute
 * also has BMP170 (works with BMP085 driver) pressure/temp sensor
 * reads 433 mhz remote temp+humidity sensor
 * more or less same code as the GPSclock but the 328 has only 2k ram so we have
 * fewer samples of history per sensor
 */


#include <TFT.h>  // Arduino LCD library
#include <SPI.h>
//#include <Wire.h>
//#include <BMP085.h>
//#include "TMP100.h"
#include <Time.h>
#include <Timezone.h>    //https://github.com/JChristensen/Timezone
#include <MySensor.h>  
#include <MyNodes.h>

// mysensor network stuff

#define NODE_ID LCDCLOCK_NODE
#define RADIO_CE 7
#define RADIO_CS 8

MySensor gw(RADIO_CE,RADIO_CS);

bool timesync=0;

//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev
time_t utc, local;

long timer;    // keeps track of time requests

// Set the appropriate digital I/O pin connections. These are the pin
// assignments for the Arduino as well for as the DS1302 chip. See the DS1302
// datasheet:
//
//   http://datasheets.maximintegrated.com/en/ds/DS1302.pdf
//const int kCePin   = 7;  // Chip Enable
//const int kIoPin   = 6;  // Input/Output
//const int kSclkPin = 5;  // Serial Clock

// Create a DS1302 object.
//DS1302 rtc(kCePin, kIoPin, kSclkPin);

// LDR to 3.3v in series with 10k provides ambient light sensor. goes to analog 0
#define LDR 0

#define SWITCH A1   // switch to gnd 

// TFT pin definition for mega328 LCD clock board
// extra defs are for software SPI mode
#define cs   10
#define rst  9  
#define mosi 11
#define sclk 13
#define dc 6      // not used but has to be defined

#define Backlight 3

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

time_t prevDisplay = 0; // when the digital clock was displayed

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


// create an instance of the TFT library
TFT t1 = TFT(cs,  rst);
// Option 1: use any pins but a little slower
//TFT t1 = TFT(cs, dc, mosi, sclk, rst);

void setup()
{

  Serial.begin(9600);  // console out
  Serial.println("Initializing...");  
  pinMode(cs, OUTPUT);      // sets the digital pin as output
  digitalWrite(cs,1);   // inactive state
//  pinMode(rst, OUTPUT);      // sets the digital pin as output
//  digitalWrite(rst,0);   // inactive state
  pinMode(RADIO_CS, OUTPUT);      // sets the digital pin as output
  digitalWrite(RADIO_CS,1);   // inactive state
//  pinMode(RADIO_CE, OUTPUT);      // sets the digital pin as output
//  digitalWrite(RADIO_CE,1);   // inactive state

// this works best if TFT starts first, then the NRF code which changes the SPI config a bit 
  t1.begin();
  t1.fillScreen(BLACK); 
  Serial.println("TFT Init done"); 
  digitalClockDisplay();
  
  pinMode(Backlight, OUTPUT);      // sets the digital pin as output
  // note - can't use pwm out because it interferes with the RR3 433mhz radio. backlight has a hi/lo setting
//  analogWrite(Backlight, 128);   // sets the backlight LED on
  digitalWrite(Backlight,1);   // turn backlight on full
  pinMode(SWITCH, INPUT_PULLUP); 

//  gw.begin(NULL,NODE_ID);  // start the radio

  // Initialize library and add callback for incoming messages
  gw.begin(incomingMessage, NODE_ID, false);

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("LEDClock", "1.0");

  // Request time from controller. 
  gw.requestTime(receiveTime);  

  timer=millis(); 
  Serial.println("Initialization complete");  
}

void loop()
{
  if ((millis()-timer) > 300000) { // sync every few minutes
    gw.requestTime(receiveTime);  // request time from network 
    timer=millis();
  } 
  gw.process();  // process network messages
  utc = now();
  local = myTZ.toLocal(utc, &tcr);
  
  if( local != prevDisplay) { //update the display only if the time has changed
       prevDisplay = local;
       if (second(prevDisplay) == 0) { // update sensors once per minute

         if ((minute(prevDisplay) ==0) || (minute(prevDisplay) ==30)) { // update the charts and history every 30 minutes
/*           drawGrid();
           updateTempHistory();
           drawTempGraph();
           updateExtTempHistory();
           drawExtTempGraph();
           updateExtRHHistory();
           drawExtRHGraph();
           updatePressureHistory();
           drawPressureGraph();
           */
        }
        digitalClockDisplay();  // update once per minute 
/*        tmpDisplay();  // BMP temperature
        ExtTempDisplay();
        ExtRHDisplay();
        pressureDisplay(); // BMP170 pressure
        */
     }
  }
  int light =analogRead(LDR); // sample ambient light
  if (light < 200) digitalWrite(Backlight,0);   // turn backlight on low  - use a bit of hysteresis so it doesn't flicker
  if (light > 300) digitalWrite(Backlight,1);   // turn backlight on high
  
}

void digitalClockDisplay(){
 
 int hour; 
  // digital clock display of the time
  /*
  Serial.print(  hourFormat12());
//  printDigits(minute());
//  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
*/
   time_t t = local;  // store the current time in time variable t 
   hour=hourFormat12(local);
   
 //  t1.fillScreen(BLACK); 
   t1.fillRect(20, 0, 300, 80, BLACK);
   t1.setTextColor(t1.newColor(TIME_R,TIME_G,TIME_B));    
   t1.setTextSize(10);
   t1.setCursor(20, 0); // 

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
/*
void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}
*/






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
