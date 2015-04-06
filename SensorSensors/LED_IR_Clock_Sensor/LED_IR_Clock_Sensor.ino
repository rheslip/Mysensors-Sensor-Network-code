// Demonstration of LED multiplexing with Arduino
// Author: Nick Gammon
// Date: 2 December 2013
// turned into RTC backed up LED clock R Heslip June 20/2014
// turned into sensor network clock aug 2014 R Heslip - syncs to network time


#include <Time.h> 
#include <Timezone.h>    //https://github.com/JChristensen/Timezone
#include <SPI.h>
#include <MySensor.h> 
#include <MyNodes.h>
#include <IRLib.h>

// mysensor network stuff

#define NODE_ID LEDCLOCK_NODE

MySensor gw;
MyMessage msg(LEDCLOCK_NODE_IRRX, V_IR_RECEIVE);

bool timesync;
long timer;
  
//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev
time_t utc, local;

// IR stuff
int RECV_PIN = 19;

IRrecv My_Receiver(RECV_PIN);

IRdecode My_Decoder;
unsigned int Buffer[RAWBUF];

IRTYPES codeType;         // The type of code
unsigned long codeValue;  // The data bits
int codeBits;             // The length of the code in bits
bool GotOne; 

// LED driver stuff

const byte PATTERN_COUNT = 16;
const byte SEGMENTS = 7;
const byte DIGITS = 4;

const byte columnPins [SEGMENTS] = { 14, 18, 5, 7, 8, 15, 4 };  // a, b, c, d, e, f, g
const byte digitPins [DIGITS]    = { 2, 16, 17, 3 };    // DIG1, DIG2, DIG3, DIG4
#define COMMON_ANODE true    // make false for common cathode LEDs
#define COLON 6   // colon driver pin

#if COMMON_ANODE
  // For common ANODE:
  const byte SEGMENT_ON = LOW;
  const byte SEGMENT_OFF = HIGH;
  const byte DIGIT_ON = HIGH;
  const byte DIGIT_OFF = LOW;
#else
  // For common CATHODE:
  const byte SEGMENT_ON = HIGH;
  const byte SEGMENT_OFF = LOW;
  const byte DIGIT_ON = LOW;
  const byte DIGIT_OFF = HIGH;
#endif 

// extra segment patterns (you can add more)
const byte SHOW_HYPHEN = 0x0A;
const byte SHOW_E      = 0x0B;
const byte SHOW_H      = 0x0C;
const byte SHOW_L      = 0x0D;
const byte SHOW_P      = 0x0E;
const byte SHOW_BLANK  = 0x0F;

const PROGMEM byte digitSegments [PATTERN_COUNT]  =
  {
  0b1111110,  // 0  
  0b0110000,  // 1  
  0b1101101,  // 2  
  0b1111001,  // 3  
  0b0110011,  // 4  
  0b1011011,  // 5  
  0b1011111,  // 6  
  0b1110000,  // 7  
  0b1111111,  // 8  
  0b1111011,  // 9  
  0b0000001,  // 0x0A -> -  
  0b1001111,  // 0x0B -> E  
  0b0110111,  // 0x0C -> H  
  0b0001110,  // 0x0D -> L  
  0b1100111,  // 0x0E -> P  
  0b0000000,  // 0x0F -> blank 
  };


volatile byte numberToShow [DIGITS] = { SHOW_H, SHOW_E, SHOW_L, 0 };  // HELO

// timer Interrupt Service Routine (ISR) to update the LEDs
//ISR (TIMER2_COMPA_vect) 

void LED_mux(void) 
  {
  static byte digit = 0;
  byte thisDigit = numberToShow [digit] & 0xf;  // convert ascii numbers 
  
  // check for out of range, if so show a blank
  if (thisDigit >= PATTERN_COUNT)
    thisDigit = SHOW_BLANK;
    
  // turn off old digit
  for (byte i = 0; i < DIGITS; i++)
    digitalWrite (digitPins[i], DIGIT_OFF);
    
  // set segments
  for (byte j = 0; j < SEGMENTS; j++)
    digitalWrite (columnPins [j],   // which segment pin
                (pgm_read_byte (digitSegments + thisDigit) // get bit pattern 
                & bit (SEGMENTS - j - 1))     // see if set or not
                ? SEGMENT_ON : SEGMENT_OFF);  // set appropriately (HIGH or LOW)
    
  // activate this digit
  digitalWrite (digitPins [digit], DIGIT_ON);
    
  // wrap if necessary
  if (++digit >= DIGITS)
    digit = 0;
  }  // end of TIMER2_COMPA_vect


void setup() 
  {

//  Serial.begin(9600);
  
  gw.begin(NULL,NODE_ID);  // start the radio

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("IR_LEDClock", "2.0");
  // Register a sensor to gw. Use binary light for test purposes.
  gw.present(LEDCLOCK_NODE_IRRX, S_IR);
  
  // Request time from controller. 
  gw.requestTime(receiveTime);  
  
  My_Receiver.enableIRIn(); // Start the IR receiver
  My_Decoder.UseExtnBuf(Buffer);
  
  for (byte i = 0; i < SEGMENTS; i++)
    pinMode(columnPins[i], OUTPUT);  // make all the segment pins outputs
    
  for (byte i = 0; i < DIGITS; i++)
    pinMode(digitPins[i], OUTPUT);   // make all the digit pins outputs
  pinMode(COLON,OUTPUT);   // colon driver

  timer=millis();
} // end of setup


void loop ()
{

  LED_mux();  // this was interrupt driven but the IR code needs the timer interrupt
  
  if ((millis()-timer) > 300000) { // time sync every 5 minutes
    gw.requestTime(receiveTime);  // request time from network  
    timer= millis();
  }
  
  gw.process();  // process network messages
  utc = now();
  local = myTZ.toLocal(utc, &tcr);
//  printTime(utc, "UTC");
//  printTime(local, tcr -> abbrev);

  int hournow=hourFormat12(local); // use 12 hour format
  int minutenow=minute(local);
  if (hournow >9) numberToShow[0]='1';
  else numberToShow[0]=0xf;  // zero blank
  numberToShow[1]=hournow%10;
  numberToShow[2]=minutenow/10;
  numberToShow[3]=minutenow%10;
  
  if ((millis()%2000) >1000) digitalWrite(COLON,1);  // on 1 in 2 seconds
  else digitalWrite(COLON,0);
 
  // do IR
  if (My_Receiver.GetResults(&My_Decoder)) {
    //Restart the receiver so it can be capturing another code
    //while we are working on decoding this one.
    My_Receiver.resume(); 
    My_Decoder.decode();
//    My_Decoder.DumpResults();
    if(My_Decoder.decode_type == UNKNOWN) {
      Serial.println(F("Unknown"));
    } else {
      codeType= My_Decoder.decode_type;
      codeValue= My_Decoder.value;
      Serial.println(My_Decoder.value,HEX);
      codeBits= My_Decoder.bits;
      char buffer[10];
      sprintf(buffer, "%08lx", My_Decoder.value);
    // Send ir result to gw
      gw.send(msg.set(buffer));
    }
  }

} // end of loop


// This is called when a new network time value is received

void receiveTime(unsigned long time) {
  // Ok, set incoming time 
  setTime(time);
  timesync=true;
}



