//
// basic sensor controller for MySensor.org network
// uses Mega1284 - serial 1 is connected to a Mega328 running the Serial Gateway code
// the Gateway handles the real time network messages
// the controller code decodes sensor messages, saves the data and sends commands to sensors
// I'm relying on the serial input buffer in the 1284 to buffer enough messages from the gateway that it won't lose messages
// this is much worse when debugging is turned on - lots of status info from the gateway
// you can tweek the serial buffer sizes in the core code for the 1284 if overruns become a problem
// in general tho you should design the nodes to keep the network traffic fairly low - a few messages per minute
// note the controller code can get delayed for quite some time doing things e.g. twitter posts
// so I added a watchdog to reset it if the delay exceeds 8 seconds - it resets once every 100 hours or so because of this
// June 2014
// R Heslip
// oct 20/14 - added code for sump water depth
// nov 30 - added attic temp/humidity/battery level sensor
// feb 2015 - changed time to local time using the Timezone library - easier to interpret twitter messages etc
// April 2015 GITHUB version - this is a "deidentified" version of the code. You need your own Twitter credentials etc

//#include <Gateway.h>  
#include <SPI.h>
#include <MySensor.h>
#include <MyNodes.h>
#include <stdarg.h>
#include <avr/progmem.h>
#include <UIPEthernet.h>
#include <UIPUdp.h>
#include <WebServer.h>
#include <Clock.h>
#include <Twitter.h>
#include <X10RF.h>
//#include <streaming.h>
#include <Time.h> 
#include <Timezone.h>    //https://github.com/JChristensen/Timezone


//#define DEBUGMSG         // define to get debugging info
//#define TELNET      // define to include Telnet 

#define MAX_RECEIVE_LENGTH 100 // Max buffersize needed for messages 
#define ERROR_LED  0  // red led on digital 0

// serial input buffer etc
char inputString[MAX_RECEIVE_LENGTH] = "";    // A string to hold incoming commands from serial/ethernet interface
int inputPos = 0;
boolean commandComplete = false;  // whether the string is complete
long msgtimestamp;  // message time Unix
String timestamp;    // message time as string
long startuptime=0;  // to calculate uptime


// tokens parsed from gateway messages
char *value=NULL;
uint8_t nodeId = 0;
uint8_t childId = 0;
uint8_t messageType = 0;
uint8_t type = 0;

// sensor values from sensor nodes
uint16_t  baro_pressure = 0;
int16_t  upstairs_temp = 0; // temp from GPS clock node
int16_t  outside_temp = 0; // outside temp from GPS clock node
int16_t  garage_temp = 0; // outside temp from garage door sensor node
int16_t  freezer1_temp = 0; // large chest freezer
int16_t  freezer2_temp = 0; // basement fridge freezer
int16_t  garage_door_dist = 0; // distance sensor value from garage door sensor node
uint16_t  outside_humidity = 0; // outside RH from GPS clock node
int16_t  sump_node_temp = 0; // basement temp from sump monitor node
uint16_t  sump_node_humidity = 0; // RH from sump monitor node
uint16_t  sump_node_battery = 0; // battery voltage from sump monitor node
uint16_t  sump_node_alarm = 0; // alarm switch from sump monitor node
int16_t  sump_node_water_depth = 0; // water depth from sump monitor node
uint32_t  IR_code = 0; // IR codes from LED clock sensor
// uint16_t  attic_humidity = 0; // attic RH - not used
int16_t  attic_temp = 0; // attic temp 
uint16_t attic_node_battery = 0; // battery on attic sensor

// time stamps for sensor values - so we can tell if it stops reporting
unsigned long  baro_pressure_stamp=0;
unsigned long  upstairs_temp_stamp = 0; // temp from GPS clock node
unsigned long  outside_temp_stamp = 0; // outside temp from GPS clock node
unsigned long  freezer_stamp = 0; // freezer alarm node
unsigned long  outside_humidity_stamp = 0; // outside RH from GPS clock node
unsigned long  sump_node_temp_stamp = 0; // basement temp from sump monitor node
unsigned long  sump_node_humidity_stamp = 0; // RH from sump monitor node
unsigned long  sump_node_battery_stamp = 0; // battery voltage from sump monitor node
unsigned long  sump_node_alarm_stamp = 0; // alarm switch from sump monitor node
unsigned long  garage_door_stamp = 0; // garage door sensor node
unsigned long  attic_stamp = 0; // attic sensor node

// time stamps for nodes without sensors
unsigned long LED_clock_stamp = 0;
unsigned long LCD_clock_stamp = 0;

// keeps track of which line to refresh on remote display
uint8_t messagetodisplay=LINE1;

// stuff specific to garage door
typedef enum {GDOOR_OPEN,GDOOR_CLOSED,GDOOR_CAR} gdoor_state ;
#define GDOOR_CLOSED_DIST 0
#define GDOOR_OPEN_DIST 55  // distance sensed by ultrasonic ping 
#define GDOOR_CAR_DIST 116
#define GDOOR_DELTA  10    // allowable slop on the reading
unsigned g_door = GDOOR_OPEN;  // door state
unsigned g_door_prev = GDOOR_OPEN;  // previous door state

// stuff specific to freezer alarm
#define FREEZER1_OK 500  // lower reading than this indicates failure
#define FREEZER2_OK 350  // lower reading than this indicates failure

// IR codes from LED clock sensor
#define MEDIAPC_ON 0x801c2ba4  // IR code sent by Harmony
#define MEDIAPC_ON1 0x801caba4  // Harmony seems to toggle between these two codes

// sump monitor stuff
#define DISTANCE_TO_BOTTOM 52  // distance from sensor to bottom of sump hole in cm - sensor reports distance from cover
#define SUMP_WATER_MAX 38    // water above this is an alarm
#define SUMP_WATER_MIN 22    // water below this means a stuck pump switch
unsigned long  sump_pump_cycled = 0; // time of last pump cycle

// X10 stuff

const int RFPin =  3;      //  
X10RF moduleX10RF (RFPin);
unsigned long X10timer=0;    // X10 interval timer
bool X10state=0;

// ethernet stuff
#define WOL_PORT 7  // port to use for WOL

byte MAC[] = {0xE, 0xD, 0xF, 0xA, 0xE, 0xED}; // mac of the sensor controller
byte bc[] = { 255, 255, 255, 255 };    // Broadcast IP address
byte pc_mac[] = {0x0,0xD,0x2,0x2,0x5,0x6};  // Dell media laptop WOL target MAC 

//  IPAddress IP(192, 168, 15, 55);
EthernetServer server = EthernetServer(23);  // telnet server
EthernetUDP Udp;  // UDP server for WOL

#ifdef TELNET
EthernetClient client;
boolean clientconnected=false; // flag - true when we have a telnet connection
#endif


// time stuff
#define LCD_UPDATE  20000    // how often to update the LCD display in my bedroom - ms
#define TWITTER_UPDATE 14400000  // how often to send an update to Twitter - ms

Clock g_Clock;
time_t local; 

//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev

unsigned long LCDdisplaytimer = 0;    // interval timer
unsigned long twittertimer=0;    // twitter interval timer

// Twitter stuff
// Your Token to Tweet (get it from http://arduino-tweet.appspot.com/)
Twitter twitter("your token here");

// Webserver stuff
/* This creates an instance of the webserver.  By specifying a prefix
 * of "", all pages will be at the root of the server. */
#define PREFIX ""
WebServer webserver(PREFIX, 80);


// no-cost stream operator as described at 
// http://sundial.org/arduino/?page_id=119
template<class T>
inline Print &operator <<(Print &obj, T arg)
{ obj.print(arg); return obj; }


// web server code

/* commands are functions that get called by the webserver framework
 * they can read any posted data from client, and they output to the
 * server to send data back to the web browser. */
void helloCmd(WebServer &server, WebServer::ConnectionType type, char *, bool addControls)
{
  P(htmlHead) =
    "<html>"
    "<head>"
    "<title>Sensor Network Web Server</title>"
    "<style type=\"text/css\">"
    "BODY { font-family: sans-serif }"
    "H1 { font-size: 14pt; text-decoration: underline }"
    "P  { font-size: 10pt; }"
    "</style>"
    "</head>"
    "<body>";

  int i;

  server.httpSuccess();
  server.printP(htmlHead);
  addControls=false;

  if (addControls)
    server << "<form action='" PREFIX "/form' method='post'>";
    
  server << "<H1>Rich's Sensor Network Status at ";
//  server << g_Clock.GetTimestamp() << " Unix Time ";  

//  server << hour(local) << ":" << minute(local)<< ":" << second(local);
  server << LocalTimeString();
  server << "</H1><br>"; 
  server << "Uptime " << (g_Clock.GetTimestamp()-startuptime)/3600<< " hours and "<< (g_Clock.GetTimestamp()-startuptime)%3600/60<< " minutes <br>";    
  server << "Last message " << (g_Clock.GetTimestamp()-msgtimestamp)<< " seconds ago from node:"<< nodeId << " child:" << childId<<"<br><br>";    
 
  server << "LCD Clock Weather Sensor Node 120 ("<<(g_Clock.GetTimestamp()-baro_pressure_stamp)<< "s)<br>"; 
  server << "Barometric Pressure  "<< baro_pressure<<"hPa<br>";

  server << "Outdoor Temperature  "<< outside_temp<<"C<br>"; 
//    server << " ("<<(g_Clock.GetTimestamp()-outside_temp_stamp)<< "s)<br>";
  server << "Outdoor Humidity     "<< outside_humidity<<"%<br>";
//    server<< " ("<<(g_Clock.GetTimestamp()-outside_humidity_stamp)<< "s)<br>";
  server << "Upstairs Temperature "<< upstairs_temp<<"C<br>";
//    server<< " ("<<(g_Clock.GetTimestamp()-upstairs_temp_stamp)<< "s)<br>";

  server << "<br>";   
  server << "Sump Alarm Sensor Node 10 ("<<(g_Clock.GetTimestamp()-sump_node_temp_stamp)<< "s)<br>";   
  server << "Sump                 "<< (sump_node_alarm ? "ALARM" : "OK");
  server<< " ("<<(g_Clock.GetTimestamp()-sump_node_alarm_stamp)<< "s)<br>"; 
  server << "Sump Water Depth  "<< sump_node_water_depth << "cm<br>";
  server << "Last Pump Cycle  "<< (g_Clock.GetTimestamp()-sump_pump_cycled)/60<< " minutes ago<br>";
  server << "Sump Backup Battery  "<< sump_node_battery << "%<br>";
//   server << " ("<<(g_Clock.GetTimestamp()-sump_node_temp_stamp)<< "s)<br>";  
  server << "Basement Temperature "<< sump_node_temp<<"C<br>";
  server << "Basement Humidity    "<< sump_node_humidity<<"%<br><br>";
//   server << " ("<<(g_Clock.GetTimestamp()-sump_node_humidity_stamp)<< "s)<br>";
//  server << "<br>";   

  server << "Freezer Alarm Sensor Node 20 ("<<(g_Clock.GetTimestamp()-freezer_stamp)<< "s)<br>";   
  server << "Chest Freezer "<< ((freezer1_temp< FREEZER1_OK) ? "ALARM" : "OK")<< " ("<< freezer1_temp<<")<br>";
  server << "Fridge Freezer "<< ((freezer2_temp< FREEZER2_OK) ? "ALARM" : "OK")<< " ("<< freezer2_temp<<")<br><br>";
  
//   server << " ("<<(g_Clock.GetTimestamp()-sump_node_battery_stamp)<< "s)<br>";   
  server << "Garage Door Sensor Node 60 ("<<(g_Clock.GetTimestamp()-garage_door_stamp)<< "s)<br>"; 
  server << "Garage Door " << (g_door ? "Closed" : "Open") <<"<br>";
  server << "Garage Temperature  "<< garage_temp<<"C<br>"; 
  server << "Garage Door sensor  "<< garage_door_dist<<"cm<br><br>";
  
  server << "Attic Sensor Node 150 ("<<(g_Clock.GetTimestamp()-attic_stamp)<< "s)<br>";   
  server << "Battery  "<< attic_node_battery << "%<br>";
  server << "Attic Temperature "<< attic_temp<<"C<br><br>";
//  server << "Attic Humidity    "<< attic_humidity<<"%<br><br>";
  
  server << "LED Clock Node 100 ("<<(g_Clock.GetTimestamp()-LED_clock_stamp)<< "s)<br>"; 
  server << "LCD Clock Node 110 ("<<(g_Clock.GetTimestamp()-LCD_clock_stamp)<< "s)<br>"; 
  
  if (addControls)
    server << "<input type='submit' value='Submit'/></form>";

  server << "</body></html>";
}

// WOL - create a magic packet and send to target MAC address

void send_magic_packet(uint8_t macaddr[]) {
    int i,j;
    Udp.beginPacket(bc, WOL_PORT);
    for (i = 0; i < 6; ++i) Udp.write(0xFF); // preamble
    for (i = 0; i < 16; i++) {  // send the target MAC 16 times
       for (j = 0; j < 6; j++) {
          Udp.write(macaddr[j]);
        }
    }
    Udp.endPacket();
}


// Arduino initialization function

void setup() {

  pinMode(ERROR_LED,OUTPUT);
  digitalWrite(ERROR_LED,0);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);   // console port - have to run fast because output messages are larger than the input messages
  Serial1.begin(9600);  // sensor gateway port
  Serial.println("Sensor Gateway Controller Nov 2014 R Heslip ");
    // set the X10 RF pin as output:
  pinMode(RFPin, OUTPUT); 
  
  // Initialize Ethernet.
//  Ethernet.begin(ArduinoMAC, ArduinoIP);
  if (Ethernet.begin(MAC) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    digitalWrite(ERROR_LED,1);
    for(;;)
      ;
  }
  // print your local IP address:
  Serial.print("Controller DCHP Address: ");
  Serial.println(Ethernet.localIP());
    // Setup the clock. we are 4 or 5 hours behind GMT 
  g_Clock.Setup();
  // eastern daylight
//  g_Clock.SetTimezoneOffset(-4,0);   // use Zulu time - no DST changes needed

    Udp.begin(7);  // start the UDP server

#ifdef TELNET    
  server.begin();  // start telnet server
#endif

    /* setup our default command that will be run when the user accesses
   * the root page on the server */
  webserver.setDefaultCommand(&helloCmd);

  /* run the same command if you try to load /index.html, a common
   * default page name */
  webserver.addCommand("index.html", &helloCmd);

  /* start the webserver */
  webserver.begin();
  wdt_enable(WDTO_8S);  // added watchdog aug 21/14 because code is locking up someplace 
}

void loop()   // Arduino superloop - where everything gets done
{
  char buff[64];
  int len = 64;
  wdt_reset();   // reset watchdog
  checkSerialInput();  // look for messages from the radio gateway
  g_Clock.Maintain();  // sync the system time using NTP - this is done at a period defined in the clock library
  if ((g_Clock.GetTimestamp() > 1000000) && (startuptime==0)) startuptime= g_Clock.GetTimestamp(); // a little kludgy - as soon as time becomes valid, save it

  /* process incoming connections one at a time forever */
  webserver.processConnection(buff, &len);


#ifdef TELNET
  // handle telnet clients
  // seems to be a bug in the telnet server stuff - it never detects a disconnect. I've tried everything!
  // until I figure it out you only get one connection, after which its hung
  // telnet is quite flakey...
  if ((client = server.available()) && !clientconnected) {
    clientconnected=true;  // check for a new telnet session
    client.flush();
    Serial.println("Telnet client connected");   
  } 
#endif

/*  
// when this code is inserted I can end a telnet session but the telnet output below in process_message no longer works
// maybe some problem with streams ?
  char c;
  if (clientconnected && client.available()) {
    c=client.read(); // read input
    client.write(c);
    if (c=='q' || c=='Q') {
       client.flush();
       client.stop();
       clientconnected=false;
       Serial.println("Telnet client disconnected");  
    }
  }    
*/    
// update remote display periodically  
  if ((millis()-LCDdisplaytimer) > LCD_UPDATE) {  
    posttodisplay();
    LCDdisplaytimer=millis();
  } 
// update Twitter periodically  
  if ((millis()-twittertimer) > TWITTER_UPDATE) {  
    postTwittersummary();
    twittertimer=millis();
  } 
// X10 test  
  if ((millis()-X10timer) > 8000) {  
     moduleX10RF.x10_sendcmd ( 'A' , 1, X10state);
    X10timer=millis();
    X10state=!X10state;
  } 
 
}


void checkSerialInput() {
  serialEvent();            // this was not in the gateway code, I didn't see where this routine got called
  if (commandComplete) {
    // A command wass issued from serial interface
    // We will now handle it
    process_message(inputString);
    commandComplete = false;  
    inputPos = 0;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read(); 
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inputPos<MAX_RECEIVE_LENGTH-1 && !commandComplete) { 
      if (inChar == '\n') {
        inputString[inputPos] = 0;
        commandComplete = true;
      } else {
        // add it to the inputString:
        inputString[inputPos] = inChar;
        inputPos++;
      }
    } else {
       // Incoming message too long. Throw away 
        inputPos = 0;
    }
  }
}



void process_message(char *commandBuffer) {
  boolean ok = false;
  char *str, *p;
  int val0,val1,val2,val3;
  value=NULL;
  int i = 0;
  val0 = 0;
  val1 = 0;
  val2 = 0;
  val3 = 0;

  msgtimestamp=g_Clock.GetTimestamp();
  timestamp = String(msgtimestamp,DEC);
  
  g_Clock.WriteTime(&Serial);  // time stamp
  Serial.print(" ");
  Serial.println(commandBuffer);   // echo gateway message to console 

#ifdef TELNET
// telnet - doesn't work very well
  if(client) {
    client.print(timestamp);  
    client.print(" ");
    client.println(commandBuffer);   // echo gateway message to telnet 
  }
#endif
  
  // Extract command data coming on serial line
  for (str = strtok_r(commandBuffer, ";", &p);       // split using semicolon
    str && i < 5;         // loop while str is not null an max 5 times
    str = strtok_r(NULL, ";", &p)) {              // get subsequent tokens
      switch (i) {
	case 0: // Radioid (destination)
	  val0 = atoi(str);
	  break;
	case 1: // Childid
	  val1 = atoi(str);
	  break;
	case 2: // Message type
	  val2 = atoi(str);
	  break;
	case 3: // Data type
	  val3 = atoi(str);
	  break;
	case 4: // Variable value
	  value = str;
	  break;
    }
    i++;
  }
// check that this is a valid message before processing it
  if (val0==0) return; // messages from node 0 are debug messages
  if ((i<4) || (i>5)) {  // time requests have no variable so i=4
    Serial.println("Invalid message format");
    return;
  }
  
  nodeId=val0;
  childId=val1;
  messageType=val2;
  type=val3;
  
#ifdef DEBUGMSG 
  Serial.print(timestamp);  
  Serial.print(" NodeId:");
  Serial.print(nodeId); 
  Serial.print(" ChildId:");
  Serial.print(childId); 
  Serial.print(" MessageType:");
  Serial.print(messageType); 
  Serial.print(" DataType:");
  Serial.print(type); 
  Serial.print(" Value:");
  Serial.println(value); 
 // send to telnet as well 
  client.print(timestamp);  
  client.print(" NodeId:");
  client.print(nodeId); 
  client.print(" ChildId:"); 
  client.print(childId); 
  client.print(" MessageType:");
  client.print(messageType); 
  client.print(" DataType:");
  client.print(type); 
  client.print(" Value:");
  client.println(value);  
#endif

// timestamps for nodes that don't have sensors
  switch (nodeId) {
    case LEDCLOCK_NODE:
      LED_clock_stamp=msgtimestamp;
      break;
    case LCDCLOCK_NODE:
      LCD_clock_stamp=msgtimestamp;
      break;
  }    

// process messages meant for controller
  switch (messageType) {
    case C_INTERNAL:  // internal messages to the controller
      switch (type) {
        case I_TIME:  // handle time requests from any node
           sendTime(nodeId,childId);
          break;
        case I_BATTERY_LEVEL:  // record node battery levels
           switch(nodeId) {
             case SUMP_NODE:
               sump_node_battery=atoi(value);
               sump_node_battery_stamp=msgtimestamp;
               break; 
             case ATTIC_NODE:
               attic_node_battery=atoi(value);
               attic_stamp=msgtimestamp;
               break;                
           }
          break;
      }
      break;
// process SET messages
    case C_SET:  // set values - store these locally
      switch (type) {
        case V_PRESSURE:   // baro pressure - don't care which node it came from
           baro_pressure=atoi(value);
           baro_pressure_stamp=msgtimestamp;
        break;
        case V_TEMP:   // Temperature 
          switch(nodeId) {
            case SUMP_NODE:
              sump_node_temp=atoi(value);
              sump_node_temp_stamp=msgtimestamp;
              break;
            case LCDGPSCLOCK_NODE:
             if (childId==LCDGPSCLOCK_TEMP_CHILD) {
                 upstairs_temp=atoi(value);  // this node has inside and outside temp
                 upstairs_temp_stamp=msgtimestamp;
             }
             else {
               outside_temp=atoi(value); //#### this is a float value, need to convert it
               outside_temp_stamp=msgtimestamp;
             }
             break;
            case GARAGE_DOOR_NODE:
              garage_temp=atoi(value);
              // garage_door_stamp=msgtimestamp;
              break;
            case ATTIC_NODE:
              attic_temp=atoi(value);
              attic_stamp=msgtimestamp;
              break;
            case FREEZER_NODE:
              switch (childId) {
                case FREEZER_NODE_FREEZER1:
                  freezer1_temp=atoi(value);
                  break;
                case FREEZER_NODE_FREEZER2:
                  freezer2_temp=atoi(value);
                  break; 
              }               
              freezer_stamp=msgtimestamp;
              break;
            break;
          }
        break;
        case V_HUM:   // humidity 
           switch(nodeId) {
             case SUMP_NODE:
               sump_node_humidity=atoi(value);
               sump_node_humidity_stamp=msgtimestamp;
               break;               
             case LCDGPSCLOCK_NODE:
               outside_humidity=atoi(value);
               outside_humidity_stamp=msgtimestamp;
               break;
 /*            case ATTIC_NODE:
               attic_humidity=atoi(value);
               attic_stamp=msgtimestamp;
               break;
*/
           }
        break;
        case V_DISTANCE:   // distance sensors 
           switch(nodeId) {
             case GARAGE_DOOR_NODE:
               garage_door_dist=atoi(value);  // senses garage door open/closed and door closed+car in garage
               garage_door_stamp=msgtimestamp;
               if (abs(garage_door_dist - GDOOR_OPEN_DIST) < GDOOR_DELTA) g_door = GDOOR_OPEN;
               else if (abs(garage_door_dist - GDOOR_CAR_DIST) < GDOOR_DELTA) g_door = GDOOR_CAR;
               else g_door = GDOOR_CLOSED;  // default, dist value returned is normally 0 but might sometimes get a reading
  
               if (g_door != g_door_prev) { // door state has changed
             /* bug in the sensor code needs to be fixed - it oscillates between states
             if (g_door == GDOOR_OPEN) posttoTwitter(LocalTimeString()+"  ");
                 else if (g_door == GDOOR_CLOSED) posttoTwitter(LocalTimeString()+" ");  // either of the other two states is door closed
                 else if (g_door == GDOOR_CAR) posttoTwitter(LocalTimeString()+" ");
                 */
                 g_door_prev=g_door;
               }
             break;  
            case SUMP_NODE:
              int16_t last=sump_node_water_depth; // to detect change in depth
              sump_node_water_depth= DISTANCE_TO_BOTTOM - atoi(value);
              if ((sump_node_water_depth-last) < -5) { // water is getting pumped out - 
                sump_pump_cycled=msgtimestamp;  // save time of last pump cycle
              }
              break;          
           }
        break;
        case V_IR_RECEIVE:   // alarm messages 
           switch(nodeId) {
             case LEDCLOCK_NODE:
               IR_code=strtoul(value, 0, 16);
               if ((IR_code == MEDIAPC_ON) || (IR_code == MEDIAPC_ON1)) {
                 Serial.println("sending magic packet");
                 send_magic_packet(pc_mac);
               }
               break;               
           }
        break;
        case V_TRIPPED:   // alarm messages 
           switch(nodeId) {
             case SUMP_NODE:
               sump_node_alarm=atoi(value);
               sump_node_alarm_stamp=msgtimestamp;
               if (sump_node_alarm) posttoTwitter(LocalTimeString()+" ");  // #### this may not be the best place to do this
               break;               
           }
        break;
      }
   break;

  }
         
}

void sendMessage1(uint8_t destination, uint8_t sensor, String payload ) {
	uint8_t command = C_SET;
	uint8_t acknowledge = 0; // no ack
	uint8_t type = V_VAR1;
	String td = encode(destination, sensor, command, acknowledge, type, payload);
//	Serial.println("--> " + td);
	Serial1.println(td);
}

void sendTime(uint8_t destination, uint8_t sensor) {
	String payload = timestamp;
	uint8_t command = C_INTERNAL;
	uint8_t acknowledge = 0; // no ack
	uint8_t type = I_TIME;
	String td = encode(destination, sensor, command, acknowledge, type, payload);
//	Serial.println("--> " + td);
	Serial1.println(td);
}

// adapted from the MySensor.org controller .js code
// encode a message as a string to send to the gateway

String encode(uint8_t destination, uint8_t sensor, uint8_t command, uint8_t acknowledge, uint8_t type, String payload) {
  String msg;
  msg = String(destination) + ';' + String(sensor) + ';' + String(command) + ';' + String(acknowledge) + ';' + String(type) + ';';
/*  if (command == 4) {  // special case of binary file mode
    for (var i = 0; i < payload.length; i++) {
	if (payload[i] < 16)
	msg += "0";
	msg += payload[i].toString(16);
     }
  } else {
    */
  msg += payload;

//  msg += '\n';
  return msg;
}

// send messages to remote display
// we do this round robin so the remote LCD can keep up

void posttodisplay(void) {
  String msg;
  switch (messagetodisplay) {
    case LINE1:  
      msg = " "; 
      sendMessage1(LCDCLOCK_NODE, COLORRED | LINE1, msg);
      break;
    case LINE2: 
      msg = "Outside Temp " + String(outside_temp) + "C"; 
      sendMessage1(LCDCLOCK_NODE, COLORYELLOW | LINE2, msg);
      break;
    case LINE3:
      msg = "Outside RH " + String(outside_humidity) + "%"; 
      sendMessage1(LCDCLOCK_NODE, COLORYELLOW | LINE3, msg);
      break;
    case LINE4:
      msg = "Upstairs Temp " + String(upstairs_temp) + "C"; 
      sendMessage1(LCDCLOCK_NODE, COLORYELLOW | LINE4, msg);
      break;
    case LINE5:
      msg = "Baro Pressure " + String(baro_pressure) + "hPa"; 
      sendMessage1(LCDCLOCK_NODE, COLORYELLOW | LINE5, msg);
      break;
    case LINE6:
      msg = " "; 
      sendMessage1(LCDCLOCK_NODE, COLORYELLOW | LINE6, msg);
      break;

  }
  ++messagetodisplay;
  if (messagetodisplay > LINE6) messagetodisplay=LINE1;
}

// return the time and date as a string

String TimeString(void) {
  String timestring;
  DateTime tm; 
  g_Clock.DecodeTo(tm);
 timestring=String(tm.Hour,DEC)+":";
 timestring+=String(tm.Minute,DEC)+":";
 timestring+=String(tm.Second,DEC)+" "; 
 timestring+=String(tm.Month,DEC)+"."; 
 timestring+=String(tm.Day,DEC)+"."; 
 timestring+=String(tm.Year,DEC)+" "; 
 return timestring;
}

// return the local (DST) time and date as a string

String LocalTimeString(void) {
  char str[30];
  local = myTZ.toLocal(g_Clock.GetTimestamp(), &tcr);
  sprintf(str,"%02d:%02d:%02d %d.%d.%d",hour(local),minute(local),second(local),month(local),day(local),year(local));
  return String(str);
}
  
// post a status summary to twitter
void postTwittersummary(void) {
 String msg;
 msg=LocalTimeString();
// msg+="Uptime " + String((g_Clock.GetTimestamp()-startuptime)/3600) + " hours " + String((g_Clock.GetTimestamp()-startuptime)%3600/60) + " min"; 
// keep the message short
 msg+="Up:" + String((g_Clock.GetTimestamp()-startuptime)/3600) + "h "; 
 msg+="O:" + String(outside_temp) +"C ";
 msg+="I:" + String(upstairs_temp)+ "C "; 
 msg+="Sump:" + String(sump_node_alarm ? "ALARM " : "OK ");
 msg+="F1:" + String ((freezer1_temp< FREEZER1_OK) ? "ALARM " : "OK ");
 msg+="F2:" + String ((freezer2_temp< FREEZER2_OK) ? "ALARM " : "OK ");
 msg+="10:"+ String((g_Clock.GetTimestamp()-sump_node_alarm_stamp))+ "s "; 
 msg+="20:"+ String((g_Clock.GetTimestamp()-freezer_stamp))+ "s "; 
 msg+="60:"+ String((g_Clock.GetTimestamp()-garage_door_stamp))+ "s "; 
 msg+="100:"+ String((g_Clock.GetTimestamp()-LED_clock_stamp))+ "s ";  
 msg+="110:"+ String((g_Clock.GetTimestamp()-LCD_clock_stamp))+ "s "; 
 msg+="120:"+ String((g_Clock.GetTimestamp()-outside_temp_stamp))+ "s "; 
 msg+="";   // so listener account gets notifications
 
 posttoTwitter(msg);
}

// post a message to twitter
void posttoTwitter(String post)
{
  char msg[160];
  for (int i=0; i<160; ++i) msg[i]=post[i]; // this is pretty kludgy 
  Serial.print("Posting to Twitter: ");
  Serial.println(msg);
  Serial.println("connecting to Twitter...");
//  client.print("Posting to Twitter: ");
//  client.println(msg);
//  client.println("connecting to Twitter...");
  if (twitter.post(msg)) {
    int status = twitter.wait();
    if (status == 200) {
      Serial.println("OK.");
//      client.println("OK.");
    } else {
      Serial.print("failed : code ");
      Serial.println(status);
//      client.print("failed : code ");
//      client.println(status);
    }
  } else {
    Serial.println("connection failed.");
//    client.println("connection failed.");
  }
//  delay(1000);
}



