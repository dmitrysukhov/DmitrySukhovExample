#include <RTClib.h>

#include <Wire.h>
#include "RTClib.h"
#include <avr/interrupt.h>
#include <Time.h>
#include <TimeLib.h>
#include <TimerOne.h>
#include <ServoTimer2.h>
ServoTimer2  myservo;  // create servo object to control a servo
int potpin = 1;  // analog pin used to connect the potentiometer

RTC_DS1307 rtc;
volatile int time = 0;

int interrupt_pin = 2;
volatile int toggle = 0;
volatile int t = 0;
#define PIN_B 3
#define PIN_R 4
#define PIN_W 5
#define PIN_G 6

#define DIVISIONS   256     // Number of segments the clock face is divided into
#define OFFSET      (int)((DIVISIONS)*0.681f) // Required segment offset approx. 257°, empirical value/depends on hall placement

// Color codes "RGB" //todo
const uint8_t black = 0;
const uint8_t red = 4;
const uint8_t green = 2;
const uint8_t blue = 1;
const uint8_t yellow = 6;
const uint8_t purple = 5;
const uint8_t cyan = 3;
const uint8_t white = 7;

uint8_t width = 1; // segment's width
bool fl = true; // width-flag
bool number = true;

int n1;
int n2;
int n3;
int n4;
int n5;
int n6;
int n7;

// Desired color of clock face; replace predefined color-codes by any of the ones given above in the "Defines" area
uint8_t bgr_clr = black ;    // Background color
uint8_t num1_c = red;       // Hour hand
uint8_t num2_c = yellow;    // Minute hand
uint8_t num3_c = green; // Second hand
uint8_t num4_c = cyan;
uint8_t num5_c = purple;
uint8_t num6_c = white;
uint8_t num7_c = blue;

uint8_t s;
uint8_t s2;
uint8_t s3;
uint8_t s4;
uint8_t s5;
uint8_t s6;
uint8_t s7;

#define runEvery(t) for (static uint16_t _lasttime; (uint16_t)((uint16_t)millis() - _lasttime) >= (t); _lasttime += (t))

struct LED
{
  bool red : 1;
  bool green : 1;
  bool blue : 1;
};

LED segment[2][(int)DIVISIONS];

boolean modeToggleFlag = false;   // Stores whether on next check the 7 segment mode should be toggled between TIME and TEMP or not

// Global, interrupt accessible variables
volatile uint16_t revTime = 0;
volatile uint16_t lastRev = micros();
volatile uint16_t segTime = 0;
volatile uint16_t currentSegment = OFFSET;


#define DIVISIONS 256 // Number of segments the clock face is divided into
#define OFFSET (int)((DIVISIONS)*0.681f) // Required segment offset approx. 257°, empirical value/depends on hall placement

// twelve servo objects can be created on most boards

int pos = 1350;    // variable to store the servo position

void pin2Interrupt(void)
{
  /* This will bring us back from sleep. */
  //  toggle = 1;
  revTime = micros() - lastRev;

  // Ignore misleading interrupt triggering
  if ((revTime >= 5000)) {
    Timer1.stop();

    lastRev = micros();
    segTime = (int)((revTime / DIVISIONS) + 0.5f);

    currentSegment = OFFSET;

    Timer1.setPeriod(segTime);
    Timer1.start();
  }
}

// Switch LED strip color the the one required of the currently active segment
void draw()
{
  digitalWrite(PIN_R, segment[0][currentSegment].red);
  digitalWrite(PIN_G, segment[0][currentSegment].green);
  digitalWrite(PIN_B, segment[0][currentSegment].blue);

  currentSegment++;
  if (currentSegment >= DIVISIONS) currentSegment = 0;
}

void setup() {
  // Zero-init segment color storage array
  
  for (uint16_t i = 0; i < DIVISIONS; i++) {
    segment[0][i].red = 1; //to see initialization
    segment[0][i].green = 0;
    segment[0][i].blue = 0;
    segment[1][i].red = 0;
    segment[1][i].green = 0;
    segment[1][i].blue = 0;
  }

  // put your setup code here, to run once:
  Serial.begin(9600);
  /* attaches the servo on pin 9 to the servo object */
  myservo.attach(9);
  Wire.begin();
  rtc.begin();
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  /* Setup the interrupt pin */
  pinMode(interrupt_pin, INPUT);
  attachInterrupt(0, pin2Interrupt, FALLING);


  do
  {
    randomSeed(analogRead(0)); //random sequence depends on the noise
    s = random(0, DIVISIONS - 1);     //a random position of the hands
    s2 = random(0, DIVISIONS - 1);   //...
    s3 = random(0, DIVISIONS - 1);  //...
    s4 = random(0, DIVISIONS - 1); //...
    s5 = random(0, DIVISIONS - 1); //...
    s6 = random(0, DIVISIONS - 1); //...
    s7 = random(0, DIVISIONS - 1); //...
  } while ( (abs(s - s2) <= 5) || (abs(s - s3) <= 5) || (abs(s - s4) <= 5) || (abs(s - s5) <= 5) || (abs(s - s6) <= 5) || (abs(s - s7) <= 5) || (abs(s2 - s3) <= 5) || (abs(s2 - s4) <= 5) || (abs(s2 - s5) <= 5) || (abs(s2 - s6) <= 5) || (abs(s2 - s7) <= 5) || (abs(s3 - s4) <= 5) || (abs(s3 - s5) <= 5) || (abs(s3 - s6) <= 5) || (abs(s3 - s7) <= 5) || (abs(s4 - s5) <= 5) || (abs(s4 - s6) <= 5) || (abs(s4 - s7) <= 5) || (abs(s5 - s6) <= 5) || (abs(s5 - s7) <= 5) || (abs(s6 - s7) <= 5));

  //clockwise or counterclockwise
  /*   int arre[7];
    for (int i = 0; i < 7; i++)
    {
     int rr = random(0, 1000);
     if (rr >= 50) arre[i] = 1;
     else arre[i] = -1;
    }
    n1 = arre[0];
    n2 = arre[1];
    n3 = arre[2];
    n4 = arre[3];
    n5 = arre[4];
    n6 = arre[5];
    n7 = arre[6];*/

  n1 = 1;
  n2 = -1;
  n3 = 1;
  n4 = -1;
  n5 = 1;
  n6 = -1;
  n7 = 1;
  
  pinMode(A1, INPUT); // res
  pinMode(A0, INPUT); // keys

  pinMode(PIN_R, OUTPUT); // led1
  pinMode(PIN_G, OUTPUT); // led2
  pinMode(PIN_B, OUTPUT); // led3
  pinMode(PIN_W, OUTPUT); // led4

  Timer1.initialize();
  Timer1.attachInterrupt(draw);

  Serial.println("Initialisation complete.");
}

void loop()
{
  runEvery(100)
  {
   // pos = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
 //    pos = map(pos, 0, 1023, 544, 2400);     // scale it to use it with the servo (value between 0 and 180)
  //  pos = 1350;
   
    myservo.write(pos);
    // Serial.println(pos);
  }

  // Get current time from DS3231 RTC every second
  runEvery(1)
  {
    makeVisible();
    if (modeToggleFlag) {
      modeToggleFlag = false;
    }
  }
  runEvery(20) {
    width = 1;
    DateTime time = rtc.now();
    fillSegments(time);

  }

  runEvery(10000)
  {
    modeToggleFlag = true;
  }
}


// Calculate current position of clock hands across the clock face DIVISIONS according to the current time; store update in hidden page
void fillSegments(DateTime time)
{
  
  //collision
  if (abs(s - s2) <= 5)
  {
    n1 *= (-1);
    n2 *= (-1) ;
  }
  if (abs(s - s3) <= 5)
  {
    n1 *= (-1);
    n3 *= (-1) ;
  }
  if (abs(s - s4) <= 5)
  {
    n1 *= (-1);
    n4 *= (-1) ;
  }
  if (abs(s - s5) <= 5)
  {
    n1 *= (-1);
    n5 *= (-1) ;
  }
  if (abs(s - s6) <= 5)
  {
    n1 *= (-1);
    n6 *= (-1) ;
  }
  if (abs(s - s7) <= 5)
  {
    n1 *= (-1);
    n7 *= (-1) ;
  }

  if (abs(s2 - s3) <= 5)
  {
    n3 *= (-1);
    n2 *= (-1) ;
  }
  if (abs(s2 - s4) <= 5)
  {
    n4 *= (-1);
    n2 *= (-1) ;
  }
  if (abs(s2 - s5) <= 5)
  {
    n5 *= (-1);
    n2 *= (-1) ;
  }
  if (abs(s2 - s6) <= 5)
  {
    n6 *= (-1);
    n2 *= (-1) ;
  }
  if (abs(s2 - s7) <= 5)
  {
    n7 *= (-1);
    n2 *= (-1) ;
  }
  if (abs(s3 - s4) <= 5)
  {
    n4 *= (-1);
    n3 *= (-1) ;
  }
  if (abs(s3 - s5) <= 5)
  {
    n5 *= (-1);
    n3 *= (-1) ;
  }
  if (abs(s3 - s6) <= 5)
  {
    n6 *= (-1);
    n3 *= (-1) ;
  }
  if (abs(s3 - s7) <= 5)
  {
    n7 *= (-1);
    n3 *= (-1) ;
  }
  if (abs(s4 - s5) <= 5)
  {
    n4 *= (-1);
    n5 *= (-1) ;
  }
  if (abs(s4 - s6) <= 5)
  {
    n4 *= (-1);
    n6 *= (-1) ;
  }
  if (abs(s4 - s7) <= 5)
  {
    n4 *= (-1);
    n7 *= (-1) ;
  }
  if (abs(s5 - s6) <= 5)
  {
    n6 *= (-1);
    n5 *= (-1) ;
  }
  if (abs(s5 - s7) <= 5)
  {
    n7 *= (-1);
    n5 *= (-1) ;
  }
  if (abs(s6 - s7) <= 5)
  {
    n6 *= (-1);
    n7 *= (-1) ;
  }




  s  += n1;
  s2 += n2;
  s3 += n3;
  s4 += n4;
  s5 += n5;
  s6 += n6;
  s7 += n7;
  //in order to make it between 0 and DIVISIONS
  if (s  >= DIVISIONS) s = 0;
  if (s2 >= DIVISIONS) s2 = 0;
  if (s3 >= DIVISIONS) s3 = 0;
  if (s4 >= DIVISIONS) s4 = 0;
  if (s < 0) s = DIVISIONS - 1;
  if (s2 < 0) s2 = DIVISIONS - 1;
  if (s3 < 0) s3 = DIVISIONS - 1;
  if (s4 < 0) s4 = DIVISIONS - 1;
  if (s5 >= DIVISIONS) s5 = 0;
  if (s6 >= DIVISIONS) s6 = 0;
  if (s7 >= DIVISIONS) s7 = 0;
  if (s5 < 0) s = DIVISIONS - 1;
  if (s6 < 0) s2 = DIVISIONS - 1;
  if (s7 < 0) s3 = DIVISIONS - 1;

  // Clear entire hidden page of segment buffer array with background color
  for (uint16_t i = 0; i < DIVISIONS; i++) {
    segment[1][i].red = (bgr_clr >> 2) & 1;
    segment[1][i].green = (bgr_clr >> 1) & 1;
    segment[1][i].blue = (bgr_clr) & 1;
  }


  for (int8_t i = -((width - 1) / 2); i <= ((width - 1) / 2); i++) {

    segment[1][(s2 + i + DIVISIONS) % DIVISIONS].red = (num1_c >> 2) & 1;
    segment[1][(s2 + i + DIVISIONS) % DIVISIONS].green = (num1_c >> 1) & 1;
    segment[1][(s2 + i + DIVISIONS) % DIVISIONS].blue = (num1_c) & 1;

    segment[1][(s3 + i + DIVISIONS) % DIVISIONS].red = (num2_c >> 2) & 1;
    segment[1][(s3 + i + DIVISIONS) % DIVISIONS].green = (num2_c >> 1) & 1;
    segment[1][(s3 + i + DIVISIONS) % DIVISIONS].blue = (num2_c) & 1;


    segment[1][(s + i + DIVISIONS) % DIVISIONS].red = (num3_c >> 2) & 1;
    segment[1][(s + i + DIVISIONS) % DIVISIONS].green = (num3_c >> 1) & 1;
    segment[1][(s + i + DIVISIONS) % DIVISIONS].blue = (num3_c) & 1;

    segment[1][(s4 + i + DIVISIONS) % DIVISIONS].red = (num4_c >> 2) & 1;
    segment[1][(s4 + i + DIVISIONS) % DIVISIONS].green = (num4_c >> 1) & 1;
    segment[1][(s4 + i + DIVISIONS) % DIVISIONS].blue = (num4_c) & 1;

    segment[1][(s5 + i + DIVISIONS) % DIVISIONS].red = (num5_c >> 2) & 1;
    segment[1][(s5 + i + DIVISIONS) % DIVISIONS].green = (num5_c >> 1) & 1;
    segment[1][(s5 + i + DIVISIONS) % DIVISIONS].blue = (num5_c) & 1;

    segment[1][(s6 + i + DIVISIONS) % DIVISIONS].red = (num6_c >> 2) & 1;
    segment[1][(s6 + i + DIVISIONS) % DIVISIONS].green = (num6_c >> 1) & 1;
    segment[1][(s6 + i + DIVISIONS) % DIVISIONS].blue = (num6_c) & 1;

    segment[1][(s7 + i + DIVISIONS) % DIVISIONS].red = (num7_c >> 2) & 1;
    segment[1][(s7 + i + DIVISIONS) % DIVISIONS].green = (num7_c >> 1) & 1;
    segment[1][(s7 + i + DIVISIONS) % DIVISIONS].blue = (num7_c) & 1;
  }
}

// Copy content from hidden to visible page; must be as time efficient as possible to avoid flickering of output
void makeVisible()
{
  for (uint16_t i = 0; i < DIVISIONS; i++) {
    segment[0][i] = segment[1][i];
  }
}
