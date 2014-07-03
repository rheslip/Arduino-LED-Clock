// Demonstration of LED multiplexing with Arduino
// Author: Nick Gammon
// Date: 2 December 2013
// turned into RTC backed up LED clock R Heslip June 20/2014


#include <Time.h> 
#include <Timezone.h>    //https://github.com/JChristensen/Timezone
#include <DS1302.h>

//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);

TimeChangeRule *tcr;        //pointer to the time change rule, use to get TZ abbrev
time_t utc, local;

// Set the appropriate digital I/O pin connections. These are the pin
// assignments for the Arduino as well for as the DS1302 chip. See the DS1302
// datasheet:
//
//   http://datasheets.maximintegrated.com/en/ds/DS1302.pdf
const int kCePin   = 10;  // Chip Enable
const int kIoPin   = 12;  // Input/Output
const int kSclkPin = 13;  // Serial Clock

// Create a DS1302 object.
DS1302 rtc(kCePin, kIoPin, kSclkPin);
// Put a suitable resistor in series with each segment LED (eg. 180 ohm)

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
ISR (TIMER2_COMPA_vect) 
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
  for (byte i = 0; i < SEGMENTS; i++)
    pinMode(columnPins[i], OUTPUT);  // make all the segment pins outputs
    
  for (byte i = 0; i < DIGITS; i++)
    pinMode(digitPins[i], OUTPUT);   // make all the digit pins outputs
  pinMode(COLON,OUTPUT);   // colon driver
  
  // set up to draw the display repeatedly
  
  // Stop timer 2
  TCCR2A = 0;
  TCCR2B = 0;

  // Timer 2 - gives us a constant interrupt to refresh the LED display
  TCCR2A = bit (WGM21) ;   // CTC mode
  OCR2A  = 63;            // count up to 64  (zero relative!!!!)
  // Timer 2 - interrupt on match at about 2 kHz
  TIMSK2 = bit (OCIE2A);   // enable Timer2 Interrupt
  // start Timer 2
  TCCR2B =  bit (CS20) | bit (CS22) ;  // prescaler of 128


  // Make a new time object to set the date and time.
  // note we have to set the RTC to UTC so the timezone stuff works
  // y, m, d,hr(24),min,sec,day
  Time t(2014, 6, 25, 01,02, 0, Time::kWednesday);
   // Set the time and date on the chip.
//  rtc.time(t);   // uncomment to set the RTC to the time above - 
                  // recompile and reflash or it will set the RTC again next power up
    
 // setTime(myTZ.toUTC(compileTime())); // get compile time

  setSyncInterval(60);  // sync to RTC every minute - RC clocked AVR is drifty
  setSyncProvider(RTCTimeSync); 
  Serial.begin(9600);
    setSyncProvider(RTCTimeSync);    // the function to get the time from the RTC
    if(timeStatus()!= timeSet) 
        Serial.println("Unable to sync with the RTC");
    else
        Serial.println("RTC has set the system time");       
  delay (1000);  // give time to read "HELO" on the display
  } // end of setup
  

void loop ()
  {
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
 
} // end of loop


time_t RTCTimeSync(){
  // returns time_t from RTC date and time with the given offset hours
  tmElements_t tm;
  Time t = rtc.time(); // store the current time in time variable t 
  int year;
  tm.Year = t.yr - 1970; 
  tm.Month=t.mon;
  tm.Day=t.day; 
  tm.Hour=t.hr; 
  tm.Minute=t.min; 
  tm.Second=t.sec;
  time_t time = makeTime(tm);
//  return time + (offset * SECS_PER_HOUR); // I'm assuming RTC is local time
  return time;
}

/*
//Function to return the compile date and time as a time_t value
time_t compileTime(void)
{
#define FUDGE 25        //fudge factor to allow for compile time (seconds, YMMV)

    char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char chMon[3], *m;
    int d, y;
    tmElements_t tm;
    time_t t;

    strncpy(chMon, compDate, 3);
    chMon[3] = '\0';
    m = strstr(months, chMon);
    tm.Month = ((m - months) / 3 + 1);

    tm.Day = atoi(compDate + 4);
    tm.Year = atoi(compDate + 7) - 1970;
    tm.Hour = atoi(compTime);
    tm.Minute = atoi(compTime + 3);
    tm.Second = atoi(compTime + 6);
    t = makeTime(tm);
    return t + FUDGE;        //add fudge factor to allow for compile time
}

//Function to print time with time zone
void printTime(time_t t, char *tz)
{
    sPrintI00(hour(t));
    sPrintDigits(minute(t));
    sPrintDigits(second(t));
    Serial.print(' ');
    Serial.print(dayShortStr(weekday(t)));
    Serial.print(' ');
    sPrintI00(day(t));
    Serial.print(' ');
    Serial.print(monthShortStr(month(t)));
    Serial.print(' ');
    Serial.print(year(t));
    Serial.print(' ');
    Serial.print(tz);
    Serial.println();
}

//Print an integer in "00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintI00(int val)
{
    if (val < 10) Serial.print('0');
    Serial.print(val, DEC);
    return;
}

//Print an integer in ":00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintDigits(int val)
{
    Serial.print(':');
    if(val < 10) Serial.print('0');
    Serial.print(val, DEC);
}
*/
