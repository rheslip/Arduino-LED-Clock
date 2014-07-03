Simple LED clock which uses an inexpensive common anode .56" 4 digit clock display from Banggood.com:

http://www.banggood.com/Wholesale-1-Pcs-7-Segment-4-Digit-Super-Red-LED-Display-Common-Anode-Time-p-35503.html

Based on LED mutiplexing code by Nick Gammon www.gammon.com.au  

Uses Arduino DS1302 library and timezone library:

//https://github.com/JChristensen/Timezone

With an AVR running at 5V the segments need 1.2k current limiters as shown on Nick Gammon's HW example:

http://www.gammon.com.au/forum/?id=12314

To set the clock, enter the current time in the Time object creation in Setup():

 // y, m, d,hr(24),min,sec,day
  Time t(2014, 6, 25, 01,02, 0, Time::kWednesday);

uncomment the next line which calls to rtc.time(t) which will set the RTC time. Run the sketch, recompile with the call to rtc.time(t) commented out again and reload the sketch. This prevents the RTC from being reinitialized if power goes off. 
