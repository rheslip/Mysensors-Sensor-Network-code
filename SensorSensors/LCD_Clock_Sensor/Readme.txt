Code for an Arduino LCD clock/weather station July 2014 RH

This is a fork of my other clock/weather station implementation based on a Mega1284 and a serial GPS for getting the time. This code syncs time using the MySensor RF library. The 328 has much less memory so we don't save as much of the weather history data. It still saves about 2 days of data but samples it only every 30 minutes.

The outdoor temperature and humidity comes from a Sharper Image weather station wireless sensor I happen to have - the weather station died but the sensor still works. It uses a funky 40 bit/36 bit 433mhz ASK pulse modulation scheme. The protocol uses 2ms hi 6ms lo pulse for a zero a 6ms hi-2ms lo pulse for 1. The receiver I used is an RF Solutions AM-RRS3-433 module. No doubt you will have to change that code for whatever outside sensors you have.

I used a 320x240 TFT color display I happened to have but the code should work with minor modifications with the AdaFruit TFT displays. It uses the AdaFruit graphics library with a bit of code added on to draw larger fonts for the time. The AdaFruit font looks pretty blocky when blown up since its only a 5x7 matrix.

There is an as yet unfixed bug in the graphing code - sometimes it draws the last line segment incorrectly. The last point is actually the the oldest data instead of the newest so its an array indexing/wraparound issue.

July 2014
- switched to MySensor RF code for syncing time - RTC drifts
- I can eliminate the 433mhz radio and get the same data from my GPS clock over the sensor network

March 2014
- added PWM LED brightness on Digital 3
- LDR on analog 0 to sense ambient light. voltage goes down as light does
- added DST switch to GND on analog 1