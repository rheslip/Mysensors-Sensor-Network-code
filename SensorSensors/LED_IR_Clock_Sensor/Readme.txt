
Simple LED clock which uses an inexpensive common anode .56" 4 digit clock display from Banggood.com:

http://www.banggood.com/Wholesale-1-Pcs-7-Segment-4-Digit-Super-Red-LED-Display-Common-Anode-Time-p-35503.html

Based on LED mutiplexing code by Nick Gammon www.gammon.com.au  

Uses Arduino timezone library:

//https://github.com/JChristensen/Timezone

With an AVR running at 5V the segments need 1.2k current limiters as shown on Nick Gammon's HW example:

http://www.gammon.com.au/forum/?id=12314

July 2014 -This version uses the MySensor library to sync its clock with the network controller.

Oct 2014 - added IR decoding library so it can work with the home theatre setup - looks like this conflicts with the LED mux interrupt
