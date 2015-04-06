Most of the source code for my sensor network project documented at rheslip.blogspot.ca. It has been
modified so as not to reveal my Twitter account info etc.

This code uses the Arduino MySensor library from Mysensors.org. The sensor network gateway is a mega328 running an unmodified
version of the Mysensors gateway sketch.

The controller code runs on a Mega1284. The serial output of the gateway is connected to the Mega1284's Serial1 port. I used a SPI ethernet
ENC28J60 module and the Ethercard library to connect the Mega1284 to my LAN and the internet. It posts network status messages to Twitter
via a free Twitter gateway service (link is in the source code). There are numerous library dependancies in the code and you will have to 
look at the source to find which ones you need. I don't expect anybody to use this code as is because it is very specific to my network, my sensor 
nodes and what I wanted to do.

Most of the code for the sensor nodes is here too. Its largely based on the examples from MySensors.org. The
clock nodes are cobbled together from various libraries and code I wrote.

