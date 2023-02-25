Small Integrated Lights Out manager
===================================

A microcontroller approach to monitoring a serial port and control
the power and reset buttons.

The name SMILO is a wink to products such as iLO, ALOM, ILOM, iDRAC
and BMC, where obviously the capabilities of SMILO are much less, and
integration with the host computer is almost nihil.  The main features
of SMILO are Serial-over-LAN (SoL) and power and reset button
support.

SMILO is currently built on top of an OLIMEX ESP32-EVB which contains
all the components one needs: LAN-port, 2x relais, UART to connect
pins to the serial port of the host board.


Buiding SMILO
-------------

SMILO currently is built using Arduino IDE.  Select the OLIMEX ESP32-EVB
as board, and compile/upload.


Author
------

Fabian Groffen
