# XJ/TJ Bus Arbiter
## Teensy Source

This code contains a modified version of https://github.com/laszlodaniel/CCDLibrary,
stripped down to support only write functionality since the only other transmitter
in the Jeeps will be the airbag module.  It is written to use the CDP68HC68S1 
integrated CCD transceiver, which though discontinued, is valuable as it handles
bus arbitration internally, meaning it will prevent our Teensy from stomping on the
airbag module when it wants to speak.

The code adds a high level abstraction called Instruments around each of the
dash's gauges and lamps.  In main.cpp, the code simply sets the value of
each instrument or the state of the lamp via high level, unit based calls such
as SetFuelPercentage or SetMPH.  Behind the scenes, the instrument subroutine
keeps track of which CCD messages should be sent when values change, and
which should be repeated periodically.  

Also in main.cpp is the setup for CAN bus reception.  The actual CAN messages 
to listen to depend on application, but the general theory is that when a 
relevant CAN message is received, a handler function can decode it and decide
whether to set an Instrument or to transmit a subsequent CAN message or messages.

The code also includes a routine to sample input voltage (e.g. 12V battery
voltage) every half second.  It does this with the help of an optocoupler
(solid state relay) between VBAT and a voltage divider in order to not drain
the battery when the vehicle is off.  The source has constants for the R1/R2
values of the voltage divider, correct if you are using the matching design,
but adjust those values if you adjust the resistors.

Finally, the software sets up a Teensy watchdog timer which will reset the Teensy
if there is no CCD transmit activity.  Watchdog "feeds" could also be added
to CAN reception if desired.  If no activity occurs in 5 seconds, a warning
is sent to the Serial output and 5 seconds later the Teensy will reset.  This 
should be customized as desired. 