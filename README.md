# Arduino Bosch to GM temp

We switched out the GM oil temp sensor in a corvette c4 -90 to
a Bosch temp and pressure sensor (0 261 230 340).

This sensor is read by a new ECU (MaxxECU) but we want 
the oil temp gauge on the instrument cluster to work.

The solution I came up with is to emulate the GM sensor based 
on values from the Bosch sensor.

The project has a series of resistors that can individually be bypassed by a solid state relay.

The relays are controlled by an arduino that reads the voltage from the sensor and does the math to control the relays that in turn ensure that the resistance in the circut is the same as for the old GM version.