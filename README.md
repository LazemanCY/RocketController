# Rocket Onboard Controller
It's a controller based on Arduino.<br />

## MCU
ATmega328p developed under Arduino IDE.<br />
 

## Sensor
It's I2C port is attached with a GY-87 module(MPU6050, BMP085, HMC5883).<br />


## What can it do
Function is very simple right now. <br />
It read acc data and dectect the ignition moment and start to count a timer.<br />
When timer is over, it switch the MOS and turn the heater strip on.<br />
Then the heater strip cut the rope and release the spring to push the parachute out off the hatch. <br /> 

## Wiring


## How to Use
