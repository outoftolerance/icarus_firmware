# Icarus Firmware
Icarus is a High Altitude Balloon project designed with specific goals in mind.
 - Take a picture of the curvature of the earth
 - Take a picture of the earth and space
 - Learn new things through project research (atmosphere, weather, etc)
 - Develop engineering skills (Software and Electrical)

The code for this project is written in the Arduino language so that it is portable and easy to understand for other people who might want to create a similar project of their own, or extend the project in their own way.

# Hardware


## Microcontroller
For this project we use the `ATSAMD21G18A-U` Microcontroller, this is supported through the Arduino project as the Arduino Zero and through Adafruit as their Feather M0 board (as well as other variants available). We make use of the Adafruit version of the board support package as it allows more flexibility in terms of serial ports by not using the EDBG interface that is by default used by the Arduino Zero, instead it uses the USB interface.

We chose this specific microcontroller because it has a feature called SERCOM that allows for in-software definition of serial, I2C, and SPI interfaces using different pins. Since we have a large number of serial peripherals for this project (GPS, Radio, Cellular, APRS) we needed to be able to define more than the standard single serial port for an Arduino board. While it would be possible to define additional serial interfaces used the `SoftwareSerial` library, this is generally not good practice and has many implementation pitfalls.

Additionally, the chip is not very large. We had the option of using the `ATMEGA2560` (as used on the Arduino Mega board, which has four serial interfaces) however the physical size (and cost!) of that microcontroller is significantly larger in comarison. We also get other side benefits such as more program memory, and faster clock speed, that aren't NEEDED for this project but are a nice to have.

## Communications
Communications is one of the major difficulties when designing a High Altitude Balloon because of the distances that it has to travel and the fact that the system needs to be recovered at a random, unknown, future location (or you lose all your hard work!). For this reason, there are multiple redundant communications interfaces that have been designed into the vehicle.

### Packet Radio
The first, and simplest, form of communication between the vehicle and us on the ground is a packet radio. Essentially this works as a serial port over radio waves and allows data to be passed from the vehicle to the ground (such as position information) as well as allowing commands to be send to the vehicle during operation.

We chose an [RFD900+ radio modem](http://store.rfdesign.com.au/rfd-900p-modem/) for this project, it is well known within the drone industry as a reliable and well-performing radio that can be configured and paired with different antenna systems to provide exceptional range.

This communications interface will be used to communicate the following information:
- Telemetry from vehicle
- Position from vehicle
- Events from vehicle
- Commands to vehicle

### Cellular
Secondly we will use a cellular modem to send and receive data from the ground to the vehicle. This method is not reliable as a tracking system for the vehicle throughtout the flight, however is robust at low altitudes within cellular network reception range. It is also a flexible interface, it can either send text messages to us, or data over the internet, or a combination of both!

The reason we added this radio was as a backup in-case the other tracking systems fail, or when at low altitudes, radio signals from the other two methods are obscured by terrain or ground objects where the balloon lands. The cellular connection should be able to text message us with the location of the vehicle at low altitudes and when at rest.

The communucations interface will be used to communicate the following information:
- Position from the vehicle
- Events from the vehicle

### APRS
An APRS (Automatic Packet Reporting System) radio has also been added, this allows for the tracking of the balloon through the HAM radio APRS network of receivers, and potentially from our own receiver. The system is very limited in terms of how frequently data can be transmitted and how much data can be transmitted at a time, however it is very very long range and is well supported as a method of tracking vehicles. 

We chose the [MicroTrak 2000](https://drive.google.com/open?id=1uPaUfZ47ibVQPEURd_sj6lUmIUkslmQF) APRS transmitter from Byonics for this project because of its small form-factor, high power, and easy integration into our custom electical and mechanical design. This model is no longer in production and has been superceded by an even smaller form-factor device, the MicroTrak 2001.

This communications interface will be used to communicate the following information:
- Position from the vehicle

## Sensors
There are some sensors needed for basic functionality of the balloon (such as a GPS receiver) and others are just for fun!

### GPS
A GPS receiver is a basic requirement for being able to position the vehicle and track it during its mission. We chose a basic off-the-shelf GPS receiver from Adafruit that has been used in high altitude balloon projects with success in the past, the [Adafruit GPS V3](https://www.adafruit.com/product/746).

### IMU
An IMU is added to give basic orentation information, as well as being able to detect freefall and other events such as touchdown. We chose a simple all-in-one IMU board, the [Adafruit 10DOF](https://www.adafruit.com/product/1604). This is no longer available but still functions great and we already had one.

### Atmospheric
None of these sensors are really neded, however the data they produce is fun to look at after the flight and get an idea of the make-up of our atmosphere! We chose to add temperature, humidity, and gas sensors to the project, the specific models and boards haven't been defined yet but will come.

## Interconnects
The different elements of the system are connected together using Serial, I2C, and SPI interfaces, as well as a few GPIO used for some smaller peripherals. The folliwing diagram shows how each element of the system is connected.

![Hardware Interface Connection Diagram](documents/hardware_interface_connection_diagram.png?raw=true)

# Software
The software for this project is likely to be much more complex than is really required for such a simple mission, however part of the goal of this project was to learn new techniques and to advance my knowledge of object-oriented programming, abstraction, division of responsibility, and writing extensible and maintainable code. 

For this reason different sub-systems are built into their own libraries, these libraries are described at a functional level here, and at a technical level in the header file comments for each package.

## Mission States
In order to control the different functions of the balloon at different stages in the flight, a state machine was developed which tracks mission state based on inputs such as telemetry, button presses, and command messages.

![Mission State Diagram](documents/software_mission_state_diagram.png?raw=true)

### State Descriptions

**Staging:** While waiting to launch and on the ground, used to prepare the vehicle and sub-systems.

**Takeoff:** Once ready to takeoff and all staging work has been completed this state is engaged manually.

**Ascending:** Once takeoff has completed and the vehicle is ascending in altitude.

**Descending:** Once ascension has completed and descent has started.

**Landing:** When vehicle is about to land and has descended almost to the ground.

**Receovery:** Once touchdown has been detected and the vehicle is waiting for recovery.

**Receovered:** When recovery has been detected but power-off is pending.

### State Table

|State|Configuration|Entrance Condititions|Exit Conditions|
|-----|-------------|---------------------|---------------|
|Staging   |<ul><li>Telemetry Reporting Frequency: 1/5 Hz</li><li>Telemetry Logging Frequency: 1/5 Hz</li><li>Position Reporting Frequency: 1/30 Hz</li><li>LEDs: On</li><li>Beeper: Off</li></ul>|<ul><li>State is set by default at boot.</li><li>Launch switch is flipped to staging and current mission state is `Takeoff`.</li></ul>|<ul><li>Launch switch is flipped to `Takeoff` from `Staging`.</li></ul>|
|Takeoff   |<ul><li>Telemetry Reporting Frequency: 1/15 Hz</li><li>Telemetry Logging Frequency: 1/5 Hz</li><li>Position Reporting Frequency: 1/15 Hz</li><li>LEDs: On</li><li>Beeper: On</li></ul>|<ul><li>Launch switch is flipped to `Takeoff` from `Staging`.</li></ul>|<ul><li>Launch switch is flipped to `Staging` from `Takeoff`.</li><li>Altitude increases above 500m.</li></ul>|
|Ascending |<ul><li>Telemetry Reporting Frequency: 1/60 Hz</li><li>Telemetry Logging Frequency: 1/5 Hz</li><li>Position Reporting Frequency: 1/30 Hz</li><li>LEDs: Off</li><li>Beeper: Off</li></ul>|<ul><li>Altitude increases above 500m.</li></ul>|<ul><li>Accelerometer detects freefall or altitude starts dropping for 5 consecutive seconds.</li></ul>|
|Descending|<ul><li>Telemetry Reporting Frequency: 1/60 Hz</li><li>Telemetry Logging Frequency: 1/5 Hz</li><li>Position Reporting Frequency: 1/30 Hz</li><li>LEDs: Off</li><li>Beeper: Off</li></ul>|<ul><li>Accelerometer detects freefall or altitude starts dropping for 5 consecutive seconds.</li></ul>|<ul><li>Altitude decreases below 500m.</li></ul>|
|Landing   |<ul><li>Telemetry Reporting Frequency: 1/60 Hz</li><li>Telemetry Logging Frequency: 1/5 Hz</li><li>Position Reporting Frequency: 1/5 Hz</li><li>LEDs: On</li><li>Beeper: On</li></ul>|<ul><li>Altitude decreases below 500m.</li></ul>|<ul><li>Accelerometer stops detecting motion or altitude stops changing for 5 consecutive seconds.</li></ul>|
|Recovery  |<ul><li>Telemetry Reporting Frequency: 1/300 Hz</li><li>Telemetry Logging Frequency: 1/5 Hz</li><li>Position Reporting Frequency: 1/300 Hz</li><li>LEDs: On</li><li>Beeper: On</li></ul>|<ul><li>Accelerometer stops detecting motion or altitude stops changing for 5 consecutive seconds.</li><li>After waiting 5 minutes in Recovered mode.</li></ul>|<ul><li>Silence button is pressed.</li></ul>|
|Recovered |<ul><li>Telemetry Reporting Frequency: 1/300 Hz</li><li>Telemetry Logging Frequency: 1/5 Hz</li><li>Position Reporting Frequency: 1/300 Hz</li><li>LEDs: Off</li><li>Beeper: Off</li></ul>|<ul><li>Silence button is pressed.</li></ul>|<ul><li>After waiting 5 minutes in Recovered mode.</li></ul>|