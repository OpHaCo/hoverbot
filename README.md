# HoverBot : the most affordable smooth and silent two wheels drive control

**NOTE: This project is currently abandoned**  
There is more active development over at https://github.com/NiklasFauth/hoverboard-firmware-hack. We already have a working custom firmware for the hoverboard mainboard, complete with ADC, PPM and Nunchuck control. (See the linked video in the description there)

<img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/img/hoverbot_dog.jpg" width="300">
<img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/img/hoverbot_trash.jpg" width="320">

Project has been done in Amiqual4Home Equipex Creativity Lab - https://amiqual4home.inria.fr/

## **Description**

With hoverbot, you can do some vehicle, chassis :
 * supporting up to 264lbs (120kg). 
 * moving silently thanks to two 350W brushless motors in wheels
 * locally or remotely controlled, 
 * at an unbeatable price buying some spare parts :
    * Mother board - 55  eur
    * 36V battery  - 100 eur
    * two wheels   - 160 eur
    * additional components - 40 eur
    * **Total = 355 eur**
 * or more cheaper :
    * just buy an hoverboard and get these parts from it - 260eur
    * additional components - 40eur
    * **Total = 300eur**

Hoverboard are chinese low cost products. Low cost spare parts gives meaningful two wheels drive platform.

You don't have to be an electronic master to do it. To have motors working, you just have to connect hoverbot shield. You can either make this shield by yourself or you can get it from (TODO).

With hoverbot you will be able to drive your : 
* robot
* chair
* old dog
* couch

## **Prerequisities**

### **hardware**

* [Two hoverboard motors] (http://gyro-service.com/moteur/19-moteur.html)
* [Hoverboard mother board] (http://gyro-service.com/cirquit-carte-mere-smart-self-balancing-wheel/17-carte-mere.html)
* [36V battery] (http://gyro-service.com/batteries-samsung-smart-balance-wheel-gyropode/18-battterie-22p-samsumg-ou-de-qualite-equivalente.html)
* [hoverbot shield](#hoverbot-shield) 
* Arduino Teensy 3.1/3.2

## **Setup**

### **Components Analysis**

Refer [ifixit](http://ifixit.org/blog/7821/swagway-teardown-hoverboards/) for a detailed analysis of hoverboard components.
Interesting parts are :
 * how do these components interact?
 * is it possible to control hoverboard externally?

### **Daughterboard / motherboard communication**

Daughterboard can be used either on right/left. So we only bought 1 daughterboard to decode Daughterboard / motherboard exchanged data.
We connected the 2 motherboard / daughterboard connectors to a single motherboard.
In this case when rotating motherboard, wheels are rotating in two different direction.
 <img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/img/hover_bot_setup.jpg" width="700">

#### **Protocol**
Probing a logic analyser on daughterboard 4 wire connectors gives some interesting info :
 * logic level = 3.3V
 * bit duration ~ 38.10µs => baud rate ~ 26300kbps
 <img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/img/bit_duration.png" width="900">
 * there is no clock provided on given lines => Async protocol 
 
Is it a standard protocol? 
 * may be UART? Some clues can help... Green and yellow lines of daughterboard are connected to [STM32F103](http://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf) PA2 - UART2 TX and PA3 - UART3 RX
In order to get UART parameters : 
 * get maximum frame length taking for example a frame of 0 or a frame of 0/1/0/ and so on... After that you will conclude (helped with Salae logic ananlysers):
**Daughterboard send data to motherboard using 3.3V UART @26300bps - 1 stop bit - no parity bit.** 

#### **Exchanged data**

Here are decoded data with  UART analyzer in Salae Logic software :

 <img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/img/uart_frame.png" width="900">

Do not move daughterboard, power motherboard and read decoded data. We can see :
 * redondant 256 integer : it is frame start => frame size = 6 9bits integers

Now let's play with photodiode used in daughterboard contacts :
 * if we hide at least one of it  last frame integer = 85
 * otherwise last frame integer = 170
If 170 value is sent to motherboard : motors won't be driven (it means user to not have its feet on hoverboard) (it means user to not have its feet on hoverboard)
 
4 remaining bytes must be decoded : two of these bytes repeats.
Rotating daughterboard gives indication :
 * little move only modify a frame integer => this integer must be low significant 9bit 
 * more rotation modifies other integer    => rotation seems to be encoded on 2 9 bit integers.
Rotating around 0 helps us :
**Angular value is coded in 2x9bits signed integer value (2's complement)**

We can check it is only an angular value applying linear acceleration to daughterboard. In this case angular value do not change.

#### ** Speed sensor **
In wheels some hall sensors are used to get wheel speed (3 sensors per wheel).
On mother board, sensor pulses are filtered using an RC filter. To get speed, we must connect it to hoverbot. 

 <img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/hoverbot_embedded_software/teensy_hoverboard/brushless_hall_sensor/img/detect_edge_bug/brushless_sensor_no_trigger.png" width="900">

If we trigger a digital output (channel 0) on a rising edge (channel 0) we do not have a clean signal :
 <img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/hoverbot_embedded_software/teensy_hoverboard/brushless_hall_sensor/img/detect_edge_bug/edge_detection_global_ko.png" width="900">

With a zoom :
 <img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/hoverbot_embedded_software/teensy_hoverboard/brushless_hall_sensor/img/detect_edge_bug/edge_detection_zoom_ko.png" width="900">

Small oscillation when V=Vdd/2 generates some falling /rising edges. Because of RC time constant, we can get lot of oscillations leading to some false detections. It could be software filtered (either using timer or checking multiple times read value). As I prefer capturing Hall sensor pulses using external interrupt it is better to have an hardware solution. As Teensy MCU does not have any input schmitt trigger we will add it :

 <img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/hoverbot_embedded_software/teensy_hoverboard/brushless_hall_sensor/img/detect_edge_bug/brushless_sensor_trigger.png" width="900">

After we have a good pulse detection :

 <img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/hoverbot_embedded_software/teensy_hoverboard/brushless_hall_sensor/img/detect_edge_bug/edge_detection_zoom_ok.png" width="900">
 
### **hardware** 

In order to control hoverboard we need an external MCU to :
 * send left and right foot angular position => we need two UART Tx lines supporting 9bits UART @ 26300bps
 * optionally read daughterboard current postion (for tests) => we need 1 UART Rx line supporting 9bits UART @ 26300bps
 
Different MCU can be used, we could use UART bitbanging libraries... But a nice solution is to use teensy 3.1/3.2 :
 * it runs @3.3V
 * [9 bit uart supported on Arduino Core] (https://www.pjrc.com/teensy/td_uart.html)
 * 3 avaialble hardware UARTs!



### **software** 

#### Prerequisities
##### Enable 9-Bit UART on Teensy
Edit :

    your_arduino_core_folder/hardware/teensy/avr/cores/teensy3/HardwareSerial.h

Uncomment :

    #define SERIAL_9BIT_SUPPORT

##### Add needed libraries
Several libraries are provided in this repo : 
 * hoverboard : hoverboard control / feddbacks
 * brushless_hall_sensor
 * logger 

If you use arduino IDE, you must create symlinks/shortcuts from these libraries path to Arduino library folder :

    ln -s your_folder/hoverbot/hoverbot_embedded_software/teensy_hoverboard/hoverboard arduino_user_path/libraries/
    ln -s your_folder/hoverbot/hoverbot_embedded_software/teensy_hoverboard/brushless_hall_sensor arduino_user_path/libraries/
    ln -s your_folder/hoverbot/hoverbot_embedded_software/teensy_hoverboard/logger arduino_user_path/libraries/

Then from Arduino IDE add these libraries.

##### Configure logging level
    
Edit 
    your_folder/hoverbot/hoverbot_embedded_software/teensy_hoverboard/logger/logger_config.h
    
and set needed log level (refer your_folder/hoverbot/hoverbot_embedded_software/teensy_hoverboard/logger/README.md)
    
### Arduino sketch


As we saw in our first tests, when same daughterboard is connected to two motherboards/daughterboard connectors :
 * wheels rotating direction is different. Hoverboard is rotating
In this case same angular value on left foot and right foot is sent to motherboard.
In order to have hoverboard moving forward / backward we must apply for example :
 * a positive angular value on right foot daughterboard Uart
 * negative angular value on left foot daughterboard Uart

From that  simple code is provided here :
[Refer code](https://github.com/OpHaCo/hoverbot/blob/master/hoverbot_embedded_software/teensy_hoverboard/teensy_hoverboard.ino)
In `echoGoForward()`  function : it reads current angular value from daughterboard to control hoverboard in forward direction.

After testing we can easily : 
 * control speed of both wheels when it is rotating in different direction (under load or not)

But we cannot :
 * control speed of both wheels when it is rotating is same directions (not under load). **In this case, motors reach needed value but quickly speed up until max speed**
 
We can suppose, closed loop algorithm on motherboard does not support forward / backward mooving when it is not under load.

## Interfaces
### UART

Hoverboard control over UART can be done running :

    python3 your_folder/hoverbot/hoverbot_gateway_software/examples/hoverboard_uart_control/hoverboard_uart_control.py -p TEENSY_PORT
    
<img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/img/hoverbot_uart.png" width="800">

#### Supported commands :

    power_on

    power_off
    
    set_speed motor1_value motor2_value
    
    x // Manual keyboard control => hoverbot controlled using numeric pad

### SPI
### Wifi
### BLE

## Commands
TODO

### Control commands
TODO
###ROS
## Events
TODO

## Hoverbot shield
### PCB design

## Examples
### Racing chair

<img src="https://raw.githubusercontent.com/OpHaCo/hoverbot/master/img/racing_chair.jpg" width="600">

## Issues

## References
* http://www.electronicspoint.com/threads/reverse-engineering-a-hoverboard-with-wheels-accelerometer.279098/
* http://drewspewsmuse.blogspot.fr/2016/06/how-i-hacked-self-balancing-scooter.html
* https://developer.mbed.org/users/Thomas_H/code/HoverboardTest/


## Next steps

* improve hoverboard control
* continue cleaning (remove some unused attributes in hoverboard)
* add PID : measure reliable speed from hall sensors
* use all days a reliable racing chair
* make hoverbot shield : deliver CAD
* ROS compatible?
* get error status
