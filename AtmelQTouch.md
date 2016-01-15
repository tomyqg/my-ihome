# QMatrix #
## 5.6.7.2 QMatrix acquisition method constraints ##

In case of XMEGA™ devices, the resources are used internal to the library and hence cannot be used by the host application
  * Timer/Counter 1 on PORTC (TCC1)
  * Analog Comparator on PORTA (ACA)
  * Event System Channel0 (EVSYS\_CH0)

The AIN0 pin of the device needs to be connected to the GND.
  * In case of XMEGA devices, the **reference pin for input to analog comparator is Pin7 of PORTA with all the combinations of libraries** supported. Hence, this needs to be connected to GND.

## 5.8.2 Pin Configuration for QMatrix Acquisition Method ##
The QMatrix acquisition method libraries needs to be used after configuring X and YA and YB lines on IO pins of the port as described in the configuration rules described in the section below. The QTouch Studio Pin Configurator Wizard can be used to assign X, YA, YB, SMP lines on the pins and rules are internally taken care in the Qtouch Studio Pin Configurator Wizard.

The snippets can be taken from the QTouch Studio Pin Configurator Wizard and copied to appropriate places in the main.c and touch_config.h files in the example projects provided._

### 5.8.2.1 Configuration Rules: ###
  1. The X lines can be configured on different ports up to a maximum of 3 ports. Ex: NUM\_X\_PORTS = 3 (maximum value supported). However the possible values are NUM\_X\_PORTS = 1 or NUM\_X\_PORTS = 2 or NUM\_X\_PORTS = 3
  1. The X lines can be configured on the three different ports.
  1. The X lines can be configured on any pins of the ports selected above. Ex: X0 on PB2, X1 on PD5, X2 on PE0, X3 on PD1( when NUM\_X\_LINES= 4), Provided that these pins do not conflict with the other pins for touch sensing or with the host application usage.
  1. The Y lines can be configured on the any of the pins of the ports selected. Ex: Any pins on the PORT\_YA and PORT\_YB selected. Suppose, PORT\_YA is D, and PORT\_YB is C. Since, pin 5 and pin 1 PORTD are already used for X lines(X1, X3), the user can select any of the remaining pins for Y0A lines. Suppose that Y0 is on pin2 and Y1 is on pin6. Hence, Y0A – PD2, Y0B – PC2, Y1A – PD6, Y1B – PC6,
  1. All the Qmatrix usage Pins X lines,YA lines, YB lines and SMP line can be on same port.Both YA and YB lines can share the same port. And the YA and YB need not be on same corresponding pins of the ports.The Macro SHARED\_YAYB should be defined as 1 if YA and YB are on same port else should be defined as 0.
  1. **The PORT\_YB is fixed for each device and should be same as the PORT on which the ADC input pins are available.**
  1. The SMP pin can be configured on any of the IO PORT pins available.
Ex: PORT\_SMP = D
SMP\_PIN = 7 as this pin is not being used by touch sensing.

## Atmel Studio Help ##
### X & Y Port-Pin selection page ###
The X and Y Line Selection Page allows the user to select a port and pin for each of the X- and Y-lines. As with the QTouch touch technology, the list of pins available for selection will be dependend on the number of pins provided for each port. This will be dictated by MCU architecture, 8 or 32-bit will correspond to 8 or 32 pins per port.

In addition to the general X- and Y-lines, some special pins needs to be placed on a port and pin as well.

  * YB-ADC port (this is generally fixed based on the MCU selected)
  * AIN0 port (fixed based on the MCU selected)
  * SMP port

### Rules & Guidelines ###

  * Same port cannot be used more than once.
  * The X-lines can be assigned pins from up to 3 different ports.
  * The X-lines can be configured on any pin of the ports selected for X-lines provided that these pins do not conflict with the other pin usage, either by the QTouch Library or by the host application.
  * The Y-lines can be configured on any pin of the ports selected for Y-lines provided that these pins do not conflict with the other pin usage, either by the QTouch Library or by the host application. Also, ports YA and YB are linked regarding pin usage. E.g. if pin 1 is used by port YA then the corresponding pin on YB is also used as these need to work together.
  * The YB port is fixed for each specific device and will be (one of) the ADC port(s) of the MCU.
  * The SMP pin can be configured on any of the available port pins.