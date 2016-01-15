# Programming Environment #
As programming environment Atmel Studio was chosen due to its integration and comprehensive solution. Out of the box user receives IDE, cross-compiler (avr-gcc), device programming tooling, simulator, debugger and more. Functionality may also be enhanced with Atmel Gallery (plug-in like extension platform).

## Atmel Software Framework ##
The ASF is a MCU software library providing a large collection of embedded software for MCUs. Normally integrated with AtmelStudio, however latest version can be found here [ASF](http://www.atmel.com/tools/AVRSOFTWAREFRAMEWORK.aspx).

Other approach is to use Linux environment and Eclipse as IDE.

## Atmel Studio ##
Latest version available on Atmel's site [AtmelStudio](http://www.atmel.com/tools/ATMELSTUDIO.aspx).

# Microcontroller #
To satisfy project needs AVR XMEGA uC was chosen as a primary device to run real-time applications.
XMEGA family detailed description can be found [here](http://www.atmel.com/products/microcontrollers/avr/avr_xmega.aspx?tab=documents).

## D Series ##
System blocks are targetted on D4 series MCU and ATxmega32D4 specifically.
More information can be found here: [32D4](http://www.atmel.com/devices/ATXMEGA32D4.aspx).

D4 offers enough processing power and internal interfaces and peripherals to run applications with specific tasks.

## In-System Programmer (ISP) ##
XMEGA uC use PDI as programming interface. Two solutions has been tested and working fine under Atmel Studio 6. First is Atmel's AVR JTAGICE mkII [here](http://www.atmel.com/tools/AVRJTAGICEMKII.aspx). The latter is a clone of original device supplied by 3rd party companies.
USB AVR PROG MKII was tested and is working fine with XMEGAs A and D series. In addition this programmer provides power supply to programmed device eliminating need for use external supply. Description can be found here [clone](http://www.sibit.pl/sklep/?25,usb-avr-prog-mkii).

# Concurrent Versioning System #
  1. As the primary versioning system SVN is being used. For Windows operating systems one can use a Subversion client **TortoiseSVN** available [here](http://tortoisesvn.tigris.org/).
  1. The other solution is to use AnkhSVN plugin to Atmel Studio. Installation is available from Atmel Gallery. More details can be found [here](http://ankhsvn.open.collab.net/).

  1. Remember to commit your changes often. Once a day/change is a preferred approach. Do not forget about inserting appropriate comment.

## Repository at code.google.com ##
Detailed information and how-to is available at the following location [how-to](https://code.google.com/p/my-ihome/source/checkout).
  * Hint: In case of first repository creation use the following location/address: 'https://my-ihome.googlecode.com/svn/trunk/'
(secured protocol version **https** must be used).
  * Put your code under _/trunk/Software/your\_module\_here_ location.

## [future use](for.md) BitBucket ##
  1. Location: https://bitbucket.org/tfidecki/myhome

# Hardware design #
Due to large components database and community support it is preferred to use CadSoft Eagle application for schematics and PCB design.

  1. Many tutorials are available on Sparkfun webpages [BetterPCBs](https://www.sparkfun.com/tutorials/115)

## Design rules ##
At this time it is recommended to follow the rules defined by Sparkfun [EagleRules](http://hades.mech.northwestern.edu/images/0/08/Eagle_Rules.pdf)

It is always good to know your final PCB will look like in reality. There are many tutorials available to create 3D model from Eagle. Google will assist you [helpMe](https://www.google.pl/#sclient=psy-ab&q=eagle+3d+tutorial&oq=eagle+3d+tut&gs_l=hp.3.0.0j0i30j0i5i30j0i8i30.1059.3184.0.4307.12.11.0.1.1.0.155.1455.0j11.11.0...0.0.0..1c.1.17.psy-ab.Hdl7zDgDxhE&pbx=1&bav=on.2,or.r_cp.r_qf.&bvm=bv.47810305,d.bGE&fp=ad2cd779562d336e&biw=1440&bih=799).
Rendered PCB allows to see how packages are placed on the PCB. It is a way to eliminate design bugs before manufacturing.

## Interference reduction ##
Article here [Powering\_uC](http://www.forbot.pl/forum/topics20/dla-poczatkujacych-zaklocenia-w-pracy-mikrokontrolerow-poradnik-praktyczny-dla-robotykow-vt6913.htm)