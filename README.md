## Update

I've decided to use PAI (https://github.com/ParadoxAlarmInterface/pai) instead of this sketch for my Paradox integration with Home Assistant.  PAI is super feature rich and a lot more stable.  As such, this project is no longer going to be actively maintained.

# Paradox Controller

This ESP8266 sketch acts as a bridge between Paradox alarm systems and platforms supporting MQTT. To coin the phase "Making your dumpb alarm system, a smart alarm system".  :)  This controller works with various Paradox alarm systems that support the serial port control protocol.  This was tested on a Paradox SP6000.  The MQTT interface is compatbile with MQTT Alarm Panel as defined within Home Assistant.  This means that you can configure the MQTT alarm panel within Home Assistant and configure the required MQTT topics in order to arm/disarm your alarm as well as get alarted when the alarm has been triggered.  This sketch supports the management of up to two alarm partitions.

The controller supports the following features:

* Arming (Stay, Night, Away) and disarming of alarm partitions
* Zone bypassing
* Sending of zone status events
* Sending of alarm partition status updates
* An event is sent when the alarm is triggered due to a zone trigger or panic button press.  A separate event is sent indicating which zone triggered the alarm or if the alarm was triggered via a panic.
* Sending of alarm partition status if the alarm siren stops.  This happens after the alarm was triggered, but the actual siren stops sounding.  Note that the alarm is still considered armed.
* Compatible with MQTT Alarm Panale within Home Assistant

Outstanding:

* Zone bypassed indicator
* Periodic zone status updates

## Getting Started

### Prerequisites

You'll need the following:

* An ESP8266 board like a NodeMCU or D1 Mini/D1 Mini Pro.  These are the two boards that this was developed and tested on.
* 3 jumper wires so that the ESP8266 board can be connected to the Paradox alarm system main board serial port.  Must have female connectors on one side which connects to the Paradox alarm system main board serial port.
* Arduino IDE to load the sketch onto the board
* Whatever your platform of choice is that supports MQTT.  I recommend Home Assistant using a MQTT broker like Mosquito.  There are many tutorials on how to get Home Assistant and the Mosquito broker up and running.

### Installing

* Fire up the Arduino IDE
* Load the sketch
* Connect your ESP8266 based board to your computer via the USB port
* Select the COM port under Tools | Port
* Hit the Upload button (second button from the left on the toolbar) and you should see the ESP8266 board LED blink rapidly as the sketch is being uploaded
* Once uploaded, the LED will blink
* If this is a new board which does not have any Wifi settings configured on yet, use your device like your mobile phone and connect to the Paradox Controller SSID.  
* Once connected, a browser should appear where you are able to configure initial settings.
* Click "Configure WiF"
* Enter your Wifi SSID and password
* Enter your MQTT broker address and port
* Click "Save"
* The board will reboot and attempt to connect to your WiFi network and MQTT broker
* Once all connects 100%, the LED will blink rapidly a few times.
* If there is an issue, open the Serial Monitor within the Arduino IDE and reboot the board to assess.  If you wish to enable debug logging, you need to find the trace method in the sketch and comment out Serial.println.  But please comment it out once done.

### Connecting to Paramdox alarm main board

Locate the serial port on the main board.  On the SP6000, there is a 4 PIN connector on the top right of the board, but each model of the board can be different.  The pinouts are

* Rx
* Tx
* Ground
* Aux+

Connect the corresponding pins to your ESP8266 board.  You might find that you'd need to swop the Rx/Tx jumper wires around depending on your set up.

### MQTT Topics

The controller sends data on the following topics:

* paradox/event - All other events will be posted to this topic in JSON format
* paradox/status - This will either contain the value of "online" or "offline depending of the controller is online or offline
* paradox/event/zone/x - x represents the zone number.  This will send 1 = zone open or 0 = zone closed.  The zone number can be from 1 to 32.
* paradox/event/trigger_zone - Either the zone number or the word "panic" will appear in this topic when the alarm is triggered
* paradox/alarm_status/x - x represents the partition number which can be either 1 or 2.  This will send the following:
*   armed_away - The partition is armed in away mode
*   triggered - The partition alarm is triggered
*   disarmed - The partition is disarmed
*   stopped - The siren is stopped.  This generally happens after a configurable amount of time set on your alarm system

The controller can receive commands on the following topics:

* paradox/action/x - This can be used to arm / disarm / bypass.  For arm/disarm, the x indicates the partition number (1 or 2) while for bypassing a zone, x indicates the zone number to bypass (1 - 32). 

## To configure in Home Assistant

Edit your configuration.yaml file and add the following entries.  Note that my alarm has two partitions so two alarm panels are added.
```
alarm_control_panel:
 - platform: mqtt
   name: paradox_alarm_partition_1
   unique_id: paradox_alarm_partition_1
   state_topic: "paradox/alarm_status/1"
   command_topic: "paradox/action/1"
   availability_topic:  "paradox/status"
 - platform: mqtt
   name: paradox_alarm_partition_2
   unique_id: paradox_alarm_partition_2
   state_topic: "paradox/alarm_status/2"
   command_topic: "paradox/action/2"
   availability_topic:  "paradox/status"
```
In order to add the zones as binary sensors, duplicate each zone into your configuration.yaml as below, but changing the state_topic from zone/1 to zone/2, zone/3 etc for each zone.  Give it a good name for each zone and a device class.

eg:
```
binary_sensor:
  - platform: mqtt
    name: Lounge Window
    state_topic: "paradox/event/zone/1"
    device_class: window
    payload_on: 1
    payload_off: 0
```
## Acknowledgments

* See https://github.com/maragelis/ParadoxRs232toMqtt.  He developed the original sketch and I decided to modify it for my own use.  His development is also ongoing so check out his sketch as well.
