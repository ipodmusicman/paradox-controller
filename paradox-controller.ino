/*
 * Paradox Controller
 * 
 * This is a bridge between Paradox alarm systems and MQTT
 * 
 * Note that I cannot take credit for the full development of this sketch.  See https://github.com/maragelis/ParadoxRs232toMqtt
 * Modified for own use.
 * 
 * Still to do:
 * 
 * Periodically get the status from the alarm panel to maintain alarm state is correct.
 * Clean-ups and refactoring
 */
#include <FS.h>   
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

#define mqttServerAddress "127.0.0.1"
#define mqttServerPort "1883"
#define serverHostname "Paradox"

#define PROPERTY_MQTT_SERVER_ADDRESS "mqtt_server_address"
#define PROPERTY_MQTT_SERVER_PORT "mqtt_server_port"

const byte COMMAND_START_COMMUNICATION = 0x5f;
const byte COMMAND_INITIALISE_COMMUNICATION = 0x00;
const byte COMMAND_INITIALISE_COMMUNICATION_SUCCESSFUL = 0x10;
const byte COMMAND_SET_TIME_AND_DATE = 0x30;
const byte COMMAND_PERFORM_ACTION = 0x40;
const byte COMMAND_PANEL_STATUS = 0x50;
const byte COMMAND_WRITE_DATA_TO_EEPROM = 0x60;
const byte COMMAND_DISCONNECT = 0x70;
const byte COMMAND_COMMUNICATION_ERROR = 0x70;
const byte COMMAND_SAVE_WINLOAD_EVENT_POINTER = 0x80;
const byte COMMAND_SPECIAL_ACTION = 0x90;
const byte COMMAND_LIVE_EVENT = 0xe0;

const byte SOURCE_ID_WINLOAD_DIRECT = 0x01;
const byte SOURCE_ID_WINLOAD_THROUGH_IP_MODULE = 0x02;
const byte SOURCE_ID_WINLOAD_THROUGH_GSM_MODULE = 0x03;
const byte SOURCE_ID_WINLOAD_THROUGH_MODEM = 0x04;
const byte SOURCE_ID_NEWARE_DIRECT = 0x05;
const byte SOURCE_ID_NEWARE_THROUGH_IP_MODULE = 0x06;
const byte SOURCE_ID_NEWARE_THROUGH_GSM_MODULE = 0x07;
const byte SOURCE_ID_NEWARE_THROUGH_MODEM = 0x08;
const byte SOURCE_ID_IP_MODULE_WEB_PAGE = 0x09;
const byte SOURCE_ID_VMPS_DIRECT = 0x0a;
const byte SOURCE_ID_VMPS_THROUGH_GSM_MODULE = 0x0b;
const byte SOURCE_ID_REMOTE = 0x0c;

const byte STATUS_REQUEST_0_SYSTEM = 0x00;
const byte STATUS_REQUEST_1_SYSTEM = 0x01;
const byte STATUS_REQUEST_2_ZONE = 0x02;
const byte STATUS_REQUEST_3_ZONE_SCALED_SIGNAL_STRENGTH = 0x03;
const byte STATUS_REQUEST_4_PGM_REPEATER_RF_KEYPAD_SIGNAL_STRENGTH = 0x04;
const byte STATUS_REQUEST_5_ZONE_EXIT_DELAY = 0x05;

const byte EVENT_GROUP_ZONE_CLOSED = 0;
const byte EVENT_GROUP_ZONE_OPEN = 1;
const byte EVENT_GROUP_PARTITION_STATUS = 2;
const byte EVENT_GROUP_BELL_STATUS = 3;
const byte EVENT_GROUP_NON_REPORTABLE_EVENT = 6;
const byte EVENT_GROUP_BUTTON_PRESSED_ON_REMOTE_B = 8;
const byte EVENT_GROUP_BUTTON_PRESSED_ON_REMOTE_C = 9;
const byte EVENT_GROUP_BUTTON_PRESSED_ON_REMOTE_D = 10;
const byte EVENT_GROUP_BUTTON_PRESSED_ON_REMOTE_E = 11;
const byte EVENT_GROUP_COLD_START_WIRELESS_ZONE = 12;
const byte EVENT_GROUP_COLD_START_WIRELESS_MODULE = 13;
const byte EVENT_GROUP_BYPASS_PROGRAMMING = 14;
const byte EVENT_GROUP_USER_CODE_ACTIVATED_OUTPUT = 15;
const byte EVENT_GROUP_WIRELESS_SMOKE_MAINENANCE_SIGNAL = 16;
const byte EVENT_GROUP_DELAY_ZONE_TRANSMISSION = 17;
const byte EVENT_GROUP_ZONE_SIGNAL_STRENGTH_WEAK_1 = 18;
const byte EVENT_GROUP_ZONE_SIGNAL_STRENGTH_WEAK_2 = 19;
const byte EVENT_GROUP_ZONE_SIGNAL_STRENGTH_WEAK_3 = 20;
const byte EVENT_GROUP_ZONE_SIGNAL_STRENGTH_WEAK_4 = 21;
const byte EVENT_GROUP_BUTTON_PRESSED_ON_REMOTE_OP_5 = 22;
const byte EVENT_GROUP_BUTTON_PRESSED_ON_REMOTE_OP_6 = 23;
const byte EVENT_GROUP_FIRE_DELAY_STARTED = 24;
const byte EVENT_GROUP_SOFTWARE_ACCESS_1 = 25;
const byte EVENT_GROUP_SOFTWARE_ACCESS_2 = 26;
const byte EVENT_GROUP_BUS_MODULE_EVENT = 27;
const byte EVENT_GROUP_STAYD_PASS_ACKNOWLEDGED = 28;
const byte EVENT_GROUP_ARMING_WITH_USER = 29;
const byte EVENT_GROUP_SPECIAL_ARMING = 30;
const byte EVENT_GROUP_DISARMING_WITH_USER = 31;
const byte EVENT_GROUP_DISARMING_AFTER_AN_ALARM_WITH_USER = 32;
const byte EVENT_GROUP_ALARM_CANCELLED_WITH_USER = 33;
const byte EVENT_GROUP_SPECIAL_DISARMING = 34;
const byte EVENT_GROUP_ZONE_BYPASSED = 35;
const byte EVENT_GROUP_ZOME_IN_ALARM = 36;
const byte EVENT_GROUP_FIRE_ALARM = 37;
const byte EVENT_GROUP_ZONE_ALARM_RESTORE = 38;
const byte EVENT_GROUP_FIRE_ALARM_RESTORE = 39;
const byte EVENT_GROUP_SPECIAL_ALARM = 40;
const byte EVENT_GROUP_ZONE_SHUTDOWN = 41;
const byte EVENT_GROUP_ZONE_TAMPERED = 42;
const byte EVENT_GROUP_ZOME_TAMPER_RESTORE = 43;
const byte EVENT_GROUP_NEW_TROUBLE = 44;
const byte EVENT_GROUP_TROUBLE_RESTORED = 45;
const byte EVENT_GROUP_BUS_EBUS_WIRELESS_MODULE_TROUBLE = 46;
const byte EVENT_GROUP_BUS_EBUS_WIRELESS_MODULE_TROUBLE_RESTORED = 47;
const byte EVENT_GROUP_SPECIAL = 48;
const byte EVENT_GROUP_LOW_BATTERY_ON_ZONE = 49;
const byte EVENT_GROUP_LOW_BATTERY_ON_ZONE_RESTORE = 50;
const byte EVENT_GROUP_ZONE_SUPERVISION_TROUBLE = 51;
const byte EVENT_GROUP_ZONE_SUPERVISION_RESTORE = 52;
const byte EVENT_GROUP_WIRELESS_MODULE_SUPERVISION_TROUBLE = 53;
const byte EVENT_GROUP_WIRELESS_MODULE_SUPERVISION_RESTORE = 54;
const byte EVENT_GROUP_WIRELESS_MODULE_TAMPER_TROUBLE = 55;
const byte EVENT_GROUP_WIRELESS_MODULE_TAMPER_RESTORE = 56;
const byte EVENT_GROUP_NON_MEDICAL_ALARM = 57;
const byte EVENT_GROUP_ZONE_FORCED = 58;
const byte EVENT_GROUP_ZONE_INCLUDED = 59;
const byte EVENT_GROUP_SYSTEM_STATUS = 64;

const byte PARTITION_STATUS_SILENT_ALARM = 2;
const byte PARTITION_STATUS_BUZZER_ALARM = 3;
const byte PARTITION_STATUS_STEADY_ALARM = 4;
const byte PARTITION_STATUS_PULSED_ALARM = 5;
const byte PARTITION_STATUS_STROBE = 6;
const byte PARTITION_STATUS_ALARM_STOPPED = 7;
const byte PARTITION_STATUS_SQUAWK_ON = 8;
const byte PARTITION_STATUS_SQUAWK_OFF = 9;
const byte PARTITION_STATUS_GROUND_START = 10;
const byte PARTITION_STATUS_DISARM_PARTITION = 11;
const byte PARTITION_STATUS_ARM_PARTITION = 12;
const byte PARTITION_STATUS_ENTRY_DELAY_STARTED = 13;
const byte PARTITION_STATUS_EXIT_DELAY_STARTED = 14;
const byte PARTITION_STATUS_PRE_ALARM_DELAY = 15;
const byte PARTITION_STATUS_REPORT_CONFIRMATION = 16;
const byte PARTITION_STATUS_ANY_PARTITION_STATUS_EVENT = 99;

const byte SPECIAL_ALARM_PANIC_NON_MEDICAL = 0;

const byte SPECIAL_SYSTEM_POWER_UP = 0;
const byte SPECIAL_REPORTING_TEST = 1;
const byte SPECIAL_SOFTWARE_LOG_ON = 2;
const byte SPECIAL_SOFTWARE_LOG_OFF = 3;
const byte SPECIAL_INSTALLER_IN_PROGRAMMING_MODE = 4;
const byte SPECIAL_INSTALLER_EXITED_PROGRAMMING_MODE = 5;
const byte SPECIAL_MAINTENANCE_IN_PROGRAMMING_MODE = 6;
const byte SPECIAL_MAINTENANCE_EXCITED_PROGRAMMING_MODE = 7;
const byte SPECIAL_CLOSING_DELINQUENCY_DELAY_ELAPSED = 8;
const byte SPECIAL_ANY_SPECIAL_EVENT = 99;

const byte ACTION_STAY_ARM = 0x01;
const byte ACTION_STAY_ARM_2 = 0x02;
const byte ACTION_SLEEP_ARM = 0x03;
const byte ACTION_FULL_ARM = 0x04;
const byte ACTION_DISARM = 0x05;
const byte ACTION_STAY_ARM_WITH_STAY_D_ENABLING = 0x06;
const byte ACTION_SLEEP_ARM_WITH_STAY_D_ENABLING = 0x07;
const byte ACTION_DISARM_BOTH_PARTITIONS_WITH_STAY_D_DISABLING = 0x08;
const byte ACTION_BYPASS = 0x10;
const byte ACTION_BEEP = 0x20;
const byte ACTION_PGM_ON_OVERRIDE_MODE = 0x30;
const byte ACTION_PGM_OFF_OVERRIDE_MODE = 0x31;
const byte ACTION_PGM_ON = 0x32;
const byte ACTION_PGM_OFF = 0x33;
const byte ACTION_RELOAD_PANEL_RAM_WITH_EEPROM = 0x80;
const byte ACTION_PERFORM_A_BUS_SCAN_MODULE = 0x85;

const int FIXED_MESSAGE_SIZE = 37;

const String ALARM_STATUS_ARMED_HOME = "armed_home";
const String ALARM_STATUS_ARMED_AWAY = "armed_away";
const String ALARM_STATUS_DISARMED = "disarmed";
const String ALARM_STATUS_ALARM_TRIGGERED = "triggered";
const String ALARM_STATUS_ALARM_STOPPED = "stopped";

const String mqttTopicEvent = "paradox/event";
const String mqttTopicStatus = "paradox/status";
const String mqttTopicAction1 = "paradox/action/1";
const String mqttTopicAction2 = "paradox/action/2";
const String mqttTopicEventZone = "paradox/event/zone/";
const String mqttTopicTriggerZone = "paradox/event/trigger_zone";
const String mqttTopicAlarmStatus = "paradox/alarm_status/";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

bool shouldSaveConfig = false;
bool panelInitialised = false;
bool pannelConnected = false;

long lastReconnectAttempt = 0;

char inData[38]; // Allocate some space for the string
byte pindex = 0; // Index into array; where to store the character

struct inPayload {
  byte pcPasswordFirst2Digits;
  byte pcPasswordSecond2Digits;
  byte Command;
  byte Subcommand;
 };
 
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  blink(100);
  delay(1000);

  Serial.begin(9600);
  Serial.flush();
  trc("Paradox Serial Interface is up");

  blink(1000);
  flushSerialBuffer();
  readConfig();
//  wifi_station_set_hostname(serverHostname);
  setupWiFi();
  ArduinoOTA.setHostname("ParadoxController");
  ArduinoOTA.begin();
  trc("Finnished wifi setup");
  blink(100);
  blink(100);
  blink(100);
  delay(1500);
  lastReconnectAttempt = 0;
}

void loop() {
  readSerial();
  if ((inData[0] & 0xF0) != COMMAND_LIVE_EVENT){ // re-align serial buffer
    blink(200);
    flushSerialBuffer();
  }
}

void sendJsonString (byte armstatus, byte event, byte sub_event, byte partition, String label) {
  String retval = "{ \"armstatus\":" + String(armstatus) + ", \"event\":" + String(event) + ", \"sub_event\":" + String(sub_event) + ", , \"partition\":" + String(partition) + ", \"label\":\"" + String(label) + "\"}";

  if (event == EVENT_GROUP_PARTITION_STATUS) {
    if (sub_event == PARTITION_STATUS_ARM_PARTITION) {
      sendMQTT(mqttTopicAlarmStatus + String(partition + 1), ALARM_STATUS_ARMED_AWAY, true);
      return;
    } else if (sub_event == PARTITION_STATUS_DISARM_PARTITION) {
      sendMQTT(mqttTopicTriggerZone, "0", true); 
      sendMQTT(mqttTopicAlarmStatus + String(partition + 1), ALARM_STATUS_DISARMED, true); 
      return;
    } else if (sub_event == PARTITION_STATUS_ALARM_STOPPED) {
      sendMQTT(mqttTopicAlarmStatus + String(partition + 1), ALARM_STATUS_ALARM_STOPPED, true); 
      return;
    }
  } else if (event == EVENT_GROUP_SPECIAL_ALARM) {
    if (sub_event == SPECIAL_ALARM_PANIC_NON_MEDICAL) {
      sendMQTT(mqttTopicAlarmStatus + String(partition + 1), ALARM_STATUS_ALARM_TRIGGERED, true); 
      return;
    }
  } else if (event == EVENT_GROUP_ZOME_IN_ALARM) {
    sendMQTT(mqttTopicEventZone + String(sub_event), "1", true); 
    sendMQTT(mqttTopicTriggerZone, String(sub_event), true); 
    sendMQTT(mqttTopicAlarmStatus + String(partition + 1), ALARM_STATUS_ALARM_TRIGGERED, true); 
    return;
  } else if (event == EVENT_GROUP_ZONE_ALARM_RESTORE) {
    sendMQTT(mqttTopicEventZone + String(sub_event), "0", true); 
    return;
  } else if (event == EVENT_GROUP_ZONE_CLOSED || event == EVENT_GROUP_ZONE_OPEN) {
    sendMQTT(mqttTopicEventZone + String(sub_event), String(event), true); 
    return;
  }
  sendMQTT(mqttTopicEvent, retval); 
}

void sendMQTT(String topicNameSend, String dataStr) {
  sendMQTT(topicNameSend, dataStr, false); 
}


void sendMQTT(String topicNameSend, String dataStr, boolean retained) {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      trc("MQTT not connected.  Attemping to reconnect ...");
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();
  }
  char topicStrSend[26];
  topicNameSend.toCharArray(topicStrSend,26);
  char dataStrSend[200];
  dataStr.toCharArray(dataStrSend,200);
  if (!client.publish(topicStrSend, dataStrSend, retained)) {
    trc("Message not published");
  }
}

void readSerial() {
  while (Serial.available() < FIXED_MESSAGE_SIZE) { 
    ArduinoOTA.handle();
    client.loop();
  }                            

  pindex = 0;

  while (pindex < FIXED_MESSAGE_SIZE) {  // Paradox packet is 37 bytes 
    inData[pindex++] = Serial.read();            
  }

  inData[++pindex] = 0x00; // Make it print-friendly

  if ((inData[0] & 0xF0) == COMMAND_LIVE_EVENT) {
    byte armstatus = inData[0];
    byte event = inData[7];
    byte sub_event = inData[8];
    byte partition = inData[9];
    String label = String(inData[15]) + String(inData[16]) + String(inData[17]) + String(inData[18]) + String(inData[19]) + String(inData[20]) + String(inData[21]) + String(inData[22]) + String(inData[23]) + String(inData[24]) + String(inData[25]) + String(inData[26]) + String(inData[27]) + String(inData[28]) + String(inData[29]) + String(inData[30]);
    label.trim();
    sendJsonString(armstatus, event, sub_event, partition, label);
    if (event == EVENT_GROUP_SPECIAL && sub_event == SPECIAL_SOFTWARE_LOG_OFF) {
      pannelConnected = false;
    } else if (event == EVENT_GROUP_SPECIAL && sub_event == SPECIAL_SOFTWARE_LOG_ON && !pannelConnected) {
      pannelConnected = true;
    }
  } else if (inData[0] == COMMAND_INITIALISE_COMMUNICATION_SUCCESSFUL) {
    panelInitialised = true;
  }
}

void saveConfigCallback () {
  shouldSaveConfig = true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  // In order to republish this payload, a copy must be made
  // as the orignal payload buffer will be overwritten whilst
  // constructing the PUBLISH packet.
  trc("Hey I got a callback ");
  // Conversion to a printable string
  payload[length] = '\0';

  String strTopic(topic);

  int partitionZoneIndex = strTopic.lastIndexOf("/");
  String partitionZone;

  if (partitionZoneIndex != -1) {
    partitionZone = strTopic.substring(partitionZoneIndex + 1);
  }

  inPayload data = decodePayload(partitionZone, String ((char*)payload));

  panelInitialised = false;

  doLogin(data.pcPasswordFirst2Digits, data.pcPasswordSecond2Digits);

  if (!panelInitialised) {
    return;
  }

  int cnt = 0;
  while (!pannelConnected && cnt < 10) {
    readSerial();
    cnt++;    
  }

  if (!pannelConnected) {
    trc("Problem connecting to panel");
  } else if (data.Command != 0x00 && pannelConnected )  {
    controlPanel(data);
  } else {
    trc("Bad Command ");
  }
}

struct inPayload decodePayload(String partitionZone, String payload) {
  char charpass1[4];
  char charpass2[4];
  char charsubcommand[4];
  
  String password = "0000";

  String pass1 = password.substring(0, 2);
  String pass2 = password.substring(2, 4);

  pass1.toCharArray(charpass1, 4);
  pass2.toCharArray(charpass2, 4);
  partitionZone.toCharArray(charsubcommand,4);

  unsigned long number1 = strtoul(charpass1, nullptr, 16);
  unsigned long number2 = strtoul(charpass2, nullptr, 16);
  unsigned long number3 = strtoul(charsubcommand, nullptr, 16);

  number3--;

  byte panelPassword1 = number1 & 0xFF; 
  byte panelPassword2 = number2 & 0xFF; 
  byte commandB = getPanelCommand(payload);
  byte subCommand = number3 & 0xFF;
  
  inPayload data1 = {panelPassword1, panelPassword2, commandB, subCommand};

  return data1;
}

byte getPanelCommand(String data){
  byte retval = 0x00;
  data.toLowerCase();
  if (data == "arm_home" || data == "stay" || data == "0") {
    retval = ACTION_STAY_ARM;
  } else if (data == "arm_away" || data == "1") {    
    retval = ACTION_FULL_ARM;
  } else if (data == "arm_night" || data == "2") {
    retval = ACTION_SLEEP_ARM;
  } else if (data == "disarm" || data == "3") {
    retval = ACTION_DISARM;
  } else if (data == "bypass" || data == "10") {
    retval = ACTION_BYPASS;
  } else if (data == "setdate") {
    panelSetDate();
  } else if (data == "disconnect" || data == "99") {
    panelDisconnect();
  }

  return retval;
}

void panelSetDate(){
  byte data[FIXED_MESSAGE_SIZE] = {};
  byte checksum;
  for (int x = 0; x < FIXED_MESSAGE_SIZE; x++) {
    data[x] = 0x00;
  }

  data[0] = COMMAND_SET_TIME_AND_DATE;
  data[4] = 0x21;
  data[5] = 0x18;
  data[6] = 0x02;
  data[7] = 0x06;
  data[8] = 0x01;
  data[9] = 0x22;
  data[33] = 0x01;

  checksum = 0;
  for (int x = 0; x < FIXED_MESSAGE_SIZE - 1; x++) {
    checksum += data[x];
  }

  while (checksum > 255) {
    checksum = checksum - (checksum / 256) * 256;
  }

  data[36] = checksum & 0xFF;

  Serial.write(data, FIXED_MESSAGE_SIZE);
}

void controlPanel(inPayload data){
  byte armdata[FIXED_MESSAGE_SIZE] = {};
  byte checksum;
  for (int x = 0; x < FIXED_MESSAGE_SIZE; x++) {
    armdata[x] = 0x00;
  }

  armdata[0] = COMMAND_PERFORM_ACTION;
  armdata[2] = data.Command;
  armdata[3] = data.Subcommand;;
  armdata[33] = 0x01;
  armdata[34] = 0x00;
  armdata[35] = 0x00;
  checksum = 0;
  
  for (int x = 0; x < FIXED_MESSAGE_SIZE - 1; x++) {
    checksum += armdata[x];
  }

  while (checksum > 255) {
    checksum = checksum - (checksum / 256) * 256;
  }

  armdata[36] = checksum & 0xFF;

  while (Serial.available() > FIXED_MESSAGE_SIZE)  {
    trc("serial cleanup");
    readSerial();
  }

  trc("sending Data");
  Serial.write(armdata, FIXED_MESSAGE_SIZE);
  readSerial();

  if ( inData[0] >= 40 && inData[0] <= 45) {
    trc(" Command success ");
  }
}

void panelDisconnect() {
  byte data[FIXED_MESSAGE_SIZE] = {};
  byte checksum;
  for (int x = 0; x < FIXED_MESSAGE_SIZE; x++) {
    data[x] = 0x00;
  }

  data[0] = COMMAND_DISCONNECT;
  data[2] = 0x05; // Validation byte
  data[33] = 0x01;

  checksum = 0;
  for (int x = 0; x < FIXED_MESSAGE_SIZE - 1; x++) {
    checksum += data[x];
  }

  while (checksum > 255) {
    checksum = checksum - (checksum / 256) * 256;
  }

  data[36] = checksum & 0xFF;

  Serial.write(data, FIXED_MESSAGE_SIZE);
  readSerial();
}

void doLogin(byte pass1, byte pass2) {
  
  byte data[FIXED_MESSAGE_SIZE] = {};
  byte data1[FIXED_MESSAGE_SIZE] = {};
  byte checksum;

  for (int x = 0; x < FIXED_MESSAGE_SIZE; x++) {
    data[x] = 0x00;
    data1[x] = 0x00;
  }

  flushSerialBuffer();
  data[0] = COMMAND_START_COMMUNICATION;
  data[1] = 0x20; // Extra validation byte
  data[33] = SOURCE_ID_WINLOAD_DIRECT;
  data[34] = 0x00; // User ID high byte
  data[35] = 0x00; // User ID low byte

  checksum = 0;
  for (int x = 0; x < FIXED_MESSAGE_SIZE - 1; x++) {
    checksum += data[x];
  }

  while (checksum > 255) {
    checksum = checksum - (checksum / 256) * 256;
  }

  data[36] = checksum & 0xFF;

  Serial.write(data, FIXED_MESSAGE_SIZE);
  readSerial();

  data1[0] = COMMAND_INITIALISE_COMMUNICATION;
  data1[4] = inData[4]; // Panel Product ID
  data1[5] = inData[5]; // Panel Firmware Version
  data1[6] = inData[6]; // Panel Firmware Revision
  data1[7] = inData[7]; // Panel Firmware Build
  data1[8] = inData[8]; // Programmmed Panel ID Digit 1 & 2 (Not required for NEware)
  data1[9] = inData[9]; // Programmmed Panel ID Digit 3 & 4 (Not required for NEware)
  data1[10] = pass1; // Panel PC Password Digit 1 & 2 (Not required for NEware)
  data1[11] = pass2; // Panel PC Password Digit 3 & 4 (Not required for NEware)
  data1[13] = 0x00; // Source Mode (old method) 0x00 Winload, 0x55 NEware
  data1[33] = SOURCE_ID_WINLOAD_DIRECT;

  checksum = 0;
  for (int x = 0; x < FIXED_MESSAGE_SIZE - 1; x++) {
    checksum += data1[x];
  }

  while (checksum > 255) {
    checksum = checksum - (checksum / 256) * 256;
  }

  data1[36] = checksum & 0xFF;

  Serial.write(data1, FIXED_MESSAGE_SIZE);
  readSerial();
}

void flushSerialBuffer(){
  while (Serial.read() >= 0) {
    ArduinoOTA.handle();
    client.loop();
  }
}

void setupWiFi(){
  WiFiManagerParameter mqttServer(PROPERTY_MQTT_SERVER_ADDRESS, "MQTT Address", mqttServerAddress, 40);
  WiFiManagerParameter mqttPort(PROPERTY_MQTT_SERVER_PORT, "MQTT Port", mqttServerPort, 6);

  WiFiManager wifiManager;
  
  if (mqttServerAddress == "" || mqttServerPort == "") {
    trc("Resetting wifiManager");
    WiFi.disconnect();
    wifiManager.resetSettings();
    ESP.reset();
    delay(1000);
  }

  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setConfigPortalTimeout(180);
  
  wifiManager.addParameter(&mqttServer);
  wifiManager.addParameter(&mqttPort);
  
  if (!wifiManager.autoConnect("ParadoxController", "")) {
    trc("Failed to initialise onboard access point");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  trc("WiFi Connected");

  strcpy(mqttServerAddress, mqttServer.getValue());
  strcpy(mqttServerPort, mqttPort.getValue());
  
  if (shouldSaveConfig) {
    trc("Saving configuration");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json[PROPERTY_MQTT_SERVER_ADDRESS] = mqttServerAddress;
    json[PROPERTY_MQTT_SERVER_PORT] = mqttServerPort;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      trc("Failed to open configuration file");
    }

    json.printTo(configFile);
    configFile.close();
  }

  trc("Setting MQTT Server connection");
  unsigned int mqtt_port_x = atoi (mqttServerPort); 
  client.setServer(mqttServerAddress, mqtt_port_x);
  client.setCallback(callback);
  reconnect();
}

boolean reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    trc("Attempting to connect to MQTT Server");
    String mqname =  WiFi.macAddress();
    char charBuf[50];
    mqname.toCharArray(charBuf, 50) ;

    char topicStatus[50];
    mqttTopicStatus.toCharArray(topicStatus, 50);
    blink(100);
    if (client.connect(charBuf, topicStatus, 0, true, "offline")) {
      trc("Connected to MQTT Server");
      blink(50);
      blink(50);
      blink(50);
      sendMQTT(mqttTopicStatus,"online", true);
      //Topic subscribed so as to get data
      subscribing(mqttTopicAction1);
      subscribing(mqttTopicAction2);
    } else {
      trc("Failed to connect to MQTT Server");
      trc(String(client.state()));
      trc("Trying again in 5 seconds ...");
      delay(5000);
    }
  }
  return client.connected();
}

void subscribing(String topicNameRec) { // MQTT subscribing to topic
  char topicStrRec[26];
  topicNameRec.toCharArray(topicStrRec,26);
  // subscription to topic for receiving data
  boolean pubresult = client.subscribe(topicStrRec);
  if (pubresult) {
    trc("Subscribed to ");
    trc(topicNameRec);
  }
}

void readConfig() {
  if (SPIFFS.begin()) {
    trc("Mounted file system");
    if (SPIFFS.exists("/config.json")) {
      trc("Reading configuration file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        trc("Opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (json.success()) {
          strcpy(mqttServerAddress, json[PROPERTY_MQTT_SERVER_ADDRESS]);
          strcpy(mqttServerPort, json[PROPERTY_MQTT_SERVER_PORT]);
        } else {
          trc("Failed to read configuration");
        }
      }
    } else {
      trc("File /config.json doesn't exist");
    }
  } else {
    trc("Failed to mount FS");
  }
}

void trc(String msg){
  //Serial.println(msg);
}

void blink(int duration) {
  digitalWrite(LED_BUILTIN,LOW);
  delay(duration);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(duration);
}

