/*
   Paradox Controller

   This is a bridge between Paradox alarm systems and MQTT, but also implementing MQTT alarm

   Note that I cannot take credit for the full development of this sketch.  See https://github.com/maragelis/ParadoxRs232toMqtt
   Modified and refactored.

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
const byte COMMAND_PANEL_STATUS_1 = 0x50;
const byte COMMAND_WRITE_DATA_TO_EEPROM = 0x60;
const byte COMMAND_CLOSE_CONNECTION = 0x70;
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

char responseMessage[38]; // Allocate some space for the response message

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.mode(WIFI_STA);
  blink(100);
  delay(1000);

  Serial.begin(9600);
  Serial.flush();
  trc("Paradox Serial Interface is up");

  blink(1000);
  flushSerialBuffer();
  readConfig();
  setupWiFi();
  ArduinoOTA.setHostname("ParadoxController");
  ArduinoOTA.begin();
  trc("Finished wifi setup");
  blink(100);
  blink(100);
  blink(100);
  delay(1500);
  lastReconnectAttempt = 0;
}

void setupWiFi() {
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
  unsigned int mqttPortInt = atoi (mqttServerPort);
  client.setServer(mqttServerAddress, mqttPortInt);
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
      sendMQTT(mqttTopicStatus, "online", true);
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
  topicNameRec.toCharArray(topicStrRec, 26);
  // subscription to topic for receiving data
  boolean pubresult = client.subscribe(topicStrRec);
  if (pubresult) {
    trc("Subscribed to " + topicNameRec);
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

void loop() {
  while (Serial.available() < FIXED_MESSAGE_SIZE) {
    ArduinoOTA.handle();
    handleMqttKeepAlive();
  }

  readMessageFromSerial();
  processEventMessage();

  if ((responseMessage[0] & 0xF0) != COMMAND_LIVE_EVENT &&
     (responseMessage[0] & 0xF0) != COMMAND_PERFORM_ACTION &&
     (responseMessage[0] & 0xF0) != COMMAND_PANEL_STATUS_1 &&
     (responseMessage[0] & 0xF0) != COMMAND_CLOSE_CONNECTION) { 
    flushSerialBuffer();
  }
}

void saveConfigCallback () {
  shouldSaveConfig = true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';

  String strTopic(topic);

  int partitionZoneIndex = strTopic.lastIndexOf("/");
  String partitionZone;

  if (partitionZoneIndex != -1) {
    partitionZone = strTopic.substring(partitionZoneIndex + 1);
  }

  sendCommandToPanel (partitionZone, String ((char*)payload));
}

void sendCommandToPanel (String partitionZone, String command) {
  if (!panelInitialised || !pannelConnected) {
    panelInitialised = false;
    pannelConnected = false;
    doLogin("0000");
  }

  if (!panelInitialised) {
    return;
  }

  int cnt = 0;
  while (!pannelConnected && cnt < 10) {
    readMessageFromSerial();
    processEventMessage();
    cnt++;
  }

  if (pannelConnected) {
    sendRequestToPanel(partitionZone, command);
  } else {
    trc("Unable to connect to panel");
  }
}

void doLogin(String password) {
  trc("Initialising panel");
  char charpass1[4];
  char charpass2[4];

  String pass1 = password.substring(0, 2);
  String pass2 = password.substring(2, 4);

  pass1.toCharArray(charpass1, 4);
  pass2.toCharArray(charpass2, 4);

  unsigned long number1 = strtoul(charpass1, nullptr, 16);
  unsigned long number2 = strtoul(charpass2, nullptr, 16);

  byte panelPassword1 = number1 & 0xFF;
  byte panelPassword2 = number2 & 0xFF;
  
  byte startCommunicationRequest[FIXED_MESSAGE_SIZE] = {};
  byte initialiseCommunicationRequest[FIXED_MESSAGE_SIZE] = {};
  byte checksum;

  for (int x = 0; x < FIXED_MESSAGE_SIZE; x++) {
    startCommunicationRequest[x] = 0x00;
    initialiseCommunicationRequest[x] = 0x00;
  }

  flushSerialBuffer();
  startCommunicationRequest[0] = COMMAND_START_COMMUNICATION;
  startCommunicationRequest[1] = 0x20; // Extra validation byte
  startCommunicationRequest[33] = SOURCE_ID_WINLOAD_DIRECT; // Source ID (See table 1)
  startCommunicationRequest[34] = 0x00; // User ID high byte
  startCommunicationRequest[35] = 0x00; // User ID low byte

  checksum = 0;
  for (int x = 0; x < FIXED_MESSAGE_SIZE - 1; x++) {
    checksum += startCommunicationRequest[x];
  }

  while (checksum > 255) {
    checksum = checksum - (checksum / 256) * 256;
  }

  startCommunicationRequest[36] = checksum & 0xFF;

  Serial.write(startCommunicationRequest, FIXED_MESSAGE_SIZE);
  readMessageFromSerial();

  initialiseCommunicationRequest[0] = COMMAND_INITIALISE_COMMUNICATION;
  initialiseCommunicationRequest[4] = responseMessage[4]; // Panel Product ID
  initialiseCommunicationRequest[5] = responseMessage[5]; // Panel Firmware Version
  initialiseCommunicationRequest[6] = responseMessage[6]; // Panel Firmware Revision
  initialiseCommunicationRequest[7] = responseMessage[7]; // Panel Firmware Build
  initialiseCommunicationRequest[8] = responseMessage[8]; // Programmmed Panel ID Digit 1 & 2 (Not required for NEware)
  initialiseCommunicationRequest[9] = responseMessage[9]; // Programmmed Panel ID Digit 3 & 4 (Not required for NEware)
  initialiseCommunicationRequest[10] = panelPassword1; // Panel PC Password Digit 1 & 2 (Not required for NEware)
  initialiseCommunicationRequest[11] = panelPassword2; // Panel PC Password Digit 3 & 4 (Not required for NEware)
  initialiseCommunicationRequest[13] = 0x00; // Source Mode (old method) 0x00 Winload, 0x55 NEware
  initialiseCommunicationRequest[33] = SOURCE_ID_WINLOAD_DIRECT; // Source ID (See table 1)
  initialiseCommunicationRequest[34] = 0x00; // User ID high byte
  initialiseCommunicationRequest[35] = 0x00; // User ID low byte

  checksum = 0;
  for (int x = 0; x < FIXED_MESSAGE_SIZE - 1; x++) {
    checksum += initialiseCommunicationRequest[x];
  }

  while (checksum > 255) {
    checksum = checksum - (checksum / 256) * 256;
  }

  initialiseCommunicationRequest[36] = checksum & 0xFF;

  Serial.write(initialiseCommunicationRequest, FIXED_MESSAGE_SIZE);
  readMessageFromSerial();

  if (responseMessage[0] == COMMAND_INITIALISE_COMMUNICATION_SUCCESSFUL) {
    panelInitialised = true;
  }
}

void sendRequestToPanel(String partitionZone, String command) {
  char charPartitionZone[4];
  partitionZone.toCharArray(charPartitionZone, 4); // Convert the string to a char array
  unsigned long longPartitionZone = strtoul(charPartitionZone, nullptr, 16); // convert the char array to an unsigned long
  longPartitionZone--; // decrease by 1 as the partition or zone is zero based
  byte bytePartitionZone = longPartitionZone & 0xFF; // convert the unsigned long to a byte
  boolean actionSuccessful = true;
  command.toLowerCase();
  if (command == "arm_home" || command == "stay" || command == "0") {
    actionSuccessful = performAction (ACTION_STAY_ARM, bytePartitionZone);
  } else if (command == "arm_away" || command == "1") {
    actionSuccessful = performAction (ACTION_FULL_ARM, bytePartitionZone);
  } else if (command == "arm_night" || command == "2") {
    actionSuccessful = performAction (ACTION_SLEEP_ARM, bytePartitionZone);
  } else if (command == "disarm" || command == "3") {
    actionSuccessful = performAction (ACTION_DISARM, bytePartitionZone);
  } else if (command == "bypass" || command == "10") {
    actionSuccessful = performAction (ACTION_BYPASS, bytePartitionZone);
  } else if (command == "disconnect" || command == "99") {
    closeConnection();
  } else if (command == "status") {
    systemStatus1();
  }

  // This is performed so that if an action fails, perhaps due to the panel being in a different status, this will query the status and update the arm/disarm status accordingly.
  if (!actionSuccessful) {
    sendCommandToPanel("1", "status");
  }
}

boolean performAction(byte action, byte partitionZone) {
  while (Serial.available() > FIXED_MESSAGE_SIZE) {
    trc("Read any other data off the serial port like events, etc");
    readMessageFromSerial();
    processEventMessage();
  }
  
  byte performActionRequest[FIXED_MESSAGE_SIZE] = {};
  byte checksum;
  for (int x = 0; x < FIXED_MESSAGE_SIZE; x++) {
    performActionRequest[x] = 0x00;
  }

  performActionRequest[0] = COMMAND_PERFORM_ACTION;
  performActionRequest[2] = action; // Action
  performActionRequest[3] = partitionZone; // Partition or Zone number
  performActionRequest[33] = SOURCE_ID_WINLOAD_DIRECT;
  performActionRequest[34] = 0x00; // User ID high byte
  performActionRequest[35] = 0x00; // User ID low byte
  checksum = 0;

  for (int x = 0; x < FIXED_MESSAGE_SIZE - 1; x++) {
    checksum += performActionRequest[x];
  }

  while (checksum > 255) {
    checksum = checksum - (checksum / 256) * 256;
  }

  performActionRequest[36] = checksum & 0xFF;

  trc("Sending perform action request");
  Serial.write(performActionRequest, FIXED_MESSAGE_SIZE);
  readMessageFromSerial();

  if (responseMessage[0] >= 40 && responseMessage[0] <= 45) {
    trc("Perform action request successful");
    return true;
  }
  return false;
}

void systemStatus1() {
  byte systemStatusOneRequest[FIXED_MESSAGE_SIZE] = {};
  byte checksum;
  for (int x = 0; x < FIXED_MESSAGE_SIZE; x++) {
    systemStatusOneRequest[x] = 0x00;
  }

  systemStatusOneRequest[0] = COMMAND_PANEL_STATUS_1;
  systemStatusOneRequest[2] = 0x80; // Validation to distinguish from Eeprom read.
  systemStatusOneRequest[3] = STATUS_REQUEST_1_SYSTEM;
  systemStatusOneRequest[33] = SOURCE_ID_WINLOAD_DIRECT;
  systemStatusOneRequest[34] = 0x00; // User ID high byte
  systemStatusOneRequest[35] = 0x00; // User ID low byte

  checksum = 0;
  for (int x = 0; x < FIXED_MESSAGE_SIZE - 1; x++) {
    checksum += systemStatusOneRequest[x];
  }

  while (checksum > 255) {
    checksum = checksum - (checksum / 256) * 256;
  }

  systemStatusOneRequest[36] = checksum & 0xFF;

  Serial.write(systemStatusOneRequest, FIXED_MESSAGE_SIZE);
  readMessageFromSerial();

  if ((responseMessage[0] & 0xF0) == COMMAND_PANEL_STATUS_1) {
    int partition1Status = bitRead(responseMessage[17], 0);
    int partition2Status = bitRead(responseMessage[21], 0);
    if (partition1Status == 1) {
      sendMQTT(mqttTopicAlarmStatus + "1", ALARM_STATUS_ARMED_AWAY, true);
    } else {
      sendMQTT(mqttTopicAlarmStatus + "1", ALARM_STATUS_DISARMED, true);
    }
    if (partition2Status == 1) {
      sendMQTT(mqttTopicAlarmStatus + "2", ALARM_STATUS_ARMED_AWAY, true);
    } else {
      sendMQTT(mqttTopicAlarmStatus + "2", ALARM_STATUS_DISARMED, true);
    }
  }  
}

void closeConnection() {
  byte closeConnectionRequest[FIXED_MESSAGE_SIZE] = {};
  byte checksum;
  for (int x = 0; x < FIXED_MESSAGE_SIZE; x++) {
    closeConnectionRequest[x] = 0x00;
  }

  closeConnectionRequest[0] = COMMAND_CLOSE_CONNECTION;
  closeConnectionRequest[2] = 0x05; // Validation byte
  closeConnectionRequest[33] = SOURCE_ID_WINLOAD_DIRECT;
  closeConnectionRequest[34] = 0x00; // User ID high byte
  closeConnectionRequest[35] = 0x00; // User ID low byte

  checksum = 0;
  for (int x = 0; x < FIXED_MESSAGE_SIZE - 1; x++) {
    checksum += closeConnectionRequest[x];
  }

  while (checksum > 255) {
    checksum = checksum - (checksum / 256) * 256;
  }

  closeConnectionRequest[36] = checksum & 0xFF;

  Serial.write(closeConnectionRequest, FIXED_MESSAGE_SIZE);
  readMessageFromSerial();
}

void processEventMessage() {
  if ((responseMessage[0] & 0xF0) == COMMAND_LIVE_EVENT) {
    byte command = responseMessage[0];
    byte eventGroupNumber = responseMessage[7];
    byte eventSubGroupNumber = responseMessage[8];
    byte partition = responseMessage[9];
    String label = "";
    if (responseMessage[14] != 1) {
      for (int k = 15; k <= 30; k++) {
        label = label + String(responseMessage[k]);
      }
      label.trim();
    }    
    sendJsonString(command, eventGroupNumber, eventSubGroupNumber, partition, label);
    if (eventGroupNumber == EVENT_GROUP_SPECIAL && eventSubGroupNumber == SPECIAL_SOFTWARE_LOG_OFF) {
      pannelConnected = false;
    } else if (eventGroupNumber == EVENT_GROUP_SPECIAL && eventSubGroupNumber == SPECIAL_SOFTWARE_LOG_ON && !pannelConnected) {
      pannelConnected = true;
    }
  }
}

void readMessageFromSerial() {
  while (Serial.available() < FIXED_MESSAGE_SIZE) {
    yield(); 
  }

  byte positionIndex = 0;

  while (positionIndex < FIXED_MESSAGE_SIZE) {
    responseMessage[positionIndex++] = Serial.read();
  }

  responseMessage[++positionIndex] = 0x00;
}

void sendJsonString (byte command, byte eventGroupNumber, byte eventSubGroupNumber, byte partition, String label) {
  if (eventGroupNumber == EVENT_GROUP_PARTITION_STATUS) {
    if (eventSubGroupNumber == PARTITION_STATUS_ARM_PARTITION) {
      sendMQTT(mqttTopicAlarmStatus + String(partition + 1), ALARM_STATUS_ARMED_AWAY, true);
      return;
    } else if (eventSubGroupNumber == PARTITION_STATUS_DISARM_PARTITION) {
      sendMQTT(mqttTopicTriggerZone, "0", true);
      sendMQTT(mqttTopicAlarmStatus + String(partition + 1), ALARM_STATUS_DISARMED, true);
      return;
    } else if (eventSubGroupNumber == PARTITION_STATUS_ALARM_STOPPED) {
      sendMQTT(mqttTopicAlarmStatus + String(partition + 1), ALARM_STATUS_ALARM_STOPPED, true);
      return;
    }
  } else if (eventGroupNumber == EVENT_GROUP_SPECIAL_ALARM) {
    if (eventSubGroupNumber == SPECIAL_ALARM_PANIC_NON_MEDICAL) {
      sendMQTT(mqttTopicTriggerZone, "panic", false);
      sendMQTT(mqttTopicAlarmStatus + String(partition + 1), ALARM_STATUS_ALARM_TRIGGERED, true);
      return;
    }
  } else if (eventGroupNumber == EVENT_GROUP_ZOME_IN_ALARM) {
    sendMQTT(mqttTopicEventZone + String(eventSubGroupNumber), "1", true);
    sendMQTT(mqttTopicTriggerZone, String(eventSubGroupNumber), false);
    sendMQTT(mqttTopicAlarmStatus + String(partition + 1), ALARM_STATUS_ALARM_TRIGGERED, true);
    return;
  } else if (eventGroupNumber == EVENT_GROUP_ZONE_ALARM_RESTORE) {
    sendMQTT(mqttTopicEventZone + String(eventSubGroupNumber), "0", true);
    return;
  } else if (eventGroupNumber == EVENT_GROUP_ZONE_CLOSED || eventGroupNumber == EVENT_GROUP_ZONE_OPEN) {
    sendMQTT(mqttTopicEventZone + String(eventSubGroupNumber), String(eventGroupNumber), true);
    return;
  }

  char commandHex[6];
  sprintf(commandHex, "%02X", command);  
  String liveEvent = "{ \"command\":" + String(commandHex) + ", \"eventGroupNumber\":" + String(eventGroupNumber) + ", \"eventSubGroupNumber\":" + String(eventSubGroupNumber) + ", \"partition\":" + String(partition) + ", \"label\":\"" + String(label) + "\"}";
  sendMQTT(mqttTopicEvent, liveEvent);
}

void sendMQTT(String topicNameSend, String dataStr) {
  sendMQTT(topicNameSend, dataStr, false);
}

void sendMQTT(String topicNameSend, String dataStr, boolean retained) {
  handleMqttKeepAlive();
  char topicStrSend[40];
  topicNameSend.toCharArray(topicStrSend, 40);
  char dataStrSend[200];
  dataStr.toCharArray(dataStrSend, 200);
  if (!client.publish(topicStrSend, dataStrSend, retained)) {
    trc("Message not published");
  }
}

void handleMqttKeepAlive() {
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
}

void flushSerialBuffer() {
  Serial.flush();
  while (Serial.read() >= 0)
  {
    ArduinoOTA.handle();
    client.loop();
  }
}

void trc(String msg) {
  // Serial.println(msg);
}

void blink(int duration) {
  digitalWrite(LED_BUILTIN, LOW);
  delay(duration);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(duration);
}

